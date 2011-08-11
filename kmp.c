/*
 * KMP : K Movie Player based on FFplay
 * Copyright (c) 2007-2011 KO Myung-Hun <komh@chollian.net>
 *
 * This file is part of KMP.
 *
 * ffplay : Simple Media Player based on the FFmpeg libraries
 * Copyright (c) 2003 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"
#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include "libavutil/avstring.h"
#include "libavutil/colorspace.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/avassert.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavcodec/audioconvert.h"
#include "libavutil/opt.h"
#include "libavcodec/avfft.h"

#if CONFIG_AVFILTER
# include "libavfilter/avcodec.h"
# include "libavfilter/avfilter.h"
# include "libavfilter/avfiltergraph.h"
# include "libavfilter/vsink_buffer.h"
#endif

#include "cmdutils.h"

#define INCL_DOS
#define INCL_WIN
#define INCL_GPI
#include <os2.h>

#include <stdio.h>
#include <stdlib.h>

#include "version.h"
#include "libavcodec/dsputil.h"
#include "libvo/cpudetect.h"
#include "libvo/fastmemcpy.h"
#include "libvo/mp_msg.h"

#include "kai.h"

#include "kmp.h"
#include "kmp_playlist.h"

static void MorphToPM( void )
{
    PPIB pib;
    PTIB tib;

    DosGetInfoBlocks(&tib, &pib);

    // Change flag from VIO to PM
    if (pib->pib_ultype==2) pib->pib_ultype = 3;
}

#include <unistd.h>
#include <assert.h>

const char program_name[] = "KMP based on ffplay";
const int program_birth_year = 2007;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_AUDIOQ_SIZE (20 * 16 * 1024)
#define MIN_FRAMES 5

/* no AV sync correction is done if below the AV sync threshold */
#define AV_SYNC_THRESHOLD 0.01
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

#define FRAME_SKIP_FACTOR 0.05

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
#define SAMPLE_ARRAY_SIZE (2*65536)

// KOMH: moved sws_flags to kmp_img.c

#define COLOR_FONT      0xFFFFFF
//#define COLOR_OUTLINE   0x010101
//#define COLOR_SHADOW    0x010101
#define COLOR_OUTLINE   0x0F0F0F
#define COLOR_SHADOW    0x0F0F0F
#define COLOR_OVERLAY   0x000008

#define SUB_MARGIN_LEFT         16
#define SUB_MARGIN_RIGHT        16
#define SUB_MARGIN_BOTTOM( h )  (( h ) / 20 )

#define OSD_MARGIN_LEFT         16
#define OSD_MARGIN_RIGHT        16
#define OSD_MARGIN_TOP( h )     (( h ) / 10 )

typedef struct PacketQueue {
    AVPacketList *first_pkt, *last_pkt;
    int nb_packets;
    int size;
    int abort_request;
    Mutex mutex;
    Cond cond;
} PacketQueue;

#define VIDEO_PICTURE_QUEUE_SIZE 2
#define SUBPICTURE_QUEUE_SIZE 4

typedef struct VideoPicture {
    double pts;                                  ///<presentation time stamp for this picture
    double target_clock;                         ///<av_gettime() time at which this should be displayed ideally
    int64_t pos;                                 ///<byte position in file
    YUV *bmp;
    int width, height; /* source height & width */
    int allocated;
    enum PixelFormat pix_fmt;

#if CONFIG_AVFILTER
    AVFilterBufferRef *picref;
#endif
} VideoPicture;

typedef struct SubPicture {
    double pts; /* presentation time stamp for this picture */
    AVSubtitle sub;
} SubPicture;

enum {
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct VideoState {
    Thread read_tid;
    Thread video_tid;
    Thread refresh_tid;
    AVInputFormat *iformat;
    int no_background;
    int abort_request;
    int paused;
    int last_paused;
    int seek_req;
    int seek_flags;
    int64_t seek_pos;
    int64_t seek_rel;
    int read_pause_return;
    AVFormatContext *ic;

    int audio_stream;

    int av_sync_type;
    double external_clock; /* external clock base */
    int64_t external_clock_time;

    double audio_clock;
    double audio_diff_cum; /* used for AV difference average computation */
    double audio_diff_avg_coef;
    double audio_diff_threshold;
    int audio_diff_avg_count;
    AVStream *audio_st;
    PacketQueue audioq;
    int audio_hw_buf_size;
    /* samples output by the codec. we reserve more space for avsync
       compensation */
    DECLARE_ALIGNED(16,uint8_t,audio_buf1)[(AVCODEC_MAX_AUDIO_FRAME_SIZE * 3) / 2];
    DECLARE_ALIGNED(16,uint8_t,audio_buf2)[(AVCODEC_MAX_AUDIO_FRAME_SIZE * 3) / 2];
    uint8_t *audio_buf;
    unsigned int audio_buf_size; /* in bytes */
    int audio_buf_index; /* in bytes */
    AVPacket audio_pkt_temp;
    AVPacket audio_pkt;
    enum AVSampleFormat audio_src_fmt;
    AVAudioConvert *reformat_ctx;

    enum ShowMode {
        SHOW_MODE_NONE = -1, SHOW_MODE_VIDEO = 0, SHOW_MODE_WAVES, SHOW_MODE_RDFT, SHOW_MODE_NB
    } show_mode;
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    int last_i_start;
    RDFTContext *rdft;
    int rdft_bits;
    FFTSample *rdft_data;
    int xpos;

    Thread subtitle_tid;
    int subtitle_stream;
    int subtitle_stream_changed;
    AVStream *subtitle_st;
    PacketQueue subtitleq;
    SubPicture subpq[SUBPICTURE_QUEUE_SIZE];
    int subpq_size, subpq_rindex, subpq_windex;
    Mutex subpq_mutex;
    Cond subpq_cond;

    double frame_timer;
    double frame_last_pts;
    double frame_last_delay;
    double video_clock;                          ///<pts of last decoded frame / predicted pts of next decoded frame
    int video_stream;
    AVStream *video_st;
    PacketQueue videoq;
    double video_current_pts;                    ///<current displayed pts (different from video_clock if frame fifos are used)
    double video_current_pts_drift;              ///<video_current_pts - time (av_gettime) at which we updated video_current_pts - used to have running video pts
    int64_t video_current_pos;                   ///<current displayed file pos
    VideoPicture pictq[VIDEO_PICTURE_QUEUE_SIZE];
    int pictq_size, pictq_rindex, pictq_windex;
    Mutex pictq_mutex;
    Cond pictq_cond;
    struct SwsContext *img_convert_ctx;

    char filename[1024];
    int width, height, xleft, ytop;

#if CONFIG_AVFILTER
    AVFilterContext *out_video_filter;          ///<the last filter in the video chain
#endif

    float skip_frames;
    float skip_frames_index;
    int refresh;

    Thread audio_tid;
    int use_sub;
    ReSampleContext *rsc;
    int paint_in_pause;
    Mutex audio_mutex;
    Cond audio_cond;
    int audio_ok_to_start;
    int video_opened;
    HKAI hkai;
} VideoState;

static int opt_help(const char *opt, const char *arg);

static double get_master_clock(VideoState *is);

#define RES_NONE    0
#define RES_48KONLY 1
#define RES_ALL     2
#define RES_AUTO    3

/* options specified by the user */
static AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;
static int fs_screen_width;
static int fs_screen_height;
static int opt_screen_width = 0;
static int opt_screen_height = 0;
static int screen_width = 0;
static int screen_height = 0;
static int audio_disable;
static int video_disable;
static int opt_wanted_stream[AVMEDIA_TYPE_NB]={
    [AVMEDIA_TYPE_AUDIO]=-1,
    [AVMEDIA_TYPE_VIDEO]=-1,
    [AVMEDIA_TYPE_SUBTITLE]=-1,
};
static int wanted_stream[AVMEDIA_TYPE_NB]={
    [AVMEDIA_TYPE_AUDIO]=-1,
    [AVMEDIA_TYPE_VIDEO]=-1,
    [AVMEDIA_TYPE_SUBTITLE]=-1,
};
static int seek_by_bytes=-1;
static int display_disable;
static int show_status = 1;
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE;
static int64_t duration = AV_NOPTS_VALUE;
static int step = 0;
static int thread_count = 1;
static int workaround_bugs = 1;
static int fast = 0;
static int genpts = 0;
static int lowres = 0;
static int idct = FF_IDCT_AUTO;
static enum AVDiscard skip_frame= AVDISCARD_DEFAULT;
static enum AVDiscard skip_idct= AVDISCARD_DEFAULT;
static enum AVDiscard skip_loop_filter= AVDISCARD_DEFAULT;
static int error_recognition = FF_ER_CAREFUL;
static int error_concealment = 3;
static int decoder_reorder_pts= -1;
static int autoexit;
static int exit_on_keydown;
static int exit_on_mousedown;
static int loop=1;
static int framedrop=-1;
static enum ShowMode show_mode = SHOW_MODE_NONE;

static int rdftspeed=20;
#if CONFIG_AVFILTER
static char *vfilters = NULL;
#endif

/* current context */
static int is_full_screen;
static VideoState *cur_stream;
static int64_t audio_callback_time;

static AVPacket flush_pkt;

static int video_driver = KVAM_AUTO;
static int audio_driver = KAIM_AUTO;
static int volume_level = 80;
static char sub_fontnamesize[ FACESIZE + 5 ] = "24.±¼¸²";
static char osd_fontnamesize[ FACESIZE + 5 ] = "13.System VIO";
static int aspect_ratio = KVAR_ORIGINAL;
static int hide_mouse = 0;
static int resample = RES_NONE;
static int use_subimg = 0;
static int fix_t23 = 0;
static int no_audio_share = 0;
static long sub_color_font = COLOR_FONT;
static long sub_color_outline = COLOR_OUTLINE;
static long sub_color_shadow = COLOR_SHADOW;
static long osd_color_font = COLOR_FONT;
static long osd_color_outline = COLOR_OUTLINE;
static long osd_color_shadow = COLOR_SHADOW;
static int fix_snap = 0;
static int audio_buffer_size = 0;
static int auto_add = 0;
static int seek_interval = 100;

#define WM_ALLOC_EVENT      ( WM_USER + 1 )
#define WM_REFRESH_EVENT    ( WM_USER + 2 )

#define KMP_NAME    "K Movie Player"
#define KMP_VERSION "v0.6.3"

#define TID_SECOND      ( TID_USERMAX - 1 )
#define TID_INFORM      ( TID_USERMAX - 2 )
#define TID_MOUSE_HIDE  ( TID_USERMAX - 3 )

#define INTERVAL_SECOND     1000
#define INTERVAL_INFORM     1500
#define INTERVAL_MOUSE_HIDE 1500

static HAB  hab;
static HWND hwndFrame;
static HWND hwndSysMenu;
static HWND hwndTitleBar;
static HWND hwndMinMax;
static HWND hwndKMP;

static BOOL  fMouseShow = TRUE;
static ULONG aulAttrValue[ KVAA_LAST ] = { -1, -1, -1, -1, -1, };

static LONG  lImgWidth = 0;
static LONG  lImgHeight = 0;

static PFNWP pfnwpOldFrame = NULL;

#define TOLERANCE   3

static PPLAYLIST ppl = NULL;

#define PLAYLIST_PREV_FILE  1
#define PLAYLIST_NEXT_FILE  2
#define PLAYLIST_QUIT       3

static int cmdPlaylist;

static double audio_clock_delta = 0;

#define SHOW_POINTER( fShow ) \
{\
    WinShowPointer( HWND_DESKTOP, fShow );\
    fMouseShow = fShow;\
}

static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
    AVPacketList *pkt1;

    /* duplicate the packet */
    if (pkt!=&flush_pkt && av_dup_packet(pkt) < 0)
        return -1;

    pkt1 = av_malloc(sizeof(AVPacketList));
    if (!pkt1)
        return -1;
    pkt1->pkt = *pkt;
    pkt1->next = NULL;


    LockMutex(q->mutex);

    if (!q->last_pkt)

        q->first_pkt = pkt1;
    else
        q->last_pkt->next = pkt1;
    q->last_pkt = pkt1;
    q->nb_packets++;
    q->size += pkt1->pkt.size + sizeof(*pkt1);
    /* XXX: should duplicate packet data in DV case */
    CondSignal(q->cond);

    UnlockMutex(q->mutex);
    return 0;
}

/* packet queue handling */
static void packet_queue_init(PacketQueue *q)
{
    memset(q, 0, sizeof(PacketQueue));
    q->mutex = CreateMutex();
    q->cond = CreateCond();
    packet_queue_put(q, &flush_pkt);
}

static void packet_queue_flush(PacketQueue *q)
{
    AVPacketList *pkt, *pkt1;

    LockMutex(q->mutex);
    for(pkt = q->first_pkt; pkt != NULL; pkt = pkt1) {
        pkt1 = pkt->next;
        av_free_packet(&pkt->pkt);
        av_freep(&pkt);
    }
    q->last_pkt = NULL;
    q->first_pkt = NULL;
    q->nb_packets = 0;
    q->size = 0;
    UnlockMutex(q->mutex);
}

static void packet_queue_end(PacketQueue *q)
{
    packet_queue_flush(q);
    DestroyMutex(q->mutex);
    DestroyCond(q->cond);
}

static void packet_queue_abort(PacketQueue *q)
{
    LockMutex(q->mutex);

    q->abort_request = 1;

    CondSignal(q->cond);

    UnlockMutex(q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block)
{
    AVPacketList *pkt1;
    int ret;

    LockMutex(q->mutex);

    for(;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }

        pkt1 = q->first_pkt;
        if (pkt1) {
            q->first_pkt = pkt1->next;
            if (!q->first_pkt)
                q->last_pkt = NULL;
            q->nb_packets--;
            q->size -= pkt1->pkt.size + sizeof(*pkt1);
            *pkt = pkt1->pkt;
            av_free(pkt1);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            CondWait(q->cond, q->mutex);
        }
    }
    UnlockMutex(q->mutex);
    return ret;
}

static inline void fill_rectangle( HPS hps,
                                   int x, int y, int w, int h, int color )
{
    RECTL   rcl;

    rcl.xLeft = x;
    rcl.yBottom = y;
    rcl.xRight = x + w;
    rcl.yTop = y + h;
    WinFillRect( hps, &rcl, color );
}

#define ALPHA_BLEND(a, oldp, newp, s)\
((((oldp << s) * (255 - (a))) + (newp * (a))) / (255 << s))

#define RGBA_IN(r, g, b, a, s)\
{\
    unsigned int v = ((const uint32_t *)(s))[0];\
    a = (v >> 24) & 0xff;\
    r = (v >> 16) & 0xff;\
    g = (v >> 8) & 0xff;\
    b = v & 0xff;\
}

#define YUVA_IN(y, u, v, a, s, pal)\
{\
    unsigned int val = ((const uint32_t *)(pal))[*(const uint8_t*)(s)];\
    a = (val >> 24) & 0xff;\
    y = (val >> 16) & 0xff;\
    u = (val >> 8) & 0xff;\
    v = val & 0xff;\
}

#define YUVA_OUT(d, y, u, v, a)\
{\
    ((uint32_t *)(d))[0] = (a << 24) | (y << 16) | (u << 8) | v;\
}


#define BPP 1

static void blend_subrect(AVPicture *dst, const AVSubtitleRect *rect, int imgw, int imgh)
{
    int wrap, wrap3, width2, skip2;
    int y, u, v, a, u1, v1, a1, w, h;
    uint8_t *lum, *cb, *cr;
    const uint8_t *p;
    const uint32_t *pal;
    int dstx, dsty, dstw, dsth;

    dstw = av_clip(rect->w, 0, imgw);
    dsth = av_clip(rect->h, 0, imgh);
    dstx = av_clip(rect->x, 0, imgw - dstw);
    dsty = av_clip(rect->y, 0, imgh - dsth);
    lum = dst->data[0] + dsty * dst->linesize[0];
    cb = dst->data[1] + (dsty >> 1) * dst->linesize[1];
    cr = dst->data[2] + (dsty >> 1) * dst->linesize[2];

    width2 = ((dstw + 1) >> 1) + (dstx & ~dstw & 1);
    skip2 = dstx >> 1;
    wrap = dst->linesize[0];
    wrap3 = rect->pict.linesize[0];
    p = rect->pict.data[0];
    pal = (const uint32_t *)rect->pict.data[1];  /* Now in YCrCb! */

    if (dsty & 1) {
        lum += dstx;
        cb += skip2;
        cr += skip2;

        if (dstx & 1) {
            YUVA_IN(y, u, v, a, p, pal);
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            cb[0] = ALPHA_BLEND(a >> 2, cb[0], u, 0);
            cr[0] = ALPHA_BLEND(a >> 2, cr[0], v, 0);
            cb++;
            cr++;
            lum++;
            p += BPP;
        }
        for(w = dstw - (dstx & 1); w >= 2; w -= 2) {
            YUVA_IN(y, u, v, a, p, pal);
            u1 = u;
            v1 = v;
            a1 = a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);

            YUVA_IN(y, u, v, a, p + BPP, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[1] = ALPHA_BLEND(a, lum[1], y, 0);
            cb[0] = ALPHA_BLEND(a1 >> 2, cb[0], u1, 1);
            cr[0] = ALPHA_BLEND(a1 >> 2, cr[0], v1, 1);
            cb++;
            cr++;
            p += 2 * BPP;
            lum += 2;
        }
        if (w) {
            YUVA_IN(y, u, v, a, p, pal);
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            cb[0] = ALPHA_BLEND(a >> 2, cb[0], u, 0);
            cr[0] = ALPHA_BLEND(a >> 2, cr[0], v, 0);
            p++;
            lum++;
        }
        p += wrap3 - dstw * BPP;
        lum += wrap - dstw - dstx;
        cb += dst->linesize[1] - width2 - skip2;
        cr += dst->linesize[2] - width2 - skip2;
    }
    for(h = dsth - (dsty & 1); h >= 2; h -= 2) {
        lum += dstx;
        cb += skip2;
        cr += skip2;

        if (dstx & 1) {
            YUVA_IN(y, u, v, a, p, pal);
            u1 = u;
            v1 = v;
            a1 = a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            p += wrap3;
            lum += wrap;
            YUVA_IN(y, u, v, a, p, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            cb[0] = ALPHA_BLEND(a1 >> 2, cb[0], u1, 1);
            cr[0] = ALPHA_BLEND(a1 >> 2, cr[0], v1, 1);
            cb++;
            cr++;
            p += -wrap3 + BPP;
            lum += -wrap + 1;
        }
        for(w = dstw - (dstx & 1); w >= 2; w -= 2) {
            YUVA_IN(y, u, v, a, p, pal);
            u1 = u;
            v1 = v;
            a1 = a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);

            YUVA_IN(y, u, v, a, p + BPP, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[1] = ALPHA_BLEND(a, lum[1], y, 0);
            p += wrap3;
            lum += wrap;

            YUVA_IN(y, u, v, a, p, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);

            YUVA_IN(y, u, v, a, p + BPP, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[1] = ALPHA_BLEND(a, lum[1], y, 0);

            cb[0] = ALPHA_BLEND(a1 >> 2, cb[0], u1, 2);
            cr[0] = ALPHA_BLEND(a1 >> 2, cr[0], v1, 2);

            cb++;
            cr++;
            p += -wrap3 + 2 * BPP;
            lum += -wrap + 2;
        }
        if (w) {
            YUVA_IN(y, u, v, a, p, pal);
            u1 = u;
            v1 = v;
            a1 = a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            p += wrap3;
            lum += wrap;
            YUVA_IN(y, u, v, a, p, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            cb[0] = ALPHA_BLEND(a1 >> 2, cb[0], u1, 1);
            cr[0] = ALPHA_BLEND(a1 >> 2, cr[0], v1, 1);
            cb++;
            cr++;
            p += -wrap3 + BPP;
            lum += -wrap + 1;
        }
        p += wrap3 + (wrap3 - dstw * BPP);
        lum += wrap + (wrap - dstw - dstx);
        cb += dst->linesize[1] - width2 - skip2;
        cr += dst->linesize[2] - width2 - skip2;
    }
    /* handle odd height */
    if (h) {
        lum += dstx;
        cb += skip2;
        cr += skip2;

        if (dstx & 1) {
            YUVA_IN(y, u, v, a, p, pal);
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            cb[0] = ALPHA_BLEND(a >> 2, cb[0], u, 0);
            cr[0] = ALPHA_BLEND(a >> 2, cr[0], v, 0);
            cb++;
            cr++;
            lum++;
            p += BPP;
        }
        for(w = dstw - (dstx & 1); w >= 2; w -= 2) {
            YUVA_IN(y, u, v, a, p, pal);
            u1 = u;
            v1 = v;
            a1 = a;
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);

            YUVA_IN(y, u, v, a, p + BPP, pal);
            u1 += u;
            v1 += v;
            a1 += a;
            lum[1] = ALPHA_BLEND(a, lum[1], y, 0);
            cb[0] = ALPHA_BLEND(a1 >> 2, cb[0], u, 1);
            cr[0] = ALPHA_BLEND(a1 >> 2, cr[0], v, 1);
            cb++;
            cr++;
            p += 2 * BPP;
            lum += 2;
        }
        if (w) {
            YUVA_IN(y, u, v, a, p, pal);
            lum[0] = ALPHA_BLEND(a, lum[0], y, 0);
            cb[0] = ALPHA_BLEND(a >> 2, cb[0], u, 0);
            cr[0] = ALPHA_BLEND(a >> 2, cr[0], v, 0);
        }
    }
}

static void free_subpicture(SubPicture *sp)
{
    avsubtitle_free(&sp->sub);
}

static void video_image_display(VideoState *is)
{
    VideoPicture *vp;
    SubPicture *sp;
    int i;
    HPS hps = NULLHANDLE; // to avoid 'uninitialized' warning
    int w, h;

    vp = &is->pictq[is->pictq_rindex];
    if (vp->bmp) {
        // KOMH: removed the codes for an aspect ratio of pixel.
        // I don't care about it

        if (is->subtitle_st) {
            if (is->subpq_size > 0) {
                sp = &is->subpq[is->subpq_rindex];

                if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
                    for (i = 0; i < sp->sub.num_rects; i++)
                        blend_subrect(( AVPicture * )vp->bmp, sp->sub.rects[i],
                                      lImgWidth, lImgHeight);
                }
            }
        }

        if( !use_subimg )
        {
            hps = WinGetPS( hwndKMP );

            if( is->use_sub )
                subErase( hps, COLOR_OVERLAY );

            osdErase( hps, COLOR_OVERLAY );
        }
        else if( !is->paint_in_pause )
        {
            w = lImgWidth;
            h = lImgHeight;

            if( is->use_sub )
            {
                ULONG ulRefClock = get_master_clock( is ) * 100;

                if( subInTime( ulRefClock ))
                    subDisplay( NULLHANDLE, vp->bmp->data[ 0 ], vp->bmp->linesize[ 0 ], h,
                                SUB_MARGIN_LEFT, SUB_MARGIN_BOTTOM( h ),
                                w - ( SUB_MARGIN_LEFT + SUB_MARGIN_RIGHT ),
                                h - SUB_MARGIN_BOTTOM( h ) * 2,
                                sub_color_font, sub_color_outline, sub_color_shadow, TRUE );
                else
                    subFindNext( ulRefClock );
            }

            if( osdOn())
            {
                osdDisplay( NULLHANDLE, vp->bmp->data[ 0 ], vp->bmp->linesize[ 0 ], h,
                            OSD_MARGIN_LEFT, h - OSD_MARGIN_TOP( h ),
                            w - ( OSD_MARGIN_LEFT + OSD_MARGIN_RIGHT ),
                            h - OSD_MARGIN_TOP( h ) * 2,
                            osd_color_font, osd_color_outline, osd_color_shadow, TRUE );
            }
        }

        imgDisplayYUV(( YUV * )vp->bmp );

        if( !use_subimg )
        {
            w = is->width;
            h = is->height;

            if( is->use_sub )
            {
                ULONG ulRefClock = get_master_clock( is ) * 100;

                if( subInTime( ulRefClock ))
                    subDisplay( hps, NULL, 0, 0,
                                SUB_MARGIN_LEFT, SUB_MARGIN_BOTTOM( h ),
                                w - ( SUB_MARGIN_LEFT + SUB_MARGIN_RIGHT ),
                                h - SUB_MARGIN_BOTTOM( h ) * 2,
                                sub_color_font, sub_color_outline, sub_color_shadow, TRUE );
                else
                    subFindNext( ulRefClock );
            }

            if( osdOn())
                osdDisplay( hps, NULL, 0, 0,
                            OSD_MARGIN_LEFT, h - OSD_MARGIN_TOP( h ),
                            w - ( OSD_MARGIN_LEFT + OSD_MARGIN_RIGHT ),
                            h - OSD_MARGIN_TOP( h ) * 2,
                            osd_color_font, osd_color_outline, osd_color_shadow, TRUE );

            WinReleasePS( hps );
        }
    }
}

/* get the current audio output buffer size, in samples. With SDL, we
   cannot have a precise information */
static int audio_write_get_buf_size(VideoState *is)
{
    return is->audio_buf_size - is->audio_buf_index;
}

static inline int compute_mod(int a, int b)
{
    return a < 0 ? a%b + b : a%b;
}

#define MapRGB( r, g, b ) ((( r ) << 16 ) | (( g ) << 8 ) | b )

static void video_audio_display(VideoState *s)
{
    int i, i_start, x, y1, y, ys, delay, n, nb_display_channels;
    int ch, channels, h, h2, bgcolor, fgcolor;
    int16_t time_diff;
    HPS hps;
    int rdft_bits, nb_freq;

    for(rdft_bits=1; (1<<rdft_bits)<2*s->height; rdft_bits++)
        ;
    nb_freq= 1<<(rdft_bits-1);

    /* compute display index : center on currently output samples */
    channels = s->audio_st->codec->channels;
    nb_display_channels = channels;
    if (!s->paused) {
        int data_used= s->show_mode == SHOW_MODE_WAVES ? s->width : (2*nb_freq);
        n = 2 * channels;
        delay = audio_write_get_buf_size(s);
        delay /= n;

        /* to be more precise, we take into account the time spent since
           the last buffer computation */
        if (audio_callback_time) {
            time_diff = av_gettime() - audio_callback_time;
            delay -= (time_diff * s->audio_st->codec->sample_rate) / 1000000;
        }

        delay += 2*data_used;
        if (delay < data_used)
            delay = data_used;

        i_start= x = compute_mod(s->sample_array_index - delay * channels, SAMPLE_ARRAY_SIZE);
        if (s->show_mode == SHOW_MODE_WAVES) {
            h= INT_MIN;
            for(i=0; i<1000; i+=channels){
                int idx= (SAMPLE_ARRAY_SIZE + x - i) % SAMPLE_ARRAY_SIZE;
                int a= s->sample_array[idx];
                int b= s->sample_array[(idx + 4*channels)%SAMPLE_ARRAY_SIZE];
                int c= s->sample_array[(idx + 5*channels)%SAMPLE_ARRAY_SIZE];
                int d= s->sample_array[(idx + 9*channels)%SAMPLE_ARRAY_SIZE];
                int score= a-d;
                if(h<score && (b^c)<0){
                    h= score;
                    i_start= idx;
                }
            }
        }

        s->last_i_start = i_start;
    } else {
        i_start = s->last_i_start;
    }

    hps = WinGetPS( hwndKMP );

    osdErase( hps, COLOR_OVERLAY );

    GpiCreateLogColorTable( hps, 0, LCOLF_RGB, 0, 0, NULL );

    bgcolor = MapRGB( 0x00, 0x00, 0x00 );
    if (s->show_mode == SHOW_MODE_WAVES) {
        fill_rectangle( hps,
                        s->xleft, s->ytop, s->width, s->height,
                        bgcolor );

        fgcolor = MapRGB( 0xff, 0xff, 0xff );

        /* total height for one channel */
        h = s->height / nb_display_channels;
        /* graph height / 2 */
        h2 = (h * 9) / 20;
        for(ch = 0;ch < nb_display_channels; ch++) {
            i = i_start + ch;
            y1 = s->ytop + ch * h + (h / 2); /* position of center line */
            for(x = 0; x < s->width; x++) {
                y = (s->sample_array[i] * h2) >> 15;
                if (y < 0) {
                    y = -y;
                    ys = y1 - y;
                } else {
                    ys = y1;
                }
                fill_rectangle( hps,
                                s->xleft + x, ys, 1, y,
                                fgcolor );
                i += channels;
                if (i >= SAMPLE_ARRAY_SIZE)
                    i -= SAMPLE_ARRAY_SIZE;
            }
        }

        fgcolor = MapRGB( 0x00, 0x00, 0xff );

        for(ch = 1;ch < nb_display_channels; ch++) {
            y = s->ytop + ch * h;
            fill_rectangle( hps,
                            s->xleft, y, s->width, 1,
                            fgcolor );
        }
    }else{
        nb_display_channels= FFMIN(nb_display_channels, 2);
        if(rdft_bits != s->rdft_bits){
            av_rdft_end(s->rdft);
            av_free(s->rdft_data);
            s->rdft = av_rdft_init(rdft_bits, DFT_R2C);
            s->rdft_bits= rdft_bits;
            s->rdft_data= av_malloc(4*nb_freq*sizeof(*s->rdft_data));
        }
        {
            FFTSample *data[2];
            for(ch = 0;ch < nb_display_channels; ch++) {
                data[ch] = s->rdft_data + 2*nb_freq*ch;
                i = i_start + ch;
                for(x = 0; x < 2*nb_freq; x++) {
                    double w= (x-nb_freq)*(1.0/nb_freq);
                    data[ch][x]= s->sample_array[i]*(1.0-w*w);
                    i += channels;
                    if (i >= SAMPLE_ARRAY_SIZE)
                        i -= SAMPLE_ARRAY_SIZE;
                }
                av_rdft_calc(s->rdft, data[ch]);
            }
            //least efficient way to do this, we should of course directly access it but its more than fast enough
            for(y=0; y<s->height; y++){
                double w= 1/sqrt(nb_freq);
                int a= sqrt(w*sqrt(data[0][2*y+0]*data[0][2*y+0] + data[0][2*y+1]*data[0][2*y+1]));
                int b= (nb_display_channels == 2 ) ? sqrt(w*sqrt(data[1][2*y+0]*data[1][2*y+0]
                       + data[1][2*y+1]*data[1][2*y+1])) : a;
                a= FFMIN(a,255);
                b= FFMIN(b,255);
                fgcolor = MapRGB(a, b, (a+b)/2);

                fill_rectangle(hps,
                               s->xpos, s->height-y, 1, 1,
                               fgcolor);
            }
        }
        s->xpos++;
        if(s->xpos >= s->width)
            s->xpos= s->xleft;
    }


    if( osdOn())
        osdDisplay( hps, NULL, 0, 0,
                    OSD_MARGIN_LEFT, s->height - OSD_MARGIN_TOP( s->height ),
                    s->width - ( OSD_MARGIN_LEFT + OSD_MARGIN_RIGHT ),
                    s->height - OSD_MARGIN_TOP( s->height ) * 2,
                    osd_color_font, osd_color_outline, osd_color_shadow, TRUE );

    WinReleasePS( hps );
}

static void stream_close(VideoState *is)
{
    VideoPicture *vp;
    int i;
    /* XXX: use a special url_shutdown call to abort parse cleanly */
    is->abort_request = 1;
    WaitThread( is->read_tid );
    WaitThread( is->refresh_tid );

    /* free all pictures */
    for(i=0;i<VIDEO_PICTURE_QUEUE_SIZE; i++) {
        vp = &is->pictq[i];
#if CONFIG_AVFILTER
        if (vp->picref) {
            avfilter_unref_buffer(vp->picref);
            vp->picref = NULL;
        }
#endif
        if (vp->bmp) {
            imgFreeYUV( vp->bmp );
            vp->bmp = NULL;
        }
    }

    tmrDone();

    DestroyMutex(is->pictq_mutex);
    DestroyCond(is->pictq_cond);
    DestroyMutex(is->subpq_mutex);
    DestroyCond(is->subpq_cond);
    if (is->img_convert_ctx)
        sws_freeContext(is->img_convert_ctx);
    av_free(is);
}

// KOMH: removed, instead splited it into main()
// static void do_exit(void)

static int video_open(VideoState *is){
    //ULONG   fl = SWP_MOVE | SWP_SIZE | SWP_ZORDER | SWP_ACTIVATE | SWP_SHOW | SWP_RESTORE;
    ULONG   fl = SWP_MOVE | SWP_SIZE | SWP_ZORDER | SWP_ACTIVATE | SWP_SHOW;
    RECTL   rcl;
    int     w, h;
    int     x, y;

    // process child window
    if( is_full_screen )
    {
        // when -fs option is used without this, title bar is not highlighted
        WinSetActiveWindow( HWND_DESKTOP, hwndFrame );

        WinSetParent( hwndSysMenu, HWND_OBJECT, FALSE );
        WinSetParent( hwndTitleBar, HWND_OBJECT, FALSE );
        WinSetParent( hwndMinMax, HWND_OBJECT, FALSE );

        if( hide_mouse )
            WinStartTimer( hab, hwndKMP, TID_MOUSE_HIDE, INTERVAL_MOUSE_HIDE );
    }
    else
    {
        WinSetParent( hwndSysMenu, hwndFrame, FALSE );
        WinSetParent( hwndTitleBar, hwndFrame, FALSE );
        WinSetParent( hwndMinMax, hwndFrame, FALSE );

        if( hide_mouse && !fMouseShow )
            SHOW_POINTER( TRUE );
    }
    //WinSendMsg( hwndFrame, WM_UPDATEFRAME, ( MPARAM )( FCF_SYSMENU | FCF_TITLEBAR | FCF_MINMAX ), 0 );

    // determine width and height of window
    if( is_full_screen )
    {
        w = fs_screen_width;
        h = fs_screen_height;
    }
    else if( screen_width )
    {
        w = screen_width;
        h = screen_height;
#if CONFIG_AVFILTER
    }else if (is->out_video_filter && is->out_video_filter->inputs[0]){
#else
    }else if( is->video_st && is->video_st->codec->width ) {
#endif
        w = lImgWidth;
        h = lImgHeight;

        // workaround for T23 laptop with S3 Video by Franz Bakan
        if( fix_t23 )
        {
            w++;
            h++;
        }
    }
    else
    {
        w = 640;
        h = 480;
    }

    if( window_title )
        WinSetWindowText( hwndTitleBar, window_title );

    rcl.xLeft = 0;
    rcl.yBottom = 0;
    rcl.xRight = w;
    rcl.yTop = h;

    WinCalcFrameRect( hwndFrame, &rcl, FALSE );

    x = ( fs_screen_width - ( rcl.xRight - rcl.xLeft )) / 2;
    y = ( fs_screen_height - ( rcl.yTop - rcl.yBottom )) / 2;

    WinSetWindowPos( hwndFrame, HWND_TOP, x, y,
                     rcl.xRight - rcl.xLeft, rcl.yTop - rcl.yBottom,
                     fl );

    // KOMH: Now, is->width and is->height is set by WM_SIZE
#if 0
    is->width = w;
    is->height = h;
#endif

    if( is->use_sub )
        subInvalidate();

    // display an information about a playing file on OSD
    if( !is->video_opened )
        WinPostMsg( hwndKMP, WM_CHAR, ( MPARAM )KC_CHAR, ( MPARAM )'d');

    is->video_opened = TRUE;

    return 0;
}

/* display the current picture, if any */
static void video_display(VideoState *is)
{
    if( !is->video_opened )
        video_open( is );
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
        video_audio_display(is);
    else if (is->video_st)
        video_image_display(is);
}


static void refresh_thread(void *opaque)
{
    VideoState *is= opaque;
    while(!is->abort_request){
        if(!is->refresh){
            is->refresh=1;
            WinPostMsg( hwndKMP, WM_REFRESH_EVENT, opaque, 0 );
        }
        //FIXME ideally we should wait the correct time but SDLs event passing is so slow it would be silly
        usleep(is->audio_st && is->show_mode != SHOW_MODE_VIDEO ? rdftspeed*1000 : 5000);
    }
    return;
}

/* get the current audio clock value */
static double get_audio_clock(VideoState *is)
{
    double pts;
    int hw_buf_size, bytes_per_sec;
    pts = is->audio_clock + audio_clock_delta;
    hw_buf_size = audio_write_get_buf_size(is);
    bytes_per_sec = 0;
    if (is->audio_st) {
        bytes_per_sec = is->audio_st->codec->sample_rate *
            2 * is->audio_st->codec->channels;
    }
    if (bytes_per_sec)
        pts -= (double)hw_buf_size / bytes_per_sec;
    return pts;
}

/* get the current video clock value */
static double get_video_clock(VideoState *is)
{
    if (is->paused) {
        return is->video_current_pts;
    } else {
        return is->video_current_pts_drift + av_gettime() / 1000000.0;
    }
}

/* get the current external clock value */
static double get_external_clock(VideoState *is)
{
    int64_t ti;
    ti = av_gettime();
    return is->external_clock + ((ti - is->external_clock_time) * 1e-6);
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
    double val;

    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        if (is->video_st)
            val = get_video_clock(is);
        else
            val = get_audio_clock(is);
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        if (is->audio_st)
            val = get_audio_clock(is);
        else
            val = get_video_clock(is);
    } else {
        val = get_external_clock(is);
    }
    return val;
}

/* seek in the stream */
static void stream_seek(VideoState *is, int64_t pos, int64_t rel, int seek_by_bytes)
{
    if (!is->seek_req) {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (seek_by_bytes)
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        is->seek_req = 1;
    }
}

/* pause or resume the video */
static void stream_toggle_pause(VideoState *is)
{
    if (is->paused) {
        is->frame_timer += av_gettime() / 1000000.0 + is->video_current_pts_drift - is->video_current_pts;
        if(is->read_pause_return != AVERROR(ENOSYS)){
            is->video_current_pts = is->video_current_pts_drift + av_gettime() / 1000000.0;
        }
        is->video_current_pts_drift = is->video_current_pts - av_gettime() / 1000000.0;
    }
    is->paused = !is->paused;
}

static double compute_target_time(double frame_current_pts, VideoState *is)
{
    double delay, sync_threshold, diff;

    /* compute nominal delay */
    delay = frame_current_pts - is->frame_last_pts;
    if (delay <= 0 || delay >= 10.0) {
        /* if incorrect delay, use previous one */
        delay = is->frame_last_delay;
    } else {
        is->frame_last_delay = delay;
    }
    is->frame_last_pts = frame_current_pts;

    /* update delay to follow master synchronisation source */
    if (((is->av_sync_type == AV_SYNC_AUDIO_MASTER && is->audio_st) ||
         is->av_sync_type == AV_SYNC_EXTERNAL_CLOCK)) {
        /* if video is slave, we try to correct big delays by
           duplicating or deleting a frame */
        diff = get_video_clock(is) - get_master_clock(is);

        /* skip or repeat frame. We take into account the
           delay to compute the threshold. I still don't know
           if it is the best guess */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD, delay);
        if (fabs(diff) < AV_NOSYNC_THRESHOLD) {
            if (diff <= -sync_threshold)
                delay = 0;
            else if (diff >= sync_threshold)
                delay = 2 * delay;
        }
    }
    is->frame_timer += delay;

    av_dlog(NULL, "video: delay=%0.3f pts=%0.3f A-V=%f\n",
            delay, frame_current_pts, -diff);

    return is->frame_timer;
}

/* called to display each frame */
static void video_refresh(void *opaque)
{
    VideoState *is = opaque;
    VideoPicture *vp;

    SubPicture *sp, *sp2;

    if (is->video_st) {
retry:
        if (is->pictq_size == 0) {
            //nothing to do, no picture to display in the que
        } else {
            double time= av_gettime()/1000000.0;
            double next_target;
            /* dequeue the picture */
            vp = &is->pictq[is->pictq_rindex];

            if(time < vp->target_clock)
                return;
            /* update current video pts */
            is->video_current_pts = vp->pts;
            is->video_current_pts_drift = is->video_current_pts - time;
            is->video_current_pos = vp->pos;
            if(is->pictq_size > 1){
                VideoPicture *nextvp= &is->pictq[(is->pictq_rindex+1)%VIDEO_PICTURE_QUEUE_SIZE];
                assert(nextvp->target_clock >= vp->target_clock);
                next_target= nextvp->target_clock;
            }else{
                next_target= vp->target_clock + is->video_clock - vp->pts; //FIXME pass durations cleanly
            }
            if((framedrop>0 || (framedrop && is->audio_st)) && time > next_target){
                is->skip_frames *= 1.0 + FRAME_SKIP_FACTOR;
                if(is->pictq_size > 1 || time > next_target + 0.5){
                    /* update queue size and signal for next picture */
                    if (++is->pictq_rindex == VIDEO_PICTURE_QUEUE_SIZE)
                        is->pictq_rindex = 0;

                    LockMutex(is->pictq_mutex);
                    is->pictq_size--;
                    CondSignal(is->pictq_cond);
                    UnlockMutex(is->pictq_mutex);
                    goto retry;
                }
            }

            if(is->subtitle_st) {
                if (is->subtitle_stream_changed) {
                    LockMutex(is->subpq_mutex);

                    while (is->subpq_size) {
                        free_subpicture(&is->subpq[is->subpq_rindex]);

                        /* update queue size and signal for next picture */
                        if (++is->subpq_rindex == SUBPICTURE_QUEUE_SIZE)
                            is->subpq_rindex = 0;

                        is->subpq_size--;
                    }
                    is->subtitle_stream_changed = 0;

                    CondSignal(is->subpq_cond);
                    UnlockMutex(is->subpq_mutex);
                } else {
                    if (is->subpq_size > 0) {
                        sp = &is->subpq[is->subpq_rindex];

                        if (is->subpq_size > 1)
                            sp2 = &is->subpq[(is->subpq_rindex + 1) % SUBPICTURE_QUEUE_SIZE];
                        else
                            sp2 = NULL;

                        if ((is->video_current_pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
                                || (sp2 && is->video_current_pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))))
                        {
                            free_subpicture(sp);

                            /* update queue size and signal for next picture */
                            if (++is->subpq_rindex == SUBPICTURE_QUEUE_SIZE)
                                is->subpq_rindex = 0;

                            LockMutex(is->subpq_mutex);
                            is->subpq_size--;
                            CondSignal(is->subpq_cond);
                            UnlockMutex(is->subpq_mutex);
                        }
                    }
                }
            }

            /* display picture */
            if (!display_disable)
                video_display(is);

            /* update queue size and signal for next picture */
            if (++is->pictq_rindex == VIDEO_PICTURE_QUEUE_SIZE)
                is->pictq_rindex = 0;

            LockMutex(is->pictq_mutex);
            is->pictq_size--;
            CondSignal(is->pictq_cond);
            UnlockMutex(is->pictq_mutex);
        }
    } else if (is->audio_st) {
        /* draw the next audio frame */

        /* if only audio stream, then display the audio bars (better
           than nothing, just to test the implementation */

        /* display picture */
        if (!display_disable)
            video_display(is);
    }
    if (show_status) {
        static int64_t last_time;
        int64_t cur_time;
        int aqsize, vqsize, sqsize;
        double av_diff;

        cur_time = av_gettime();
        if (!last_time || (cur_time - last_time) >= 30000) {
            aqsize = 0;
            vqsize = 0;
            sqsize = 0;
            if (is->audio_st)
                aqsize = is->audioq.size;
            if (is->video_st)
                vqsize = is->videoq.size;
            if (is->subtitle_st)
                sqsize = is->subtitleq.size;
            av_diff = 0;
            if (is->audio_st && is->video_st)
                av_diff = get_audio_clock(is) - get_video_clock(is);
            printf("%7.2f A-V:%7.3f s:%3.1f aq=%5dKB vq=%5dKB sq=%5dB f=%"PRId64"/%"PRId64"   \r",
                   get_master_clock(is),
                   av_diff,
                   FFMAX(is->skip_frames-1, 0),
                   aqsize / 1024,
                   vqsize / 1024,
                   sqsize,
                   is->video_st ? is->video_st->codec->pts_correction_num_faulty_dts : 0,
                   is->video_st ? is->video_st->codec->pts_correction_num_faulty_pts : 0);
            fflush(stdout);
            last_time = cur_time;
        }
    }
}

/* allocate a picture (needs to do that in main thread to avoid
   potential locking problems */
static void alloc_picture(void *opaque)
{
    VideoState *is = opaque;
    VideoPicture *vp;

    vp = &is->pictq[is->pictq_windex];

    if (vp->bmp)
        imgFreeYUV(vp->bmp);

    imgDone();

#if CONFIG_AVFILTER
    if (vp->picref)
        avfilter_unref_buffer(vp->picref);
    vp->picref = NULL;

    vp->width   = is->out_video_filter->inputs[0]->w;
    vp->height  = is->out_video_filter->inputs[0]->h;
    vp->pix_fmt = is->out_video_filter->inputs[0]->format;
#else
    vp->width   = is->video_st->codec->width;
    vp->height  = is->video_st->codec->height;
    vp->pix_fmt = is->video_st->codec->pix_fmt;
#endif

    lImgWidth = vp->width;
    lImgHeight = vp->height;

    if( fix_t23 || fix_snap )
    {
        while( lImgWidth > fs_screen_width )
        {
            lImgWidth >>= 1;
            lImgHeight >>= 1;
        }

        while( lImgHeight > fs_screen_height )
        {
            lImgWidth >>= 1;
            lImgHeight >>= 1;
        }
    }

    if( imgInit( video_driver, hwndKMP, COLOR_OVERLAY,
                 lImgWidth, lImgHeight,
                 aspect_ratio, aulAttrValue ))
    {
        fprintf( stderr, "Video setup failed!!!\n");

        LockMutex( is->audio_mutex );
        is->abort_request = 1;
        CondSignal( is->audio_cond );
        UnlockMutex( is->audio_mutex );

        LockMutex(is->pictq_mutex);
        is->videoq.abort_request = 1;
        CondSignal(is->pictq_cond);
        UnlockMutex(is->pictq_mutex);

        return;
    }

    LockMutex( is->audio_mutex );
    is->audio_ok_to_start = 1;
    CondSignal( is->audio_cond );
    UnlockMutex( is->audio_mutex );

    vp->bmp = imgCreateYUV();
    // KOMH: imgCreateYUV() does not fail

    LockMutex(is->pictq_mutex);
    vp->allocated = 1;
    CondSignal(is->pictq_cond);
    UnlockMutex(is->pictq_mutex);
}

static int queue_picture(VideoState *is, AVFrame *src_frame, double pts1, int64_t pos)
{
    VideoPicture *vp;
    double frame_delay, pts = pts1;
    int src_pix_fmt;

    /* compute the exact PTS for the picture if it is omitted in the stream
     * pts1 is the dts of the pkt / pts of the frame */
    if (pts != 0) {
        /* update video clock with pts, if present */
        is->video_clock = pts;
    } else {
        pts = is->video_clock;
    }
    /* update video clock for next frame */
    frame_delay = av_q2d(is->video_st->codec->time_base);
    /* for MPEG2, the frame can be repeated, so we update the
       clock accordingly */
    frame_delay += src_frame->repeat_pict * (frame_delay * 0.5);
    is->video_clock += frame_delay;

#if defined(DEBUG_SYNC) && 0
    printf("frame_type=%c clock=%0.3f pts=%0.3f\n",
           av_get_picture_type_char(src_frame->pict_type), pts, pts1);
#endif

    /* wait until we have space to put a new picture */
    LockMutex(is->pictq_mutex);

    if(is->pictq_size>=VIDEO_PICTURE_QUEUE_SIZE && !is->refresh)
        is->skip_frames= FFMAX(1.0 - FRAME_SKIP_FACTOR, is->skip_frames * (1.0-FRAME_SKIP_FACTOR));

    while (is->pictq_size >= VIDEO_PICTURE_QUEUE_SIZE &&
           !is->videoq.abort_request) {
        CondWait(is->pictq_cond, is->pictq_mutex);
    }
    UnlockMutex(is->pictq_mutex);

    if (is->videoq.abort_request)
        return -1;

    vp = &is->pictq[is->pictq_windex];

    /* alloc or resize hardware picture buffer */
    if (!vp->bmp ||
#if CONFIG_AVFILTER
        vp->width  != is->out_video_filter->inputs[0]->w ||
        vp->height != is->out_video_filter->inputs[0]->h) {
#else
        vp->width != is->video_st->codec->width ||
        vp->height != is->video_st->codec->height) {
#endif

        vp->allocated = 0;

        WinPostMsg( hwndKMP, WM_ALLOC_EVENT, is, 0 );

        /* wait until the picture is allocated */
        LockMutex(is->pictq_mutex);
        while (!vp->allocated && !is->videoq.abort_request) {
            CondWait(is->pictq_cond, is->pictq_mutex);
        }
        UnlockMutex(is->pictq_mutex);

        if (is->videoq.abort_request)
            return -1;
    }

    /* if the frame is not skipped, then display it */
    if (vp->bmp) {
#if CONFIG_AVFILTER
        if(vp->picref)
            avfilter_unref_buffer(vp->picref);
        vp->picref = src_frame->opaque;
#endif

        // KOMH: treats PIX_FMT_YUVJ420P as PIX_FMT_YUV420P
        src_pix_fmt = vp->pix_fmt != PIX_FMT_YUVJ420P ?
                      vp->pix_fmt :  PIX_FMT_YUV420P;

        // if needed, then convert it
        if( vp->width   != lImgWidth        ||
            vp->height  != lImgHeight       ||
            src_pix_fmt != PIX_FMT_YUV420P  ||
            use_subimg )
        {
            // KOMH: does not allow user to override sws_flags
            //sws_flags  = av_get_int(sws_opts, "sws_flags", NULL);
            is->img_convert_ctx = sws_getCachedContext(is->img_convert_ctx,
                vp->width, vp->height, src_pix_fmt,
                lImgWidth, lImgHeight, PIX_FMT_YUV420P,
                sws_flags, NULL, NULL, NULL);
            if (is->img_convert_ctx == NULL) {
                fprintf(stderr, "Cannot initialize the conversion context\n");
                exit(1);
            }
            sws_scale(is->img_convert_ctx, src_frame->data, src_frame->linesize,
                      0, vp->height, vp->bmp->data, vp->bmp->linesize );
        }
        else
        {
            vp->bmp->data[ 0 ] = src_frame->data[ 0 ];
            vp->bmp->data[ 1 ] = src_frame->data[ 1 ];
            vp->bmp->data[ 2 ] = src_frame->data[ 2 ];

            vp->bmp->linesize[ 0 ] = src_frame->linesize[ 0 ];
            vp->bmp->linesize[ 1 ] = src_frame->linesize[ 1 ];
            vp->bmp->linesize[ 2 ] = src_frame->linesize[ 2 ];
        }

        vp->pts = pts;
        vp->pos = pos;

        /* now we can update the picture count */
        if (++is->pictq_windex == VIDEO_PICTURE_QUEUE_SIZE)
            is->pictq_windex = 0;
        LockMutex(is->pictq_mutex);
        vp->target_clock= compute_target_time(vp->pts, is);

        is->pictq_size++;
        UnlockMutex(is->pictq_mutex);
    }
    return 0;
}

static void audio_thread( void *arg )
{
    VideoState *is = arg;

    if( !video_disable )
    {
        // KOMH: when using uniaud and snap, if audio is played before video is
        // initialized, then system will be locked
        LockMutex( is->audio_mutex );
        while( !is->audio_ok_to_start && !is->abort_request )
            CondWait( is->audio_cond, is->audio_mutex );
        UnlockMutex( is->audio_mutex );
    }

    kaiSetVolume( is->hkai, MCI_SET_AUDIO_ALL, volume_level );

    kaiPlay( is->hkai );

#if 0
    while( kaiStatus( is->hkai ) & KAIS_PLAYING )
        tmrDelay( 10 );
#endif
}

static int get_video_frame(VideoState *is, AVFrame *frame, int64_t *pts, AVPacket *pkt)
{
    int got_picture, i;

    if (packet_queue_get(&is->videoq, pkt, 1) < 0)
        return -1;

    if (pkt->data == flush_pkt.data) {
        avcodec_flush_buffers(is->video_st->codec);

        LockMutex(is->pictq_mutex);
        //Make sure there are no long delay timers (ideally we should just flush the que but thats harder)
        for (i = 0; i < VIDEO_PICTURE_QUEUE_SIZE; i++) {
            is->pictq[i].target_clock= 0;
        }
        while (is->pictq_size && !is->videoq.abort_request) {
            CondWait(is->pictq_cond, is->pictq_mutex);
        }
        is->video_current_pos = -1;
        UnlockMutex(is->pictq_mutex);

        is->frame_last_pts = AV_NOPTS_VALUE;
        is->frame_last_delay = 0;
        is->frame_timer = (double)av_gettime() / 1000000.0;
        is->skip_frames = 1;
        is->skip_frames_index = 0;
        return 0;
    }

    avcodec_decode_video2(is->video_st->codec, frame, &got_picture, pkt);

    if (got_picture) {
        if (decoder_reorder_pts == -1) {
            *pts = frame->best_effort_timestamp;
        } else if (decoder_reorder_pts) {
            *pts = frame->pkt_pts;
        } else {
            *pts = frame->pkt_dts;
        }

        if (*pts == AV_NOPTS_VALUE) {
            *pts = 0;
        }

        is->skip_frames_index += 1;
        if(is->skip_frames_index >= is->skip_frames){
            is->skip_frames_index -= FFMAX(is->skip_frames, 1.0);
            return 1;
        }

    }
    return 0;
}

#if CONFIG_AVFILTER
typedef struct {
    VideoState *is;
    AVFrame *frame;
    int use_dr1;
} FilterPriv;

static int input_get_buffer(AVCodecContext *codec, AVFrame *pic)
{
    AVFilterContext *ctx = codec->opaque;
    AVFilterBufferRef  *ref;
    int perms = AV_PERM_WRITE;
    int i, w, h, stride[4];
    unsigned edge;
    int pixel_size;

    av_assert0(codec->flags & CODEC_FLAG_EMU_EDGE);

    if (codec->codec->capabilities & CODEC_CAP_NEG_LINESIZES)
        perms |= AV_PERM_NEG_LINESIZES;

    if(pic->buffer_hints & FF_BUFFER_HINTS_VALID) {
        if(pic->buffer_hints & FF_BUFFER_HINTS_READABLE) perms |= AV_PERM_READ;
        if(pic->buffer_hints & FF_BUFFER_HINTS_PRESERVE) perms |= AV_PERM_PRESERVE;
        if(pic->buffer_hints & FF_BUFFER_HINTS_REUSABLE) perms |= AV_PERM_REUSE2;
    }
    if(pic->reference) perms |= AV_PERM_READ | AV_PERM_PRESERVE;

    w = codec->width;
    h = codec->height;

    if(av_image_check_size(w, h, 0, codec))
        return -1;

    avcodec_align_dimensions2(codec, &w, &h, stride);
    edge = codec->flags & CODEC_FLAG_EMU_EDGE ? 0 : avcodec_get_edge_width();
    w += edge << 1;
    h += edge << 1;

    if(!(ref = avfilter_get_video_buffer(ctx->outputs[0], perms, w, h)))
        return -1;

    pixel_size = av_pix_fmt_descriptors[ref->format].comp[0].step_minus1+1;
    ref->video->w = codec->width;
    ref->video->h = codec->height;
    for(i = 0; i < 4; i ++) {
        unsigned hshift = (i == 1 || i == 2) ? av_pix_fmt_descriptors[ref->format].log2_chroma_w : 0;
        unsigned vshift = (i == 1 || i == 2) ? av_pix_fmt_descriptors[ref->format].log2_chroma_h : 0;

        if (ref->data[i]) {
            ref->data[i]    += ((edge * pixel_size) >> hshift) + ((edge * ref->linesize[i]) >> vshift);
        }
        pic->data[i]     = ref->data[i];
        pic->linesize[i] = ref->linesize[i];
    }
    pic->opaque = ref;
    pic->age    = INT_MAX;
    pic->type   = FF_BUFFER_TYPE_USER;
    pic->reordered_opaque = codec->reordered_opaque;
    if(codec->pkt) pic->pkt_pts = codec->pkt->pts;
    else           pic->pkt_pts = AV_NOPTS_VALUE;
    return 0;
}

static void input_release_buffer(AVCodecContext *codec, AVFrame *pic)
{
    memset(pic->data, 0, sizeof(pic->data));
    avfilter_unref_buffer(pic->opaque);
}

static int input_reget_buffer(AVCodecContext *codec, AVFrame *pic)
{
    AVFilterBufferRef *ref = pic->opaque;

    if (pic->data[0] == NULL) {
        pic->buffer_hints |= FF_BUFFER_HINTS_READABLE;
        return codec->get_buffer(codec, pic);
    }

    if ((codec->width != ref->video->w) || (codec->height != ref->video->h) ||
        (codec->pix_fmt != ref->format)) {
        av_log(codec, AV_LOG_ERROR, "Picture properties changed.\n");
        return -1;
    }

    pic->reordered_opaque = codec->reordered_opaque;
    if(codec->pkt) pic->pkt_pts = codec->pkt->pts;
    else           pic->pkt_pts = AV_NOPTS_VALUE;
    return 0;
}

static int input_init(AVFilterContext *ctx, const char *args, void *opaque)
{
    FilterPriv *priv = ctx->priv;
    AVCodecContext *codec;
    if(!opaque) return -1;

    priv->is = opaque;
    codec    = priv->is->video_st->codec;
    codec->opaque = ctx;
    if((codec->codec->capabilities & CODEC_CAP_DR1)
    ) {
        av_assert0(codec->flags & CODEC_FLAG_EMU_EDGE);
        priv->use_dr1 = 1;
        codec->get_buffer     = input_get_buffer;
        codec->release_buffer = input_release_buffer;
        codec->reget_buffer   = input_reget_buffer;
        codec->thread_safe_callbacks = 1;
    }

    priv->frame = avcodec_alloc_frame();

    return 0;
}

static void input_uninit(AVFilterContext *ctx)
{
    FilterPriv *priv = ctx->priv;
    av_free(priv->frame);
}

static int input_request_frame(AVFilterLink *link)
{
    FilterPriv *priv = link->src->priv;
    AVFilterBufferRef *picref;
    int64_t pts = 0;
    AVPacket pkt;
    int ret;

    while (!(ret = get_video_frame(priv->is, priv->frame, &pts, &pkt)))
        av_free_packet(&pkt);
    if (ret < 0)
        return -1;

    if(priv->use_dr1 && priv->frame->opaque) {
        picref = avfilter_ref_buffer(priv->frame->opaque, ~0);
    } else {
        picref = avfilter_get_video_buffer(link, AV_PERM_WRITE, link->w, link->h);
        av_image_copy(picref->data, picref->linesize,
                      priv->frame->data, priv->frame->linesize,
                      picref->format, link->w, link->h);
    }
    av_free_packet(&pkt);

    avfilter_copy_frame_props(picref, priv->frame);
    picref->pts = pts;

    avfilter_start_frame(link, picref);
    avfilter_draw_slice(link, 0, link->h, 1);
    avfilter_end_frame(link);

    return 0;
}

static int input_query_formats(AVFilterContext *ctx)
{
    FilterPriv *priv = ctx->priv;
    enum PixelFormat pix_fmts[] = {
        priv->is->video_st->codec->pix_fmt, PIX_FMT_NONE
    };

    avfilter_set_common_pixel_formats(ctx, avfilter_make_format_list(pix_fmts));
    return 0;
}

static int input_config_props(AVFilterLink *link)
{
    FilterPriv *priv  = link->src->priv;
    AVCodecContext *c = priv->is->video_st->codec;

    link->w = c->width;
    link->h = c->height;
    link->sample_aspect_ratio = priv->is->video_st->sample_aspect_ratio;
    link->time_base = priv->is->video_st->time_base;

    return 0;
}

static AVFilter input_filter =
{
    .name      = "ffplay_input",

    .priv_size = sizeof(FilterPriv),

    .init      = input_init,
    .uninit    = input_uninit,

    .query_formats = input_query_formats,

    .inputs    = (AVFilterPad[]) {{ .name = NULL }},
    .outputs   = (AVFilterPad[]) {{ .name = "default",
                                    .type = AVMEDIA_TYPE_VIDEO,
                                    .request_frame = input_request_frame,
                                    .config_props  = input_config_props, },
                                  { .name = NULL }},
};

static int configure_video_filters(AVFilterGraph *graph, VideoState *is, const char *vfilters)
{
    char sws_flags_str[128];
    int ret;
    enum PixelFormat pix_fmts[] = { PIX_FMT_YUV420P, PIX_FMT_NONE };
    AVFilterContext *filt_src = NULL, *filt_out = NULL;
    snprintf(sws_flags_str, sizeof(sws_flags_str), "flags=%d", sws_flags);
    graph->scale_sws_opts = av_strdup(sws_flags_str);

    if ((ret = avfilter_graph_create_filter(&filt_src, &input_filter, "src",
                                            NULL, is, graph)) < 0)
        return ret;
    if ((ret = avfilter_graph_create_filter(&filt_out, avfilter_get_by_name("buffersink"), "out",
                                            NULL, pix_fmts, graph)) < 0)
        return ret;

    if(vfilters) {
        AVFilterInOut *outputs = avfilter_inout_alloc();
        AVFilterInOut *inputs  = avfilter_inout_alloc();

        outputs->name    = av_strdup("in");
        outputs->filter_ctx = filt_src;
        outputs->pad_idx = 0;
        outputs->next    = NULL;

        inputs->name    = av_strdup("out");
        inputs->filter_ctx = filt_out;
        inputs->pad_idx = 0;
        inputs->next    = NULL;

        if ((ret = avfilter_graph_parse(graph, vfilters, &inputs, &outputs, NULL)) < 0)
            return ret;
        av_freep(&vfilters);
    } else {
        if ((ret = avfilter_link(filt_src, 0, filt_out, 0)) < 0)
            return ret;
    }

    if ((ret = avfilter_graph_config(graph, NULL)) < 0)
        return ret;

    is->out_video_filter = filt_out;

    return ret;
}

#endif  /* CONFIG_AVFILTER */

static int video_thread(void *arg)
{
    VideoState *is = arg;
    AVFrame *frame= avcodec_alloc_frame();
    int64_t pts_int = AV_NOPTS_VALUE, pos = -1;
    double pts;
    int ret;

#if CONFIG_AVFILTER
    AVFilterGraph *graph = avfilter_graph_alloc();
    AVFilterContext *filt_out = NULL;

    if ((ret = configure_video_filters(graph, is, vfilters)) < 0)
        goto the_end;
    filt_out = is->out_video_filter;
#endif

    for(;;) {
#if !CONFIG_AVFILTER
        AVPacket pkt;
#else
        AVFilterBufferRef *picref;
        AVRational tb = filt_out->inputs[0]->time_base;
#endif
        while (is->paused && !is->videoq.abort_request)
            tmrDelay(10);
#if CONFIG_AVFILTER
        ret = av_vsink_buffer_get_video_buffer_ref(filt_out, &picref, 0);
        if (picref) {
            avfilter_fill_frame_from_video_buffer_ref(frame, picref);
            pts_int = picref->pts;
            pos     = picref->pos;
            frame->opaque = picref;
        }

        if (av_cmp_q(tb, is->video_st->time_base)) {
            av_unused int64_t pts1 = pts_int;
            pts_int = av_rescale_q(pts_int, tb, is->video_st->time_base);
            av_dlog(NULL, "video_thread(): "
                   "tb:%d/%d pts:%"PRId64" -> tb:%d/%d pts:%"PRId64"\n",
                   tb.num, tb.den, pts1,
                   is->video_st->time_base.num, is->video_st->time_base.den, pts_int);
        }
#else
        ret = get_video_frame(is, frame, &pts_int, &pkt);
        pos = pkt.pos;
        av_free_packet(&pkt);
#endif

        if (ret < 0) goto the_end;

        if (!picref)
            continue;

        pts = pts_int*av_q2d(is->video_st->time_base);

        ret = queue_picture(is, frame, pts, pos);

        if (ret < 0)
            goto the_end;

        if (step)
            if (cur_stream)
                stream_toggle_pause(cur_stream);
    }
 the_end:
#if CONFIG_AVFILTER
    avfilter_graph_free(&graph);
#endif
    av_free(frame);
    imgDone();
    return;
}

static void subtitle_thread(void *arg)
{
    VideoState *is = arg;
    SubPicture *sp;
    AVPacket pkt1, *pkt = &pkt1;
    int got_subtitle;
    double pts;
    int i, j;
    int r, g, b, y, u, v, a;

    for(;;) {
        while (is->paused && !is->subtitleq.abort_request) {
            tmrDelay(10);
        }
        if (packet_queue_get(&is->subtitleq, pkt, 1) < 0)
            break;

        if(pkt->data == flush_pkt.data){
            avcodec_flush_buffers(is->subtitle_st->codec);
            continue;
        }
        LockMutex(is->subpq_mutex);
        while (is->subpq_size >= SUBPICTURE_QUEUE_SIZE &&
               !is->subtitleq.abort_request) {
            CondWait(is->subpq_cond, is->subpq_mutex);
        }
        UnlockMutex(is->subpq_mutex);

        if (is->subtitleq.abort_request)
            return 0;

        sp = &is->subpq[is->subpq_windex];

       /* NOTE: ipts is the PTS of the _first_ picture beginning in
           this packet, if any */
        pts = 0;
        if (pkt->pts != AV_NOPTS_VALUE)
            pts = av_q2d(is->subtitle_st->time_base)*pkt->pts;

        avcodec_decode_subtitle2(is->subtitle_st->codec, &sp->sub,
                                 &got_subtitle, pkt);

        if (got_subtitle && sp->sub.format == 0) {
            sp->pts = pts;

            for (i = 0; i < sp->sub.num_rects; i++)
            {
                for (j = 0; j < sp->sub.rects[i]->nb_colors; j++)
                {
                    RGBA_IN(r, g, b, a, (uint32_t*)sp->sub.rects[i]->pict.data[1] + j);
                    y = RGB_TO_Y_CCIR(r, g, b);
                    u = RGB_TO_U_CCIR(r, g, b, 0);
                    v = RGB_TO_V_CCIR(r, g, b, 0);
                    YUVA_OUT((uint32_t*)sp->sub.rects[i]->pict.data[1] + j, y, u, v, a);
                }
            }

            /* now we can update the picture count */
            if (++is->subpq_windex == SUBPICTURE_QUEUE_SIZE)
                is->subpq_windex = 0;
            LockMutex(is->subpq_mutex);
            is->subpq_size++;
            UnlockMutex(is->subpq_mutex);
        }
        av_free_packet(pkt);
    }
    return 0;
}

/* copy samples for viewing in editor window */
static void update_sample_display(VideoState *is, short *samples, int samples_size)
{
    int size, len;

    size = samples_size / sizeof(short);
    while (size > 0) {
        len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
        if (len > size)
            len = size;
        fast_memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        samples += len;
        is->sample_array_index += len;
        if (is->sample_array_index >= SAMPLE_ARRAY_SIZE)
            is->sample_array_index = 0;
        size -= len;
    }
}

/* return the new audio buffer size (samples can be added or deleted
   to get better sync if video or external master clock) */
static int synchronize_audio(VideoState *is, short *samples,
                             int samples_size1, double pts)
{
    int n, samples_size;
    double ref_clock;

    n = 2 * is->audio_st->codec->channels;
    samples_size = samples_size1;

    /* if not master, then we try to remove or add samples to correct the clock */
    if (((is->av_sync_type == AV_SYNC_VIDEO_MASTER && is->video_st) ||
         is->av_sync_type == AV_SYNC_EXTERNAL_CLOCK)) {
        double diff, avg_diff;
        int wanted_size, min_size, max_size, nb_samples;

        ref_clock = get_master_clock(is);
        diff = get_audio_clock(is) - ref_clock;

        if (diff < AV_NOSYNC_THRESHOLD) {
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum;
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
                /* not enough measures to have a correct estimate */
                is->audio_diff_avg_count++;
            } else {
                /* estimate the A-V difference */
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);

                if (fabs(avg_diff) >= is->audio_diff_threshold) {
                    wanted_size = samples_size + ((int)(diff * is->audio_st->codec->sample_rate) * n);
                    nb_samples = samples_size / n;

                    min_size = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX)) / 100) * n;
                    max_size = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX)) / 100) * n;
                    if (wanted_size < min_size)
                        wanted_size = min_size;
                    else if (wanted_size > max_size)
                        wanted_size = max_size;

                    /* add or remove samples to correction the synchro */
                    if (wanted_size < samples_size) {
                        /* remove samples */
                        samples_size = wanted_size;
                    } else if (wanted_size > samples_size) {
                        uint8_t *samples_end, *q;
                        int nb;

                        /* add samples */
                        nb = (samples_size - wanted_size);
                        samples_end = (uint8_t *)samples + samples_size - n;
                        q = samples_end + n;
                        while (nb > 0) {
                            fast_memcpy(q, samples_end, n);
                            q += n;
                            nb -= n;
                        }
                        samples_size = wanted_size;
                    }
                }
#if 0
                printf("diff=%f adiff=%f sample_diff=%d apts=%0.3f vpts=%0.3f %f\n",
                       diff, avg_diff, samples_size - samples_size1,
                       is->audio_clock, is->video_clock, is->audio_diff_threshold);
#endif
            }
        } else {
            /* too big difference : may be initial PTS errors, so
               reset A-V filter */
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum = 0;
        }
    }

    return samples_size;
}

/* decode one audio frame and returns its uncompressed size */
static int audio_decode_frame(VideoState *is, double *pts_ptr)
{
    AVPacket *pkt_temp = &is->audio_pkt_temp;
    AVPacket *pkt = &is->audio_pkt;
    AVCodecContext *dec= is->audio_st->codec;
    int n, len1, data_size;
    double pts;

    for(;;) {
        /* NOTE: the audio packet can contain several frames */
        while (pkt_temp->size > 0) {
            data_size = sizeof(is->audio_buf1);
            len1 = avcodec_decode_audio3(dec,
                                        (int16_t *)is->audio_buf1, &data_size,
                                        pkt_temp);
            if (len1 < 0) {
                /* if error, we skip the frame */
                pkt_temp->size = 0;
                break;
            }

            pkt_temp->data += len1;
            pkt_temp->size -= len1;
            if (data_size <= 0)
                continue;

            if (dec->sample_fmt != is->audio_src_fmt) {
                if (is->reformat_ctx)
                    av_audio_convert_free(is->reformat_ctx);
                is->reformat_ctx= av_audio_convert_alloc(AV_SAMPLE_FMT_S16, 1,
                                                         dec->sample_fmt, 1, NULL, 0);
                if (!is->reformat_ctx) {
                    fprintf(stderr, "Cannot convert %s sample format to %s sample format\n",
                        av_get_sample_fmt_name(dec->sample_fmt),
                        av_get_sample_fmt_name(AV_SAMPLE_FMT_S16));
                        break;
                }
                is->audio_src_fmt= dec->sample_fmt;
            }

            if (is->reformat_ctx) {
                const void *ibuf[6]= {is->audio_buf1};
                void *obuf[6]= {is->audio_buf2};
                int istride[6]= {av_get_bytes_per_sample(dec->sample_fmt)};
                int ostride[6]= {2};
                int len= data_size/istride[0];
                if (av_audio_convert(is->reformat_ctx, obuf, ostride, ibuf, istride, len)<0) {
                    printf("av_audio_convert() failed\n");
                    break;
                }
                is->audio_buf= is->audio_buf2;
                /* FIXME: existing code assume that data_size equals framesize*channels*2
                          remove this legacy cruft */
                data_size= len*2;
            }else{
                is->audio_buf= is->audio_buf1;
            }

            /* if no pts, then compute it */
            pts = is->audio_clock;
            *pts_ptr = pts;
            n = 2 * dec->channels;
            is->audio_clock += (double)data_size /
                (double)(n * dec->sample_rate);
#ifdef DEBUG
            {
                static double last_clock;
                printf("audio: delay=%0.3f clock=%0.3f pts=%0.3f\n",
                       is->audio_clock - last_clock,
                       is->audio_clock, pts);
                last_clock = is->audio_clock;
            }
#endif
            return data_size;
        }

        /* free the current packet */
        if (pkt->data)
            av_free_packet(pkt);

        if (is->paused || is->audioq.abort_request) {
            return -1;
        }

        /* read next packet */
        if (packet_queue_get(&is->audioq, pkt, 1) < 0)
            return -1;
        if(pkt->data == flush_pkt.data){
            avcodec_flush_buffers(dec);
            continue;
        }

        pkt_temp->data = pkt->data;
        pkt_temp->size = pkt->size;

        /* if update the audio clock with the pts */
        if (pkt->pts != AV_NOPTS_VALUE) {
            is->audio_clock = av_q2d(is->audio_st->time_base)*pkt->pts;
        }
    }
}

/* prepare a new audio buffer */
static ULONG APIENTRY kai_audio_callback( PVOID pCBData, PVOID pBuffer, ULONG len )
{
    static DECLARE_ALIGNED(16,uint8_t,res_audio_buf[(AVCODEC_MAX_AUDIO_FRAME_SIZE * 3) / 2]);

    VideoState *is = pCBData;
    uint8_t *stream = pBuffer;
    int audio_size, len1;
    double pts;

    audio_callback_time = av_gettime();

    while (len > 0) {
        if (is->audio_buf_index >= is->audio_buf_size) {
           audio_size = audio_decode_frame(is, &pts);
           if (audio_size < 0) {
                /* if error, just output silence */
               is->audio_buf = is->audio_buf1;
               is->audio_buf_size = 1024;
               memset(is->audio_buf, 0, is->audio_buf_size);
           } else {
               if( is->rsc )
               {
                   int n;
                   int nb_out_samples;

                   n = 2 * is->audio_st->codec->channels;
                   nb_out_samples = audio_resample( is->rsc,
                                                    ( short * )res_audio_buf,
                                                    ( short * )is->audio_buf,
                                                    audio_size / n );
                   audio_size = nb_out_samples * n;
                   fast_memcpy( is->audio_buf, res_audio_buf, audio_size );
               }

               if (is->show_mode != SHOW_MODE_VIDEO)
                   update_sample_display(is, (int16_t *)is->audio_buf, audio_size);
               audio_size = synchronize_audio(is, (int16_t *)is->audio_buf, audio_size,
                                              pts);
               is->audio_buf_size = audio_size;
           }
           is->audio_buf_index = 0;
        }
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len)
            len1 = len;
        fast_memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
        len -= len1;
        stream += len1;
        is->audio_buf_index += len1;
    }

    return ( stream - ( uint8_t * )pBuffer );
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    AVCodec *codec;
    AVDictionary *opts;
    AVDictionaryEntry *t = NULL;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return -1;
    avctx = ic->streams[stream_index]->codec;

    opts = filter_codec_opts(codec_opts, avctx->codec_id, 0);

    /* prepare audio output */
    if (avctx->codec_type == AVMEDIA_TYPE_AUDIO) {
        if (avctx->channels > 0) {
            avctx->request_channels = FFMIN(2, avctx->channels);
        } else {
            avctx->request_channels = 2;
        }

        if(( resample == RES_48KONLY ) && ( avctx->sample_rate == 48000 ))
        {
            printf("Resample 48KHz to 44.1KHz\n");
            is->rsc = av_audio_resample_init( avctx->channels, avctx->channels,
                                              44100, avctx->sample_rate,
                                              is->audio_src_fmt, is->audio_src_fmt,
                                              16, 10, 0, 0.8 );
            avctx->sample_rate = 44100;
        }
    }

    codec = avcodec_find_decoder(avctx->codec_id);
    if (!codec)
        return -1;

    avctx->workaround_bugs = workaround_bugs;
    avctx->lowres = lowres;
    if(lowres) avctx->flags |= CODEC_FLAG_EMU_EDGE;
    avctx->idct_algo= idct;
    if(fast) avctx->flags2 |= CODEC_FLAG2_FAST;
    avctx->skip_frame= skip_frame;
    avctx->skip_idct= skip_idct;
    avctx->skip_loop_filter= skip_loop_filter;
    avctx->error_recognition= error_recognition;
    avctx->error_concealment= error_concealment;
    avctx->thread_count= thread_count;

    if(codec->capabilities & CODEC_CAP_DR1)
        avctx->flags |= CODEC_FLAG_EMU_EDGE;

    if (!codec ||
        avcodec_open2(avctx, codec, &opts) < 0)
        return -1;
    if ((t = av_dict_get(opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        return AVERROR_OPTION_NOT_FOUND;
    }

    /* prepare audio output */
    if (avctx->codec_type == AVMEDIA_TYPE_AUDIO) {
        if(avctx->sample_rate <= 0 || avctx->channels <= 0){
            fprintf(stderr, "Invalid sample rate or channel count\n");
            return -1;
        }
        KAISPEC ksWanted, ksObtained;

        if( kaiInit( audio_driver ))
        {
            fprintf( stderr, "Cannot initialize audio\n");
            return -1;
        }

        ksWanted.usDeviceIndex      = 0;
        ksWanted.ulType             = KAIT_PLAY;
        ksWanted.ulBitsPerSample    = BPS_16;
        ksWanted.ulSamplingRate     = avctx->sample_rate;
        ksWanted.ulDataFormat       = MCI_WAVE_FORMAT_PCM;
        ksWanted.ulChannels         = avctx->channels;
        ksWanted.ulNumBuffers       = 2;
        ksWanted.ulBufferSize       = audio_buffer_size ? audio_buffer_size : ( 2 * avctx->channels * 2048 );
        ksWanted.fShareable         = !no_audio_share;
        ksWanted.pfnCallBack        = kai_audio_callback;
        ksWanted.pCallBackData      = is;

        if( kaiOpen( &ksWanted, &ksObtained, &is->hkai ))
        {
            fprintf( stderr, "Cannot open audio\n");
            return -1;
        }

        if( no_audio_share )
            printf("Opened audio as exclusive mode\n");

        is->audio_hw_buf_size = ksObtained.ulBufferSize;
        is->audio_src_fmt= AV_SAMPLE_FMT_S16;
    }

    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;
    switch(avctx->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        is->audio_stream = stream_index;
        is->audio_st = ic->streams[stream_index];
        is->audio_buf_size = 0;
        is->audio_buf_index = 0;

        /* init averaging filter */
        is->audio_diff_avg_coef = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
        is->audio_diff_avg_count = 0;
        /* since we do not have a precise anough audio fifo fullness,
           we correct audio sync only if larger than this threshold */
        is->audio_diff_threshold = 2.0 * 1024 / avctx->sample_rate;

        memset(&is->audio_pkt, 0, sizeof(is->audio_pkt));
        packet_queue_init(&is->audioq);
        is->audio_tid = CreateThread( audio_thread, is );
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_stream = stream_index;
        is->video_st = ic->streams[stream_index];

        packet_queue_init(&is->videoq);
        is->video_tid = CreateThread(video_thread, is);
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_stream = stream_index;
        is->subtitle_st = ic->streams[stream_index];
        packet_queue_init(&is->subtitleq);

        is->subtitle_tid = CreateThread(subtitle_thread, is);
        break;
    default:
        break;
    }
    return 0;
}

static void stream_component_close(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;
    avctx = ic->streams[stream_index]->codec;

    switch(avctx->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        packet_queue_abort(&is->audioq);

        kaiStop( is->hkai );
        WaitThread( is->audio_tid );
        kaiClose( is->hkai );
        kaiDone();

        packet_queue_end(&is->audioq);
        if (is->reformat_ctx)
            av_audio_convert_free(is->reformat_ctx);
        is->reformat_ctx = NULL;
        if( is->rsc )
            audio_resample_close( is->rsc );
        break;
    case AVMEDIA_TYPE_VIDEO:
        packet_queue_abort(&is->videoq);

        /* note: we also signal this mutex to make sure we deblock the
           video thread in all cases */
        LockMutex(is->pictq_mutex);
        CondSignal(is->pictq_cond);
        UnlockMutex(is->pictq_mutex);

        WaitThread( is->video_tid );

        packet_queue_end(&is->videoq);
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        packet_queue_abort(&is->subtitleq);

        /* note: we also signal this mutex to make sure we deblock the
           video thread in all cases */
        LockMutex(is->subpq_mutex);
        is->subtitle_stream_changed = 1;

        CondSignal(is->subpq_cond);
        UnlockMutex(is->subpq_mutex);

        WaitThread( is->subtitle_tid );

        packet_queue_end(&is->subtitleq);
        break;
    default:
        break;
    }

    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    avcodec_close(avctx);
    switch(avctx->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        is->audio_st = NULL;
        is->audio_stream = -1;
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_st = NULL;
        is->video_stream = -1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_st = NULL;
        is->subtitle_stream = -1;
        break;
    default:
        break;
    }
}

/* since we have only one decoding thread, we can use a global
   variable instead of a thread local variable */
static VideoState *global_video_state;

static int decode_interrupt_cb(void)
{
    return (global_video_state && global_video_state->abort_request);
}

/* this thread gets the stream from the disk or the network */
static int read_thread(void *arg)
{
    VideoState *is = arg;
    AVFormatContext *ic = NULL;
    int err, i, ret;
    int st_index[AVMEDIA_TYPE_NB];
    AVPacket pkt1, *pkt = &pkt1;
    int eof=0;
    int pkt_in_play_range = 0;
    AVDictionaryEntry *t;
    AVDictionary **opts;
    int orig_nb_streams;
    int empty_count = 0;

    memset(st_index, -1, sizeof(st_index));
    is->video_stream = -1;
    is->audio_stream = -1;
    is->subtitle_stream = -1;

    global_video_state = is;
    avio_set_interrupt_cb(decode_interrupt_cb);

    err = avformat_open_input(&ic, is->filename, is->iformat, &format_opts);
    if (err < 0) {
        print_error(is->filename, err);
        ret = -1;
        goto fail;
    }
    if ((t = av_dict_get(format_opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        ret = AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }
    is->ic = ic;

    if(genpts)
        ic->flags |= AVFMT_FLAG_GENPTS;

    opts = setup_find_stream_info_opts(ic, codec_opts);
    orig_nb_streams = ic->nb_streams;

    err = avformat_find_stream_info(ic, opts);
    if (err < 0) {
        fprintf(stderr, "%s: could not find codec parameters\n", is->filename);
        ret = -1;
        goto fail;
    }
    for (i = 0; i < orig_nb_streams; i++)
        av_dict_free(&opts[i]);
    av_freep(&opts);

    if(ic->pb)
        ic->pb->eof_reached= 0; //FIXME hack, ffplay maybe should not use url_feof() to test for the end

    if(seek_by_bytes<0)
        seek_by_bytes= !!(ic->iformat->flags & AVFMT_TS_DISCONT);

    /* if seeking requested, we execute it */
    if (start_time != AV_NOPTS_VALUE) {
        int64_t timestamp;

        timestamp = start_time;
        /* add the stream start time */
        if (ic->start_time != AV_NOPTS_VALUE)
            timestamp += ic->start_time;
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            fprintf(stderr, "%s: could not seek to position %0.3f\n",
                    is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }

    for (i = 0; i < ic->nb_streams; i++)
        ic->streams[i]->discard = AVDISCARD_ALL;
    if (!video_disable)
        st_index[AVMEDIA_TYPE_VIDEO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
                                wanted_stream[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);
    if (!audio_disable)
        st_index[AVMEDIA_TYPE_AUDIO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
                                wanted_stream[AVMEDIA_TYPE_AUDIO],
                                st_index[AVMEDIA_TYPE_VIDEO],
                                NULL, 0);
    if (!video_disable)
        st_index[AVMEDIA_TYPE_SUBTITLE] =
            av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
                                wanted_stream[AVMEDIA_TYPE_SUBTITLE],
                                (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ?
                                 st_index[AVMEDIA_TYPE_AUDIO] :
                                 st_index[AVMEDIA_TYPE_VIDEO]),
                                NULL, 0);
    if (show_status) {
        av_dump_format(ic, 0, is->filename, 0);
    }

    is->show_mode = show_mode;

    /* open the streams */
    if (st_index[AVMEDIA_TYPE_AUDIO] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }

    ret=-1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        // KOMH: init subtitle here to avoid latency
        is->use_sub = !subInit( hab, is->filename, sub_fontnamesize,
                                1 / av_q2d( ic->streams[ st_index[AVMEDIA_TYPE_VIDEO ]]->time_base ),
                                use_subimg );

        ret= stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }
    is->refresh_tid = CreateThread(refresh_thread, is);
    if (is->show_mode == SHOW_MODE_NONE)
        is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;
    if (ret < 0) {
        // KOMH: resume audio thread if no video
        LockMutex( is->audio_mutex );
        is->audio_ok_to_start = 1;
        CondSignal( is->audio_cond );
        UnlockMutex( is->audio_mutex );
    }

    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }

    if (is->video_stream < 0 && is->audio_stream < 0) {
        fprintf(stderr, "%s: could not open codecs\n", is->filename);
        ret = -1;
        goto fail;
    }

    for(;;) {
        if (is->abort_request)
            break;

        // KOMH: check at least 10 times if the queues are empty, before exit
        if( !is->audioq.size && !is->videoq.size && !is->subtitleq.size )
        {
            if( ++empty_count > 10 )
                break;
        }
        else
            empty_count = 0;

        if (is->paused != is->last_paused) {
            is->last_paused = is->paused;
            if (is->paused)
                is->read_pause_return= av_read_pause(ic);
            else
                av_read_play(ic);
        }
#if CONFIG_RTSP_DEMUXER
        if (is->paused && !strcmp(ic->iformat->name, "rtsp")) {
            /* wait 10 ms to avoid trying to get another packet */
            /* XXX: horrible */
            tmrDelay(10);
            continue;
        }
#endif
        if (is->seek_req) {
            int64_t seek_target= is->seek_pos;
            int64_t seek_min= is->seek_rel > 0 ? seek_target - is->seek_rel + 2: INT64_MIN;
            int64_t seek_max= is->seek_rel < 0 ? seek_target - is->seek_rel - 2: INT64_MAX;
//FIXME the +-2 is due to rounding being not done in the correct direction in generation
//      of the seek_pos/seek_rel variables

            ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max, is->seek_flags);
            if (ret < 0) {
                fprintf(stderr, "%s: error while seeking\n", is->ic->filename);
            }else{
                if (is->audio_stream >= 0) {
                    packet_queue_flush(&is->audioq);
                    packet_queue_put(&is->audioq, &flush_pkt);
                }
                if (is->subtitle_stream >= 0) {
                    packet_queue_flush(&is->subtitleq);
                    packet_queue_put(&is->subtitleq, &flush_pkt);
                }
                if (is->video_stream >= 0) {
                    packet_queue_flush(&is->videoq);
                    packet_queue_put(&is->videoq, &flush_pkt);
                }
            }
            is->seek_req = 0;
            eof= 0;
        }

        /* if the queue are full, no need to read more */
        if (   is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE
            || (   (is->audioq   .size  > MIN_AUDIOQ_SIZE || is->audio_stream<0)
                && (is->videoq   .nb_packets > MIN_FRAMES || is->video_stream<0)
                && (is->subtitleq.nb_packets > MIN_FRAMES || is->subtitle_stream<0))) {
            /* wait 10 ms */
            tmrDelay(10);
            continue;
        }
        if(eof) {
            if(is->video_stream >= 0){
                av_init_packet(pkt);
                pkt->data=NULL;
                pkt->size=0;
                pkt->stream_index= is->video_stream;
                packet_queue_put(&is->videoq, pkt);
            }
            tmrDelay(10);
            if(is->audioq.size + is->videoq.size + is->subtitleq.size ==0){
                if(loop!=1 && (!loop || --loop)){
                    stream_seek(cur_stream, start_time != AV_NOPTS_VALUE ? start_time : 0, 0, 0);
                }else if(autoexit){
                    ret=AVERROR_EOF;
                    goto fail;
                }
            }
            eof=0;
            continue;
        }
        ret = av_read_frame(ic, pkt);
        if (ret < 0) {
            if (ret == AVERROR_EOF || url_feof(ic->pb))
                eof=1;
            if (ic->pb && ic->pb->error)
                break;
            tmrDelay(100); /* wait for user event */
            continue;
        }
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        pkt_in_play_range = duration == AV_NOPTS_VALUE ||
                (pkt->pts - ic->streams[pkt->stream_index]->start_time) *
                av_q2d(ic->streams[pkt->stream_index]->time_base) -
                (double)(start_time != AV_NOPTS_VALUE ? start_time : 0)/1000000
                <= ((double)duration/1000000);
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            packet_queue_put(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream && pkt_in_play_range) {
            packet_queue_put(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            packet_queue_put(&is->subtitleq, pkt);
        } else {
            av_free_packet(pkt);
        }
    }

    ret = 0;
 fail:
    /* disable interrupting */
    global_video_state = NULL;

    /* close each stream */
    if (is->audio_stream >= 0)
        stream_component_close(is, is->audio_stream);
    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);
    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);
    if (is->ic) {
        av_close_input_file(is->ic);
        is->ic = NULL; /* safety */
    }
    avio_set_interrupt_cb(NULL);

    if( is->use_sub )
        subDone();

    //if (ret != 0)
        WinPostMsg( hwndKMP, WM_QUIT, 0, 0 );
    return;
}

static VideoState *stream_open(const char *filename, AVInputFormat *iformat)
{
    VideoState *is;

    is = av_mallocz(sizeof(VideoState));
    if (!is)
        return NULL;
    // KOMH: set refresh_tid to -1, otherwise hang on exiting due to a failure
    // of opening a media, because 0 means a current thread and it's not
    // possible to wait for a current thread to finish itself
    is->refresh_tid = -1;
    av_strlcpy(is->filename, filename, sizeof(is->filename));
    is->iformat = iformat;
    is->ytop = 0;
    is->xleft = 0;

    /* start video display */
    is->pictq_mutex = CreateMutex();
    is->pictq_cond = CreateCond();

    is->subpq_mutex = CreateMutex();
    is->subpq_cond = CreateCond();

    is->audio_mutex = CreateMutex();
    is->audio_cond = CreateCond();

    tmrInit();

    is->av_sync_type = av_sync_type;
    is->read_tid = CreateThread(read_thread, is);
    if (!is->read_tid) {
        av_free(is);
        return NULL;
    }
    return is;
}

static void stream_cycle_channel(VideoState *is, int codec_type)
{
    AVFormatContext *ic = is->ic;
    int start_index, stream_index;
    AVStream *st;

    if (codec_type == AVMEDIA_TYPE_VIDEO)
        start_index = is->video_stream;
    else if (codec_type == AVMEDIA_TYPE_AUDIO)
        start_index = is->audio_stream;
    else
        start_index = is->subtitle_stream;
    if (start_index < (codec_type == AVMEDIA_TYPE_SUBTITLE ? -1 : 0))
        return;
    stream_index = start_index;
    for(;;) {
        if (++stream_index >= is->ic->nb_streams)
        {
            if (codec_type == AVMEDIA_TYPE_SUBTITLE)
            {
                stream_index = -1;
                goto the_end;
            } else
                stream_index = 0;
        }
        if (stream_index == start_index)
            return;
        st = ic->streams[stream_index];
        if (st->codec->codec_type == codec_type) {
            /* check that parameters are OK */
            switch(codec_type) {
            case AVMEDIA_TYPE_AUDIO:
                if (st->codec->sample_rate != 0 &&
                    st->codec->channels != 0)
                    goto the_end;
                break;
            case AVMEDIA_TYPE_VIDEO:
            case AVMEDIA_TYPE_SUBTITLE:
                goto the_end;
            default:
                break;
            }
        }
    }
 the_end:
    stream_component_close(is, start_index);
    stream_component_open(is, stream_index);
}


static void toggle_full_screen(void)
{
    is_full_screen = !is_full_screen;
    video_open(cur_stream);
}

static void toggle_pause(void)
{
    if (cur_stream)
        stream_toggle_pause(cur_stream);
    step = 0;
}

static void step_to_next_frame(void)
{
    if (cur_stream) {
        /* if the stream is paused unpause it, then step */
        if (cur_stream->paused)
            stream_toggle_pause(cur_stream);
    }
    step = 1;
}

static void toggle_audio_display(void)
{
    if (cur_stream) {
        int bgcolor = MapRGB(0x00, 0x00, 0x00);
        HPS hps = WinGetPS( hwndKMP );
        GpiCreateLogColorTable( hps, 0, LCOLF_RGB, 0, 0, NULL );
        cur_stream->show_mode = (cur_stream->show_mode + 1) % SHOW_MODE_NB;
        fill_rectangle(hps,
                       cur_stream->xleft, cur_stream->ytop, cur_stream->width, cur_stream->height,
                       bgcolor);
        WinReleasePS( hps );
        if( !cur_stream->show_mode )
            imgClearRect( NULL );
    }
}

static MRESULT EXPENTRY NewFrameWndProc( HWND hwnd, ULONG msg, MPARAM mp1, MPARAM mp2 )
{
    switch( msg )
    {
        case WM_QUERYTRACKINFO :
        {
            //USHORT      ustflags = SHORT1FROMMP( mp1 );
            PTRACKINFO  pti = ( PTRACKINFO )mp2;
            RECTL       rcl;

            if( !cur_stream->video_st || is_full_screen )
                break;

            pfnwpOldFrame( hwnd, msg, mp1, mp2 );

            //if( ustflags & ( TF_LEFT | TF_RIGHT | TF_TOP | TF_BOTTOM | TF_SETPOINTERPOS ))
            {
                pti->rclBoundary.xLeft = 0;
                pti->rclBoundary.yBottom = 0;
                pti->rclBoundary.xRight = fs_screen_width;
                pti->rclBoundary.yTop = fs_screen_height;

                rcl.xLeft = 0;
                rcl.yBottom = 0;
                rcl.xRight = lImgWidth + 1;
                rcl.yTop = lImgHeight + 1;

                WinCalcFrameRect( hwnd, &rcl, FALSE );

                pti->ptlMinTrackSize.x = rcl.xRight - rcl.xLeft;
                pti->ptlMinTrackSize.y = rcl.yTop - rcl.yBottom;

                pti->ptlMaxTrackSize.x = fs_screen_width;
                pti->ptlMaxTrackSize.y = fs_screen_height;
            }

            return MRFROMLONG( TRUE );
        }

        case WM_ADJUSTWINDOWPOS :
        {
            PSWP    pswp = ( PSWP )mp1;
            RECTL   rcl;

            if( !cur_stream->video_st || is_full_screen )
                break;

            if( pswp->fl & SWP_SIZE )
            {
                rcl.xLeft = pswp->x;
                rcl.yBottom = pswp->y;
                rcl.xRight = rcl.xLeft + pswp->cx;
                rcl.yTop = rcl.yBottom + pswp->cy;

                WinCalcFrameRect( hwnd, &rcl, TRUE );

                if( rcl.xRight - rcl.xLeft <= lImgWidth )
                    rcl.xRight = rcl.xLeft + ( lImgWidth + 1 );

                if( rcl.yTop - rcl.yBottom <= lImgHeight )
                    rcl.yTop = rcl.yBottom + ( lImgHeight + 1 );

                WinCalcFrameRect( hwnd, &rcl, FALSE );

                if( rcl.xRight - rcl.xLeft > fs_screen_width )
                {
                    rcl.xLeft = 0;
                    rcl.xRight = fs_screen_width;
                }

                if( rcl.yTop - rcl.yBottom > fs_screen_height )
                {
                    rcl.yBottom = 0;
                    rcl.yTop = fs_screen_height;
                }

                pswp->fl |= SWP_MOVE;
                pswp->x = rcl.xLeft;
                pswp->y = rcl.yBottom;

                pswp->cx = rcl.xRight - rcl.xLeft;
                pswp->cy = rcl.yTop - rcl.yBottom;
            }

            break;
        }
    }

    return pfnwpOldFrame( hwnd, msg, mp1, mp2 );
}

static void inform( const char *format, ... )
{
    CHAR    szInformedMsg[ 256 ];
    va_list arg_ptr;

    va_start( arg_ptr, format );
    vsnprintf( szInformedMsg, sizeof( szInformedMsg ), format, arg_ptr );
    va_end( arg_ptr );

    osdSetText( szInformedMsg );

    WinStartTimer( hab, hwndKMP, TID_INFORM, INTERVAL_INFORM );
}

#define CAS_CTRL( f )   (( f ) & KC_CTRL )
#define CAS_ALT( f )    (( f ) & KC_ALT )
#define CAS_SHIFT( f )  (( f ) & KC_SHIFT )
#define CAS_CO( f )     ( CAS_CTRL( f ) && !CAS_ALT( f ) && !CAS_SHIFT( f ))
#define CAS_AO( f )     ( !CAS_CTRL( f ) && CAS_ALT( f ) && !CAS_SHIFT( f ))
#define CAS_SO( f )     ( !CAS_CTRL( f ) && !CAS_ALT( f ) && CAS_SHIFT( f ))
#define CAS_NONE( f )   ( !CAS_CTRL( f ) && !CAS_ALT( f ) && !CAS_SHIFT( f ))

#define FKC_CHAR( f )   (( f ) & KC_CHAR )
#define FKC_SCAN( f )   (( f ) & KC_SCANCODE )
#define FKC_VIRT( f )   (( f ) & KC_VIRTUALKEY )
#define FKC_KEYUP( f )  (( f ) & KC_KEYUP )

static MRESULT EXPENTRY WndProc( HWND hwnd, ULONG msg, MPARAM mp1, MPARAM mp2 )
{
    if( !cur_stream || !cur_stream->ic )
        return WinDefWindowProc( hwnd, msg, mp1, mp2 );

    switch( msg )
    {
        case WM_CHAR :
        {
            USHORT fsFlags;
            USHORT usCh;
            USHORT usVk;

            fsFlags = SHORT1FROMMP( mp1 );
            usCh = SHORT1FROMMP( mp2 );
            usVk = SHORT2FROMMP( mp2 );

            if( FKC_KEYUP( fsFlags ))
                return MRFROMLONG( TRUE );
            else if( exit_on_keydown )
            {
                WinPostMsg( hwnd, WM_QUIT, 0, 0 );
                break;
            }

            if( FKC_VIRT( fsFlags ))
            {
                switch( usVk )
                {
                    case VK_ESC :
                        if( is_full_screen )
                            toggle_full_screen();
                        break;

                    case VK_NEWLINE :
                    case VK_ENTER :
                        toggle_full_screen();
                        break;

                    case VK_SPACE :
                        toggle_pause();
                        inform("%s", cur_stream->paused ? "Pause" : "Resume" );
                        break;

                    case VK_LEFT :
                    case VK_RIGHT :
                    {
                        static int64_t last_seek_time = 0;

                        int64_t cur_seek_time = av_gettime();
                        double incr, pos;
                        int tns;
                        int ns, hh, mm, ss;

                        // KOMH: too fast seek cause kai to hang, so wait a little
                        if( cur_stream->seek_req ||
                            last_seek_time + seek_interval * 1000 > cur_seek_time )
                            break;

                        last_seek_time = cur_seek_time;

                        if( CAS_CO( fsFlags ))
                            incr = -60.0;
                        else
                            incr = -10.0;

                        if( usVk == VK_RIGHT )
                            incr *= -1;

                        tns = cur_stream->ic->duration / AV_TIME_BASE;
                        pos = get_master_clock( cur_stream );

                        // KOMH: to avoid seek error
                        if( pos + incr > tns - 3 )
                        {
                            incr = ( tns - 3 ) - pos;
                            ns = tns - 3;
                        }
                        else if( pos + incr < 0 )
                        {
                            incr = -pos;
                            ns = 0;
                        }
                        else
                            ns = pos + incr;

                        hh = ns / 3600;
                        mm = ( ns % 3600 ) / 60;
                        ss = ( ns % 60 );

                        inform( "Seek %d seconds %s / %02d:%02d:%02d (%d%%)",
                                CAS_CO( fsFlags ) ? 60 : 10,
                                usVk == VK_RIGHT ? "forward" : "backward",
                                hh, mm, ss,
                                ns * 100 / tns );

                        if (seek_by_bytes) {
                            if (cur_stream->video_stream >= 0 && cur_stream->video_current_pos>=0){
                                pos= cur_stream->video_current_pos;
                            }else if(cur_stream->audio_stream >= 0 && cur_stream->audio_pkt.pos>=0){
                                pos= cur_stream->audio_pkt.pos;
                            }else
                                pos = avio_tell(cur_stream->ic->pb);
                            if (cur_stream->ic->bit_rate)
                                incr *= cur_stream->ic->bit_rate / 8.0;
                            else
                                incr *= 180000.0;
                            pos += incr;
                            stream_seek(cur_stream, pos, incr, 1);
                        } else {
                            //pos = get_master_clock(cur_stream);
                            pos += incr;
                            stream_seek(cur_stream, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
                        }
                        break;
                    }

                    case VK_UP :
                    case VK_DOWN :
                    {
                        int   delta;

                        if( CAS_CO( fsFlags ))
                            delta = 5;
                        else
                            delta = 1;

                        if( usVk == VK_DOWN )
                            delta *= -1;

                        volume_level += delta;
                        if( volume_level > 100 )
                            volume_level = 100;
                        if( volume_level < 0 )
                            volume_level = 0;

                        kaiSetVolume( cur_stream->hkai, MCI_SET_AUDIO_ALL, volume_level );
                        inform("Volume : %d%%", volume_level );
                        break;
                    }

                    case VK_PAGEUP :
                        cmdPlaylist = PLAYLIST_PREV_FILE;
                        WinPostMsg( hwnd, WM_QUIT, 0, 0 );
                        break;

                    case VK_PAGEDOWN :
                        cmdPlaylist = PLAYLIST_NEXT_FILE;
                        WinPostMsg( hwnd, WM_QUIT, 0, 0 );
                        break;
                }
            }
            else if( usCh ) // KOMH: FKC_CHAR( fsFlags ) does not work when ALT pressed
            {
                switch( usCh )
                {
                    case 'q' :
                    case 'Q' :
                        cmdPlaylist = PLAYLIST_QUIT;
                        WinPostMsg( hwnd, WM_QUIT, 0, 0 );
                        break;

                    case 'f' :
                    case 'F' :
                        toggle_full_screen();
                        break;

                    case 'p' :
                    case 'P' :
                        toggle_pause();
                        inform("%s", cur_stream->paused ? "Pause" : "Resume");
                        break;

                    case 's' :
                    case 'S' :
                        step_to_next_frame();
                        inform("Step to next frame");
                        break;

                    case 'a' :
                    case 'A' :
                        stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                        inform("Cycle audio channel");
                        break;

                    case 'v' :
                    case 'V' :
                        stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                        inform("Cycle video channel");
                        break;

                    case 't' :
                    case 'T' :
                        stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                        inform("Cycle subtitle channel");
                        break;

                    case 'w' :
                    case 'W' :
                    {
                        static const char *szModeName[] = {"Normal play",
                                                           "Audio wave",
                                                           "Audio RDFT",};
                        toggle_audio_display();
                        inform("%s mode", szModeName[ cur_stream->show_mode ]);
                        break;
                    }

                    case 'm' :
                    case 'M' :
                    {
                        static BOOL fMute = FALSE;

                        fMute = !fMute;
                        kaiSetSoundState( cur_stream->hkai, MCI_SET_AUDIO_ALL, !fMute );
                        inform("Mute %s", fMute ? "On" : "Off");
                        break;
                    }

                    case '+' :
                    case '=' :
                    case '-' :
                    case '_' :
                        if( cur_stream->use_sub )
                        {
                            FIXED fxPointSize;
                            CHAR  szFontName[ FACESIZE + 1 ];

                            fxPointSize = subQueryFontSize(
                                            (( usCh == '+' ) || ( usCh == '=' )) ?
                                            SQFS_NEXT : SQFS_PREV );

                            fxPointSize = subSetFontSize( fxPointSize );

                            inform("Font : %d.%s",
                                   FIXEDINT( fxPointSize ),
                                   subQueryFontName( szFontName, sizeof( szFontName )));
                        }
                        break;

                    case '0' :
                    case '1' :
                    case '2' :
                    case '3' :
                        if( CAS_AO( fsFlags ))
                        {
                            static PSZ pszAR[] = { "None",
                                                   "Original",
                                                   "Force to 4:3",
                                                   "Force to 16:9", };

                            ULONG ulRatio = KVAR_NONE;

                            switch( usCh )
                            {
                            #if 0
                                case '0' :
                                    ulRatio = KVAR_NONE;
                                    break;
                            #endif

                                case '1' :
                                    ulRatio = KVAR_ORIGINAL;
                                    break;

                                case '2' :
                                    ulRatio = KVAR_FORCE43;
                                    break;

                                case '3' :
                                    ulRatio = KVAR_FORCE169;
                                    break;
                            }

                            imgSetAspectRatio( ulRatio );

                            inform("Aspect ratio : %s", pszAR[ ulRatio ]);
                        }
                        break;

                    case '8' :
                    case '9' :
                    case 'i' :
                    case 'I' :
                    case 'o' :
                    case 'O' :
                    case 'k' :
                    case 'K' :
                    case 'l' :
                    case 'L' :
                    case ',' :
                    case '.' :
                    {
                        static PSZ pszAttr[] = { "Brightness",
                                                 "Contrast",
                                                 "Saturation",
                                                 "Hue", };

                        ULONG ulAttr;
                        ULONG ulValue;
                        int   delta;

                        switch( usCh )
                        {
                            case '8' :
                            case '9' :
                                ulAttr = KVAA_BRIGHTNESS;
                                delta = ( usCh == '8' ) ? -5 : 5;
                                break;

                            case 'i' :
                            case 'I' :
                            case 'o' :
                            case 'O' :
                                ulAttr = KVAA_CONTRAST;
                                delta = (( usCh == 'i' ) || ( usCh == 'I' )) ? -5 : 5;
                                break;

                            case 'k' :
                            case 'K' :
                            case 'l' :
                            case 'L' :
                                ulAttr = KVAA_SATURATION;
                                delta = (( usCh == 'k' ) || ( usCh == 'K' )) ? -5 : 5;
                                break;

                            case ',' :
                            case '.' :
                            default  : // to avoid warning
                                ulAttr = KVAA_HUE;
                                delta = ( usCh == ',' ) ? -5 : 5;
                                break;
                        }

                        if( imgQueryAttr( ulAttr, &ulValue ))
                            break;

                        ulValue += delta;

                        if( !imgSetAttr( ulAttr, &ulValue ))
                        {
                            aulAttrValue[ ulAttr ] = ulValue;

                            inform("%s : %lu", pszAttr[ ulAttr ], ulValue );
                        }

                        break;
                    }

                    case '/' :
                    {
                        int i;

                        imgResetAttr();

                        for( i = 0; i < KVAA_LAST; i++ )
                            aulAttrValue[ i ] = -1;

                        inform("Reset all attributes to default");
                        break;
                    }

                    case '[' :
                    case ']' :
                        if( cur_stream->use_sub )
                        {
                            LONG lDelta = subQueryDelta();

                            lDelta += ( usCh == '[' ) ? -50 : 50;

                            subSetDelta( lDelta );

                            inform("Subtitle sync correction : %+.1fs", lDelta / 100. );
                        }
                        break;

                    case ';' :
                    case '\'' :
                        audio_clock_delta += ( usCh == ';' ) ? -0.1 : 0.1;
                        inform("Audio sync correction : %+.1fs", audio_clock_delta );
                        break;

                    case 'd' :
                    case 'D' :
                    {
                        char *filename;

                        filename = strrchr( input_filename, '\\');
                        if( !filename )
                            filename = strrchr( input_filename, '/');
                        if( !filename )
                            filename = strrchr( input_filename, ':');
                        if( !filename )
                            filename = input_filename;
                        else
                            filename++;

                        inform("[%d/%d] %s [%s]",
                               plQueryIndex( ppl ) + 1,
                               plQueryCount( ppl ),
                               filename,
                               cur_stream->use_sub ? "SUB" : "NO SUB");
                        break;
                    }
                }
            }

            return MRFROMLONG( TRUE );
        }

        case WM_MOUSEMOVE :
        case WM_BUTTON1DOWN :
        case WM_BUTTON2DOWN :
        case WM_BUTTON3DOWN :
        {
            double x = SHORT1FROMMP( mp1 );
            if( hide_mouse && is_full_screen )
            {
                if( !fMouseShow )
                    SHOW_POINTER( TRUE );

                WinStartTimer( hab, hwnd, TID_MOUSE_HIDE, INTERVAL_MOUSE_HIDE );
            }

            if( msg == WM_MOUSEMOVE )
                break; // fall through to default window procedure

            if( exit_on_mousedown )
            {
                WinPostMsg( hwnd, WM_QUIT, 0, 0 );
                break;
            }

            if( WinQueryFocus( HWND_DESKTOP ) != hwnd )
            {
                WinSetFocus( HWND_DESKTOP, hwnd );
                return MRFROMLONG( TRUE );
            }

            if(seek_by_bytes || cur_stream->ic->duration<=0){
                uint64_t size=  avio_size(cur_stream->ic->pb);
                stream_seek(cur_stream, size*x/cur_stream->width, 0, 1);
            } else {
                double frac;
                int64_t ts;
                int ns, hh, mm, ss;
                int tns, thh, tmm, tss;

                tns = cur_stream->ic->duration/1000000LL;
                thh = tns/3600;
                tmm = (tns%3600)/60;
                tss = (tns%60);
                frac = x/cur_stream->width;
                ns = frac*tns;
                hh = ns/3600;
                mm = (ns%3600)/60;
                ss = (ns%60);
                inform("Seek to %2.0f%% (%2d:%02d:%02d) of (%2d:%02d:%02d)", frac*100,
                    hh, mm, ss, thh, tmm, tss);
                ts = frac*cur_stream->ic->duration;
                if (cur_stream->ic->start_time != AV_NOPTS_VALUE)
                    ts += cur_stream->ic->start_time;
                stream_seek(cur_stream, ts, 0, 0);
            }
            return MRFROMLONG( TRUE );
        }

#if 0
        case WM_BUTTON1DBLCLK :
            toggle_full_screen();
            return MRFROMLONG( TRUE );
#endif

        case WM_SIZE :
            cur_stream->width  = SHORT1FROMMP( mp2 );
            cur_stream->height = SHORT2FROMMP( mp2 );

            if ( !is_full_screen ) {
                screen_width  = SHORT1FROMMP( mp2 );
                screen_height = SHORT2FROMMP( mp2 );
            }

            if( cur_stream->use_sub )
                subInvalidate();

            inform("%s mode : Size = %d X %d",
                   is_full_screen ? "Full screen" : "Windowed",
                   cur_stream->width, cur_stream->height );

            return 0;

        case WM_TIMER :
            switch( SHORT1FROMMP( mp1 ))
            {
                case TID_INFORM :
                    osdInvalidate();

                    WinStopTimer( hab, hwnd, TID_INFORM );

                    if( cur_stream->paused )
                        WinInvalidateRect( hwnd, NULL, TRUE );

                    return 0;

                case TID_SECOND :
                {
                    CHAR szPlayTime[ 256 ];

                    int ns, hh, mm, ss;
                    int tns, thh, tmm, tss;

                    if( cur_stream->paused )
                        break;

                    tns = cur_stream->ic->duration / 1000000LL;
                    thh = tns / 3600;
                    tmm = ( tns % 3600 ) / 60;
                    tss = ( tns % 60 );

                    ns = get_master_clock( cur_stream );
                    hh = ns / 3600;
                    mm = ( ns % 3600 ) / 60;
                    ss = ( ns % 60 );

                    snprintf( szPlayTime, sizeof( szPlayTime ),
                              "%02d:%02d:%02d/%02d:%02d:%02d (%d%%) - %s",
                              hh, mm, ss, thh, tmm, tss, ns * 100 / tns,
                              KMP_NAME );

                    WinSetWindowText( hwndTitleBar, szPlayTime );

                    return 0;
                }

                case TID_MOUSE_HIDE :
                    if( fMouseShow )
                        SHOW_POINTER( FALSE );

                    WinStopTimer( hab, hwnd, TID_MOUSE_HIDE );

                    return 0;
            }
            break;  // fall through to default window procedure

        case WM_PAINT :
        {
            HPS     hps;
            RECTL   rcl;

            hps = WinBeginPaint( hwnd, NULLHANDLE, &rcl );
            // No video ?
            if( !cur_stream->video_st )
            {
                // Then just fill with BLACK
                WinFillRect( hps, &rcl, CLR_BLACK );
            }
            WinEndPaint( hps );
        }   // fall through to WM_MOVE

        case WM_MOVE :
            if( cur_stream->paused )
            {
                // to prevent subtitles/OSD from being overwritten when subimg mode
                cur_stream->paint_in_pause = TRUE;
                video_display( cur_stream );
                cur_stream->paint_in_pause = FALSE;
            }

            return 0;

        case WM_CLOSE :
            cmdPlaylist = PLAYLIST_QUIT;
            WinPostMsg( hwnd, WM_QUIT, 0, 0 );

            return 0;

        case WM_ALLOC_EVENT:
            // KOMH: call alloc_picture() first to set lImgWidth and lImgHeight
            alloc_picture(( void * )mp1 );
            video_open(( VideoState * )mp1 );
            break;

        case WM_REFRESH_EVENT:
            video_refresh(( void * )mp1 );
            cur_stream->refresh=0;
            return 0;
    }

    return WinDefWindowProc( hwnd, msg, mp1, mp2 );
}

static int opt_frame_size(const char *opt, const char *arg)
{
    av_log(NULL, AV_LOG_WARNING, "Option -s is deprecated, use -video_size.\n");
    return opt_default("video_size", arg);
}

static int opt_width(const char *opt, const char *arg)
{
    opt_screen_width = parse_number_or_die(opt, arg, OPT_INT64, 1, INT_MAX);
    return 0;
}

static int opt_height(const char *opt, const char *arg)
{
    opt_screen_height = parse_number_or_die(opt, arg, OPT_INT64, 1, INT_MAX);
    return 0;
}

static int opt_format(const char *opt, const char *arg)
{
    file_iformat = av_find_input_format(arg);
    if (!file_iformat) {
        fprintf(stderr, "Unknown input format: %s\n", arg);
        return AVERROR(EINVAL);
    }
    return 0;
}

static int opt_frame_pix_fmt(const char *opt, const char *arg)
{
    av_log(NULL, AV_LOG_WARNING, "Option -pix_fmt is deprecated, use -pixel_format.\n");
    return opt_default("pixel_format", arg);
}

static int opt_sync(const char *opt, const char *arg)
{
    if (!strcmp(arg, "audio"))
        av_sync_type = AV_SYNC_AUDIO_MASTER;
    else if (!strcmp(arg, "video"))
        av_sync_type = AV_SYNC_VIDEO_MASTER;
    else if (!strcmp(arg, "ext"))
        av_sync_type = AV_SYNC_EXTERNAL_CLOCK;
    else {
        fprintf(stderr, "Unknown value for %s: %s\n", opt, arg);
        exit(1);
    }
    return 0;
}

static int opt_seek(const char *opt, const char *arg)
{
    start_time = parse_time_or_die(opt, arg, 1);
    return 0;
}

static int opt_duration(const char *opt, const char *arg)
{
    duration = parse_time_or_die(opt, arg, 1);
    return 0;
}

static int opt_thread_count(const char *opt, const char *arg)
{
    thread_count= parse_number_or_die(opt, arg, OPT_INT64, 0, INT_MAX);
#if !HAVE_THREADS
    fprintf(stderr, "Warning: not compiled with thread support, using thread emulation\n");
#endif
    return 0;
}

static int opt_show_mode(const char *opt, const char *arg)
{
    show_mode = !strcmp(arg, "video") ? SHOW_MODE_VIDEO :
                !strcmp(arg, "waves") ? SHOW_MODE_WAVES :
                !strcmp(arg, "rdft" ) ? SHOW_MODE_RDFT  :
                parse_number_or_die(opt, arg, OPT_INT, 0, SHOW_MODE_NB-1);
    return 0;
}

static int opt_input_file(const char *opt, const char *filename)
{
    if (input_filename) {
        fprintf(stderr, "Argument '%s' provided as input filename, but '%s' was already specified.\n",
                filename, input_filename);
        exit(1);
    }
    if (!strcmp(filename, "-"))
        filename = "pipe:";
    input_filename = filename;
    plAddFile( ppl, filename, FALSE );
    return 0;
}

static void opt_video( const char *opt, const char *arg )
{
    if( !strcmp( arg, "auto" ))
        video_driver = KVAM_AUTO;
    else if( !strcmp( arg, "snap" ))
        video_driver = KVAM_SNAP;
    else if( !strcmp( arg, "wo" ))
        video_driver = KVAM_WO;
    else if( !strcmp( arg, "dive" ))
        video_driver = KVAM_DIVE;
    else
    {
        fprintf( stderr, "invalid video driver : %s\n", arg );
        exit( 1 );
    }
}

static void opt_audio( const char *opt, const char *arg )
{
    if( !strcmp( arg, "auto" ))
        audio_driver = KAIM_AUTO;
    else if( !strcmp( arg, "uniaud" ))
        audio_driver = KAIM_UNIAUD;
    else if( !strcmp( arg, "dart" ))
        audio_driver = KAIM_DART;
    else
    {
        fprintf( stderr, "invalid audio driver : %s\n", arg );
        exit( 1 );
    }
}

static void opt_volume( const char *opt, const char *arg )
{
    volume_level = atoi( arg );
    if( volume_level < 0 || volume_level > 100 )
    {
        fprintf( stderr, "invalid volume level : %s\n", arg );
        exit( 1 );
    }
}

static void opt_subfont( const char *opt, const char *arg )
{
    long size;
    char *facename;

    size = strtol( arg, &facename, 0 );
    if( !size || ( *facename != '.' ))
    {
        fprintf( stderr, "Invalid font name and size: %s. Use like 24.Gulim\n", arg );
        exit( 1 );
    }

    av_strlcpy( sub_fontnamesize, arg, sizeof( sub_fontnamesize ));
}

static void opt_osdfont( const char *opt, const char *arg )
{
    long size;
    char *facename;

    size = strtol( arg, &facename, 0 );
    if( !size || ( *facename != '.' ))
    {
        fprintf( stderr, "Invalid font name and size: %s. Use like 24.Gulim\n", arg );
        exit( 1 );
    }

    av_strlcpy( osd_fontnamesize, arg, sizeof( osd_fontnamesize ));
}

static void opt_aspect( const char *opt, const char *arg )
{
    if( !strcmp( arg, "none" ))
        aspect_ratio = KVAR_NONE;
    else if( !strcmp( arg, "original" ))
        aspect_ratio = KVAR_ORIGINAL;
    else if( !strcmp( arg, "force43" ))
        aspect_ratio = KVAR_FORCE43;
    else if( !strcmp( arg, "force169" ))
        aspect_ratio = KVAR_FORCE169;
    else
    {
        fprintf( stderr, "Invalid aspect ratio : %s\n", arg );
        exit( 1 );
    }
}

static void opt_brightness( const char *opt, const char *arg )
{
    char *p;
    long attr = strtol( arg, &p, 0 );

    if( *p || (( attr < 0 ) || ( attr > 255 )))
    {
        fprintf( stderr, "Invalid brightness level : %s\n", arg );
        exit( 1 );
    }

    aulAttrValue[ KVAA_BRIGHTNESS ] = attr;
}

static void opt_contrast( const char *opt, const char *arg )
{
    char *p;
    long attr = strtol( arg, &p, 0 );

    if( *p || (( attr < 0 ) || ( attr > 255 )))
    {
        fprintf( stderr, "Invalid contrast level : %s\n", arg );
        exit( 1 );
    }

    aulAttrValue[ KVAA_CONTRAST ] = attr;
}

static void opt_saturation( const char *opt, const char *arg )
{
    char *p;
    long attr = strtol( arg, &p, 0 );

    if( *p || (( attr < 0 ) || ( attr > 255 )))
    {
        fprintf( stderr, "Invalid saturation level : %s\n", arg );
        exit( 1 );
    }

    aulAttrValue[ KVAA_SATURATION ] = attr;
}

static void opt_hue( const char *opt, const char *arg )
{
    char *p;
    long attr = strtol( arg, &p, 0 );

    if( *p || (( attr < 0 ) || ( attr > 255 )))
    {
        fprintf( stderr, "Invalid hue level : %s\n", arg );
        exit( 1 );
    }

    aulAttrValue[ KVAA_HUE ] = attr;
}

static void opt_subcolor( const char *opt, const char *arg )
{
    char *p;
    long color;

    color = strtol( arg, &p, 16 );
    if( color )
        sub_color_font = color;

    if( *p == ',' )
        p++;

    color = strtol( p, &p, 16 );
    if( color )
        sub_color_outline = color;

    if( *p == ',' )
        p++;

    color = strtol( p, &p, 16 );
    if( color )
        sub_color_shadow = color;
}

static void opt_osdcolor( const char *opt, const char *arg )
{
    char *p;
    long color;

    color = strtol( arg, &p, 16 );
    if( color )
        osd_color_font = color;

    if( *p == ',' )
        p++;

    color = strtol( p, &p, 16 );
    if( color )
        osd_color_outline = color;

    if( *p == ',' )
        p++;

    color = strtol( p, &p, 16 );
    if( color )
        osd_color_shadow = color;
}

static void opt_xext( const char *opt, const char *arg )
{
    plSetExcludeExts( ppl, arg );
}

static const OptionDef options[] = {
#include "cmdutils_common_opts.h"
    { "x", HAS_ARG, {(void*)opt_width}, "force displayed width", "width" },
    { "y", HAS_ARG, {(void*)opt_height}, "force displayed height", "height" },
    { "s", HAS_ARG | OPT_VIDEO, {(void*)opt_frame_size}, "set frame size (WxH or abbreviation)", "size" },
    { "fs", OPT_BOOL, {(void*)&is_full_screen}, "force full screen" },
    { "an", OPT_BOOL, {(void*)&audio_disable}, "disable audio" },
    { "vn", OPT_BOOL, {(void*)&video_disable}, "disable video" },
    { "ast", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&opt_wanted_stream[AVMEDIA_TYPE_AUDIO]}, "select desired audio stream", "stream_number" },
    { "vst", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&opt_wanted_stream[AVMEDIA_TYPE_VIDEO]}, "select desired video stream", "stream_number" },
    { "sst", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&opt_wanted_stream[AVMEDIA_TYPE_SUBTITLE]}, "select desired subtitle stream", "stream_number" },
    { "ss", HAS_ARG, {(void*)&opt_seek}, "seek to a given position in seconds", "pos" },
    { "t", HAS_ARG, {(void*)&opt_duration}, "play  \"duration\" seconds of audio/video", "duration" },
    { "bytes", OPT_INT | HAS_ARG, {(void*)&seek_by_bytes}, "seek by bytes 0=off 1=on -1=auto", "val" },
    { "nodisp", OPT_BOOL, {(void*)&display_disable}, "disable graphical display" },
    { "f", HAS_ARG, {(void*)opt_format}, "force format", "fmt" },
    { "pix_fmt", HAS_ARG | OPT_EXPERT | OPT_VIDEO, {(void*)opt_frame_pix_fmt}, "set pixel format", "format" },
    { "stats", OPT_BOOL | OPT_EXPERT, {(void*)&show_status}, "show status", "" },
    { "bug", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&workaround_bugs}, "workaround bugs", "" },
    { "fast", OPT_BOOL | OPT_EXPERT, {(void*)&fast}, "non spec compliant optimizations", "" },
    { "genpts", OPT_BOOL | OPT_EXPERT, {(void*)&genpts}, "generate pts", "" },
    { "drp", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&decoder_reorder_pts}, "let decoder reorder pts 0=off 1=on -1=auto", ""},
    { "lowres", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&lowres}, "", "" },
    { "skiploop", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&skip_loop_filter}, "", "" },
    { "skipframe", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&skip_frame}, "", "" },
    { "skipidct", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&skip_idct}, "", "" },
    { "idct", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&idct}, "set idct algo",  "algo" },
    { "er", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&error_recognition}, "set error detection threshold (0-4)",  "threshold" },
    { "ec", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&error_concealment}, "set error concealment options",  "bit_mask" },
    { "sync", HAS_ARG | OPT_EXPERT, {(void*)opt_sync}, "set audio-video sync. type (type=audio/video/ext)", "type" },
    { "threads", HAS_ARG | OPT_EXPERT, {(void*)opt_thread_count}, "thread count", "count" },
    { "autoexit", OPT_BOOL | OPT_EXPERT, {(void*)&autoexit}, "exit at the end", "" },
    { "exitonkeydown", OPT_BOOL | OPT_EXPERT, {(void*)&exit_on_keydown}, "exit on key down", "" },
    { "exitonmousedown", OPT_BOOL | OPT_EXPERT, {(void*)&exit_on_mousedown}, "exit on mouse down", "" },
    { "loop", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&loop}, "set number of times the playback shall be looped", "loop count" },
    { "framedrop", OPT_BOOL | OPT_EXPERT, {(void*)&framedrop}, "drop frames when cpu is too slow", "" },
    { "window_title", OPT_STRING | HAS_ARG, {(void*)&window_title}, "set window title", "window title" },
#if CONFIG_AVFILTER
    { "vf", OPT_STRING | HAS_ARG, {(void*)&vfilters}, "video filters", "filter list" },
#endif
    { "rdftspeed", OPT_INT | HAS_ARG| OPT_AUDIO | OPT_EXPERT, {(void*)&rdftspeed}, "rdft speed", "msecs" },
    { "showmode", HAS_ARG, {(void*)opt_show_mode}, "select show mode (0 = video, 1 = waves, 2 = RDFT)", "mode" },
    { "default", HAS_ARG | OPT_AUDIO | OPT_VIDEO | OPT_EXPERT, {(void*)opt_default}, "generic catch all option", "" },
    { "i", HAS_ARG, {(void *)opt_input_file}, "read specified file", "input_file"},
    { "video", HAS_ARG, {(void*)opt_video}, "set video driver (driver=auto/snap/wo/dive)", "driver" },
    { "audio", HAS_ARG, {(void*)opt_audio}, "set audio driver (driver=auto/uniaud/dart)", "driver" },
    { "vol", HAS_ARG, {(void*)opt_volume}, "set initial volume level in percentage", "level" },
    { "subfont", HAS_ARG, {(void*)opt_subfont}, "set subtitle font name and size (xx=size, name=font name)", "xx.name" },
    { "osdfont", HAS_ARG, {(void*)opt_osdfont}, "set OSD font name and size (xx=size, name=font name)", "xx.name" },
    { "aspect", HAS_ARG, {(void*)opt_aspect}, "set aspect ratio (ratio=none, original, force43, force169)", "ratio" },
    { "hidemouse", OPT_BOOL, {(void*)&hide_mouse}, "hide mouse pointer on full screen mode" },
    { "brightness", HAS_ARG, {(void*)opt_brightness}, "set brightness level, except dive (level=0..255)", "level" },
    { "contrast", HAS_ARG, {(void*)opt_contrast}, "set contrast level, except dive (level=0..255)", "level" },
    { "saturation", HAS_ARG, {(void*)opt_saturation}, "set saturation level, except dive (level=0..255)", "level" },
    { "hue", HAS_ARG, {(void*)opt_hue}, "set hue level, except dive (level=0..255)", "level" },
    { "res48", OPT_BOOL, {(void*)&resample}, "resample 48KHz audio to 44.1KHz(experimental)" },
    { "subimg", OPT_BOOL, {(void*)&use_subimg}, "display subtitles/OSD to image" },
    { "subcolor", HAS_ARG, {(void*)opt_subcolor}, "set subtitle color in hexa RGB(font, outline, shadow)", "f,o,s" },
    { "osdcolor", HAS_ARG, {(void*)opt_osdcolor}, "set OSD color in hexa RGB(font, outline, shadow) ", "f,o,s" },
    { "autoadd", OPT_BOOL, {(void*)&auto_add}, "automatically add files having a similar name", "" },
    { "xext", HAS_ARG, {(void*)&opt_xext}, "exclude the following extensions on using -autoadd", "e1,e2,..." },
    { "fixt23", OPT_BOOL | OPT_EXPERT, {(void*)&fix_t23}, "workaround diagonal stripes on T23 laptop with S3 Video" },
    { "noaudioshare", OPT_BOOL | OPT_EXPERT, {(void*)&no_audio_share}, "open audio as exclusive mode" },
    { "fixsnap", OPT_BOOL | OPT_EXPERT, {(void*)&fix_snap}, "workaround for a larger movie than screen size on snap mode" },
    { "audiobufsize", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&audio_buffer_size}, "set audio buffer size in bytes", "size" },
    { "seekinterval", OPT_INT | HAS_ARG | OPT_EXPERT, {(void*)&seek_interval}, "set seek interval time in ms", "time" },
    { NULL, },
};

static void show_usage(void)
{
    printf("K Movie Player\n");
    printf("usage: kmp [options] input_file\n");
    printf("\n");
}

static int opt_help(const char *opt, const char *arg)
{
    av_log_set_callback(log_callback_help);
    show_usage();
    show_help_options(options, "Main options:\n",
                      OPT_EXPERT, 0);
    show_help_options(options, "\nAdvanced options:\n",
                      OPT_EXPERT, OPT_EXPERT);
    printf("\n");
    av_opt_show2(avcodec_opts[0], NULL,
                 AV_OPT_FLAG_DECODING_PARAM, 0);
    printf("\n");
    av_opt_show2(avformat_opts, NULL,
                 AV_OPT_FLAG_DECODING_PARAM, 0);
#if !CONFIG_AVFILTER
    printf("\n");
    av_opt_show2(sws_opts, NULL,
                 AV_OPT_FLAG_ENCODING_PARAM, 0);
#endif
    printf("\nWhile playing:\n"
           "q                   quit\n"
           "f, Enter            toggle full screen\n"
           "p, SPC              pause\n"
           "a                   cycle audio channel\n"
           "v                   cycle video channel\n"
           "t                   cycle subtitle channel\n"
           "w                   show audio waves\n"
           "s                   activate frame-step mode\n"
           "left/right          seek backward/forward 10 seconds\n"
           "Ctrl+left/right     seek backward/forward 1 minute\n"
           "mouse click         seek to position in file corresponding to fraction of width\n"
           "m                   toggle audio mute\n"
           "down/up             volume down/up by 1%%\n"
           "Ctrl+down/up        volume down/up by 5%%\n"
           "+/-                 grow up/down font size\n"
           "Alt-0/1/2/3         set aspec ratio none/original/force43/force169 respectively\n"
           "8/9                 brightness down/up by 5 level(except dive)\n"
           "i/o                 contrast down/up by 5 level(except dive)\n"
           "k/l                 saturation down/up by 5 level(except dive)\n"
           ",/.                 hue down/up by 5 level(except dive)\n"
           "/                   reset attributes(b/c/s/h) to default value(except dive)\n"
           "[/]                 correct subtitle sync -/+ 0.5 seconds\n"
           "PageUp/PageDown     play a previous/next file\n"
           ";/'                 correct audio sync -/+ 0.1 seconds(experimental)\n"
           "d                   display an information about a playing file on OSD\n"
           );
    return 0;
}

static void init_flags( void )
{
    GetCpuCaps( &gCpuCaps );
    printf("CPUflags:  MMX: %d MMX2: %d 3DNow: %d 3DNow2: %d SSE: %d SSE2: %d\n",
           gCpuCaps.hasMMX, gCpuCaps.hasMMX2,
           gCpuCaps.has3DNow, gCpuCaps.has3DNowExt,
           gCpuCaps.hasSSE, gCpuCaps.hasSSE2 );

    sws_flags = SWS_FAST_BILINEAR | SWS_PRINT_INFO;

    printf("SwScaler CPUflags : ");
    if( gCpuCaps.hasMMX )
    {
        sws_flags |= SWS_CPU_CAPS_MMX;
        printf("MMX ");
    }

    if( gCpuCaps.hasMMX2 )
    {
        sws_flags |= SWS_CPU_CAPS_MMX2;
        printf("MMX2 ");
    }

    if( gCpuCaps.has3DNow )
    {
        sws_flags |= SWS_CPU_CAPS_3DNOW;
        printf("3DNow ");
    }
    printf("\n");
}

#define WC_KMP   "WC_KMP"

/* Called from the main */
int main( int argc, char **argv )
{
    HMQ     hmq;
    ULONG   flFrameFlags;
    QMSG    qm;
    BOOL    fQuit;

    _envargs( &argc, &argv, "KMPOPT");
    _response( &argc, &argv );

    MorphToPM(); // Morph the VIO application to a PM one to be able to use Win* functions

    // Make stdout and stderr unbuffered
    setbuf( stdout, NULL );
    setbuf( stderr, NULL );

    ppl = plCreate();

    av_log_set_flags(AV_LOG_SKIP_REPEATED);

    /* register all codecs, demux and protocols */
    avcodec_register_all();
#if CONFIG_AVDEVICE
    avdevice_register_all();
#endif
#if CONFIG_AVFILTER
    avfilter_register_all();
#endif
    av_register_all();

    init_opts();

    printf("KMP " KMP_VERSION " based on FFplay " FFMPEG_VERSION ", Copyright (c) 2007-2011 KO Myung-Hun.\n");

    // KOMH: call init_flags() after init_opts() to override sws_flags
    init_flags();

    parse_options(argc, argv, options, opt_input_file);

    if( plQueryCount( ppl ) == 0 ) {
        show_usage();
        fprintf(stderr, "An input file must be specified\n");
        fprintf(stderr, "Use -h to get full help\n");
        exit(1);
    }

    if( plQueryCount( ppl ) == 1 && auto_add )
        plAddFileAuto( ppl, TOLERANCE );

    if (display_disable) {
        video_disable = 1;
    }

do
{
    fQuit = FALSE;

    cmdPlaylist = PLAYLIST_NEXT_FILE;

    input_filename = plQueryFileName( ppl );
    printf("Playing file : %s [%d/%d]\n", input_filename, plQueryIndex( ppl ) + 1, plQueryCount( ppl ));

    av_init_packet(&flush_pkt);
    flush_pkt.data= "FLUSH";

    hab = WinInitialize( 0 );
    hmq = WinCreateMsgQueue( hab, 0);

    WinRegisterClass(
        hab,
        WC_KMP,
        WndProc,
        CS_SIZEREDRAW | CS_MOVENOTIFY,
        sizeof( PVOID )
    );

    flFrameFlags = FCF_SYSMENU | FCF_TITLEBAR | FCF_MINMAX | FCF_SIZEBORDER |
                   FCF_TASKLIST;

    hwndFrame = WinCreateStdWindow(
                    HWND_DESKTOP,               // parent window handle
                    WS_VISIBLE,                 // frame window style
                    &flFrameFlags,              // window style
                    WC_KMP,                     // class name
                    KMP_NAME,                   // window title
                    0L,                         // default client style
                    NULLHANDLE,                 // resource in exe file
                    1,                          // frame window id
                    &hwndKMP                    // client window handle
                );

    if( hwndFrame != NULLHANDLE )
    {
        screen_width = opt_screen_width;
        screen_height = opt_screen_height;

        wanted_stream[AVMEDIA_TYPE_AUDIO] = opt_wanted_stream[AVMEDIA_TYPE_AUDIO];
        wanted_stream[AVMEDIA_TYPE_VIDEO] = opt_wanted_stream[AVMEDIA_TYPE_VIDEO];
        wanted_stream[AVMEDIA_TYPE_SUBTITLE] = opt_wanted_stream[AVMEDIA_TYPE_SUBTITLE];

        hwndSysMenu = WinWindowFromID( hwndFrame, FID_SYSMENU );
        hwndTitleBar = WinWindowFromID( hwndFrame, FID_TITLEBAR );
        hwndMinMax = WinWindowFromID( hwndFrame, FID_MINMAX );

        fs_screen_width = WinQuerySysValue( HWND_DESKTOP, SV_CXSCREEN );
        fs_screen_height = WinQuerySysValue( HWND_DESKTOP, SV_CYSCREEN );

        osdInit( hab, osd_fontnamesize, use_subimg );

        cur_stream = stream_open(input_filename, file_iformat);

        if( fix_t23 )
            pfnwpOldFrame = WinSubclassWindow( hwndFrame, NewFrameWndProc );

        if( !window_title )
            WinStartTimer( hab, hwndKMP, TID_SECOND, INTERVAL_SECOND );
            
        while( WinGetMsg( hab, &qm, NULLHANDLE, 0, 0 ))
            WinDispatchMsg( hab, &qm );

        if( !window_title )
            WinStopTimer( hab, hwndKMP, TID_SECOND );

        if( fix_t23 )
            WinSubclassWindow( hwndFrame, pfnwpOldFrame );

        if (cur_stream) {
            stream_close(cur_stream);
            cur_stream = NULL;
        }
        if (show_status)
            printf("\n");

        // restore mouse pointer status
        if( hide_mouse && !fMouseShow )
            SHOW_POINTER( TRUE );

        osdDone();

        WinDestroyWindow( hwndFrame );
    }

    WinDestroyMsgQueue( hmq );
    WinTerminate( hab );

    switch( cmdPlaylist )
    {
        case PLAYLIST_PREV_FILE :
            fQuit = plMovePrev( ppl ) == NULL;
            break;

        case PLAYLIST_NEXT_FILE :
            fQuit = plMoveNext( ppl ) == NULL;
            break;

        case PLAYLIST_QUIT :
            fQuit = TRUE;
            break;
    }
} while( !fQuit );

    plDestroy( ppl );

    uninit_opts();
#if CONFIG_AVFILTER
    avfilter_uninit();
#endif

    av_log(NULL, AV_LOG_QUIET, "");

    return 0;
}
