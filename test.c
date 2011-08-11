/*
    test.c
    simple test program based on avcodec_sample.0.4.9.cpp
*/

#define INCL_DOS
#define INCL_WIN
#define INCL_GPI
#include <os2.h>

#include <mmioos2.h>
#include <fourcc.h>

#undef __STRICT_ANSI__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libswscale/rgb2rgb.h"
#include "libavcodec/dsputil.h"

#include "kva.h"
#include "libsub/subreader.h"

AVFormatContext *pFormatCtx;
AVCodecContext  *pCodecCtxV;
AVFrame         *pFrame;
AVFrame         *pFrameOS2;
int16_t         *pAudioBuf;
int             videoStream;
int             audioStream;

CHAR            szSubName[ 255 ];

BOOL            fQuit = FALSE;

void MorphToPM()
{
    PPIB pib;
    PTIB tib;

    DosGetInfoBlocks(&tib, &pib);

    // Change flag from VIO to PM:
    if( pib->pib_ultype == 2 )
        pib->pib_ultype = 3;
}

MRESULT EXPENTRY WndProc( HWND hwnd, ULONG msg, MPARAM mp1, MPARAM mp2 )
{
    return WinDefWindowProc( hwnd, msg, mp1, mp2 );
}

VOID Delay( ULONG ms )
{
    PTIB  ptib;
    ULONG ulClass;
    ULONG ulDelta;

    DosGetInfoBlocks( &ptib, NULL );
    ulClass = HIBYTE( ptib->tib_ptib2->tib2_ulpri );
    ulDelta = LOBYTE( ptib->tib_ptib2->tib2_ulpri );

    DosSetPriority( PRTYS_THREAD, PRTYC_TIMECRITICAL, PRTYD_MAXIMUM, 0 );
    DosSleep( ms );
    DosSetPriority( PRTYS_THREAD, ulClass, ulDelta, 0 );
}

VOID DisplaySub( HPS hps, PRECTL prcl, subtitle *sub, ULONG ulColor )
{
    FONTMETRICS fm;
    ULONG       ulHeight;
    POINTL      aptl[ TXTBOX_COUNT ];
    POINTL      ptl;
    int         len, extent;
    int         i;

    GpiQueryFontMetrics( hps, sizeof( FONTMETRICS ), &fm );
    ulHeight = fm.lMaxBaselineExt + fm.lExternalLeading;

    ptl.y = ulHeight * ( sub->lines - 1 ) + fm.lMaxDescender;

    GpiCreateLogColorTable( hps, 0, LCOLF_RGB, 0, 0, NULL );
    GpiSetColor( hps, ulColor );

    for( i = 0; i < sub->lines; i++ )
    {
        len = strlen( sub->text[ i ]);
        GpiQueryTextBox( hps, len, sub->text[ i ], TXTBOX_COUNT, aptl );
        extent = aptl[ TXTBOX_CONCAT ].x - aptl[ TXTBOX_BOTTOMLEFT ].x;
        ptl.x = (( prcl->xRight - prcl->xLeft ) - extent ) / 2 + prcl->xLeft;

        GpiCharStringAt( hps, &ptl, strlen( sub->text[ i ]  ), sub->text[ i ]);

        ptl.y -= ulHeight;
    }
}

void VideoThread( void *arg )
{
    HWND        hwnd = ( HWND )arg;
    AVPacket    packet;
    int         frameFinished;
    ULONG       frame_delay = av_q2d( pFormatCtx->streams[ videoStream ]->time_base ) * 1000;

    sub_data  * sd;
    PSZ         pszExt;
    subtitle  * cur_sub = NULL;
    subtitle  * prev_sub = NULL;
    ULONG       ulCurTime = 0;

    PVOID   pBuffer, pBuffer1;
    ULONG   ulBPL;

    HPS     hps;
    RECTL   rcl;

    struct SwsContext * sws;
    int    sws_flags = SWS_FAST_BILINEAR | SWS_PRINT_INFO;

    mm_flags = mm_support();

    if( mm_flags & MM_MMX )
        sws_flags |= SWS_CPU_CAPS_MMX;

    if( mm_flags & MM_MMXEXT )
        sws_flags |= SWS_CPU_CAPS_MMX2;

    if( mm_flags & MM_3DNOW )
        sws_flags |= SWS_CPU_CAPS_3DNOW;

    sws_rgb2rgb_init( sws_flags );

    sws = sws_getContext( pCodecCtxV->width, pCodecCtxV->height, pCodecCtxV->pix_fmt,
                          pCodecCtxV->width, pCodecCtxV->height, PIX_FMT_YUYV422,
                          sws_flags, NULL, NULL, NULL );

    WinSetPresParam( hwnd, PP_FONTNAMESIZE, 8, "32.±¼¸²" );

    hps = WinGetPS( hwnd );

    WinQueryWindowRect( hwnd, &rcl );

    pszExt = strrchr( szSubName, '.' );
    strcpy( pszExt, ".smi" );

    sd = sub_read_file( strdup( szSubName ), 0 );

    if( sd )
    {
#if 0
        int i, j;

        printf("subtitle info \n");
        printf("filename = %s, uses_time = %d, num = %d, errs = %d\n",
               sd->filename, sd->sub_uses_time, sd->sub_num, sd->sub_errs );

        for( i = 0; i < sd->sub_num; i++ )
        {
            printf("%dth subtitle info\n", i );
            printf("lines = %d, start = %lu, end = %lu, alignment = %d\n",
                   sd->subtitles[ i ].lines, sd->subtitles[ i ].start, sd->subtitles[ i ].end, sd->subtitles[ i ].alignment );

            for( j = 0; j < sd->subtitles[ i ].lines; j++ )
                printf("  %dth line = %s\n", j, sd->subtitles[ i ].text[ j ]);
        }
#endif
        prev_sub = cur_sub = sd->subtitles;
    }

    printf("video: stream time_base = %f, codec time_base = %f\n",
           av_q2d( pFormatCtx->streams[ videoStream ]->time_base),
           av_q2d( pCodecCtxV->time_base ));
    printf("audio: stream time_base = %f, codec time_base = %f\n",
           av_q2d( pFormatCtx->streams[ audioStream ]->time_base),
           av_q2d( pFormatCtx->streams[ audioStream ]->codec->time_base ));

    while( !fQuit )
    {
        if( av_read_frame( pFormatCtx, &packet ) < 0 )
        {
            printf("Maybe reached at end\n");
            continue;
        }

    #if 0
        if( packet.stream_index == audioStream )
        {
            printf("audio : pts = %Ld, dts = %Ld, duration = %d\n",
                   packet.pts, packet.dts, packet.duration );
        }
    #endif
        // Is this a packet from the video stream?
        if( packet.stream_index == videoStream )
        {
            // Decode video frame
            avcodec_decode_video( pCodecCtxV, pFrame, &frameFinished,
                                  packet.data, packet.size );

            // Did we get a video frame?
            if(frameFinished)
            {
            #if 0
                if( packet.pts == AV_NOPTS_VALUE )
                    printf("video : pts = AV_NOPTS_VALUE, dts = %Ld, duration = %d           \n",
                           packet.dts, packet.duration );
                else
                    printf("video : pts = %Ld, dts = %Ld, duration = %d               \n",
                           packet.pts, packet.dts, packet.duration );
            #endif
                kvaLockBuffer( &pBuffer, &ulBPL );

                //pBuffer1 = av_malloc( pCodecCtxV->height * ulBPL );

                if( sws )
                {
                    sws_scale( sws, pFrame->data, pFrame->linesize, 0, pCodecCtxV->height,
                               &pBuffer, &ulBPL );

                    emms_c();
                }

                //memcpy( pBuffer, pBuffer1, pCodecCtxV->height * ulBPL );

                //av_free( pBuffer1 );

            if( sd )
            {
                if( prev_sub != cur_sub )
                {
                    DisplaySub( hps, &rcl, prev_sub, 8 );
                    prev_sub = cur_sub;
                }
            }

                kvaUnlockBuffer();

            if( sd )
            {
                ulCurTime = frame_delay * packet.dts / 10; // in 1/100 s
            #if 0
                printf("ulCurTime = %lu, start = %lu, end = %lu\n",
                       ulCurTime, cur_sub->start, cur_sub->end );
            #endif

                if( ulCurTime >= cur_sub->start && ulCurTime <= cur_sub->end )
                {
                    DisplaySub( hps, &rcl, cur_sub, 0xFFFFFF );
                }
                else
                {
                    prev_sub = cur_sub;

                    while( cur_sub != sd->subtitles && ulCurTime < cur_sub->start )
                        cur_sub--;
                    while( cur_sub->lines && ulCurTime > cur_sub->end )
                        cur_sub++;
                }
            }

                Delay( frame_delay );
            }
        }

        // Free the packet that was allocated by av_read_frame
        av_free_packet(&packet);
    }

    sub_free( sd );

    WinReleasePS( hps );

    if( sws )
        sws_freeContext( sws );

    _endthread();
}

int main(int argc, char *argv[])
{
    HAB     hab;
    HMQ     hmq;
    ULONG   flFrameFlags;
    HWND    hwndFrame;
    HWND    hwndClient;
    QMSG    qm;
    PSZ     szWndClass = "KMP_CLASS";
    PSZ     szAppTitle = "K Movie Player";

    TID     tid_video;

    KVASETUP KvaSetup = { 0 };
    RECTL    rcl;

    int      i;
    AVCodec  *pCodecV;

    int ret = 1;

    if( argc < 2 )
    {
        fprintf( stderr, "Specify input file\n");

        return ret;
    }

    // Morph the VIO application to a PM one to be able to use Win* functions
    MorphToPM();

    // Make stdout and stderr unbuffered
    setbuf( stdout, NULL );
    setbuf( stderr, NULL );

    hab = WinInitialize( 0 );
    hmq = WinCreateMsgQueue( hab, 0);

    WinRegisterClass(
        hab,
        szWndClass,
        WndProc,
        CS_SIZEREDRAW,
        sizeof( PVOID )
    );

    flFrameFlags = FCF_SYSMENU | FCF_TITLEBAR | FCF_MINMAX | FCF_SIZEBORDER |
                   FCF_SHELLPOSITION | FCF_TASKLIST;

    hwndFrame = WinCreateStdWindow (
                HWND_DESKTOP,
                WS_VISIBLE ,
                &flFrameFlags,
                szWndClass,
                szAppTitle,
                0,
                0,
                1,
                &hwndClient);

    if( kvaInit( KVAM_WO, hwndClient, 0x000008 ))
    {
       WinMessageBox( HWND_DESKTOP, HWND_DESKTOP,
                      "Error",
                      "Can't init overlay!", 0, MB_ICONHAND | MB_OK);

       goto exit_frame;
    }

    // Register all formats and codecs
    av_register_all();

    // Open video file
    if( av_open_input_file( &pFormatCtx, argv[1], NULL, 0, NULL ) != 0 )
        goto exit_kva; // Couldn't open file

    strcpy( szSubName, argv[ 1 ]);

    // Retrieve stream information
    if( av_find_stream_info( pFormatCtx ) < 0 )
        goto exit_close_file; // Couldn't find stream information

    // Dump information about file onto standard error
    dump_format( pFormatCtx, 0, argv[ 1 ], FALSE );

    // Find the first video stream
    videoStream = -1;
    audioStream = -1;
    for( i = 0; i < pFormatCtx->nb_streams; i++ )
    {
        switch( pFormatCtx->streams[ i ]->codec->codec_type )
        {
            case CODEC_TYPE_VIDEO :
                if( videoStream < 0 )
                    videoStream = i;
                break;

            case CODEC_TYPE_AUDIO :
                if( audioStream < 0 )
                    audioStream = i;
                break;
        }
    }
    if( videoStream < 0 )
        goto exit_close_file; // Didn't find a video stream

    // Get a pointer to the codec context for the video stream
    pCodecCtxV = pFormatCtx->streams[ videoStream ]->codec;

    // Find the decoder for the video stream
    pCodecV = avcodec_find_decoder( pCodecCtxV->codec_id );
    if( pCodecV == NULL )
        goto exit_close_file; // Codec not found

    // Inform the codec that we can handle truncated bitstreams -- i.e.,
    // bitstreams where frame boundaries can fall in the middle of packets
    //if( pCodecV->capabilities & CODEC_CAP_TRUNCATED )
        //pCodecCtxV->flags |= CODEC_FLAG_TRUNCATED;

    // Open codec
    if( avcodec_open( pCodecCtxV, pCodecV ) < 0 )
        goto exit_close_file; // Could not open codec

    // Allocate video frame
    pFrame = avcodec_alloc_frame();

    // Allocate an AVFrame structure
    pFrameOS2 = avcodec_alloc_frame();

    KvaSetup.ulLength = sizeof( KVASETUP );
    KvaSetup.szlSrcSize.cx = pCodecCtxV->width;
    KvaSetup.szlSrcSize.cy = pCodecCtxV->height;
    KvaSetup.fccSrcColor = FOURCC_Y422;
    KvaSetup.rclSrcRect.xLeft = 0;
    KvaSetup.rclSrcRect.yTop = 0;
    KvaSetup.rclSrcRect.xRight = pCodecCtxV->width;
    KvaSetup.rclSrcRect.yBottom = pCodecCtxV->height;
    KvaSetup.ulRatio = KVAR_ORIGINAL;
    if( kvaSetup( &KvaSetup ))
    {
       WinMessageBox (HWND_DESKTOP, HWND_DESKTOP,
        "Error",
        "Can't setup overlay!", 0, MB_ICONHAND | MB_OK);

       goto exit_close_codec;
    }


    WinSetWindowPos( hwndFrame, HWND_TOP, 100, 100,
                     pCodecCtxV->width, pCodecCtxV->height,
                     SWP_SIZE | SWP_MOVE | SWP_SHOW | SWP_ZORDER | SWP_ACTIVATE );
    WinQueryWindowRect( hwndClient, &rcl );
    WinSetWindowPos( hwndFrame, HWND_TOP, 100, 100,
                     2 * pCodecCtxV->width - rcl.xRight + rcl.xLeft,
                     2 * pCodecCtxV->height -rcl.yTop + rcl.yBottom,
                     SWP_SIZE | SWP_MOVE | SWP_SHOW | SWP_ZORDER | SWP_ACTIVATE );

    tid_video = _beginthread( VideoThread, NULL, 32768, ( void * )hwndClient );

    while( WinGetMsg( hab, &qm, NULLHANDLE, 0, 0 ))
          WinDispatchMsg( hab, &qm );

    fQuit = TRUE;

    DosWaitThread( &tid_video, DCWW_WAIT );

    // Free the RGB image
    av_free(pFrameOS2);

    // Free the YUV frame
    av_free(pFrame);

    ret = 0;

exit_close_codec:
    // Close the codec
    avcodec_close(pCodecCtxV);

exit_close_file:
    // Close the video file
    av_close_input_file(pFormatCtx);

exit_kva:
    kvaDone();

exit_frame:
    WinDestroyWindow( hwndFrame );
    WinDestroyMsgQueue( hmq );
    WinTerminate( hab );

    return ret;
}
