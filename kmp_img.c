/*
    KMP : Image library for OS/2
    Copyright (C) 2007-2008 by KO Myung-Hun <komh@chollian.net>

    This file is part of KMP.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.

    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    Changes :
        KO Myung-Hun <komh@chollian.net> 2007/02/25
            - Added imgQueryAttr(), imgSetAttr() and imgResetAttr()
            - Changed imgInit() to initialize attribute
            - Reset attribute on imgDone()

        KO Myung-Hun <komh@chollian.net> 2007/03/03
            - Changed the prototype of imgCreateYUV() to use m_kvas instead
              of parameters
            - Allocate memory for YUV buffer more accurately in imgCreateYUV()
            - Changed imgConvert() to use m_kvas instead of pSrcBmp->w and
              pSrcBmp->h

        KO Myung-Hun <komh@chollian.net> 2007/03/10
            - Use swscale library instead of yuv2rgb library

        KO Myung-Hun <komh@chollian.net> 2007/06/11
            - Fixed wrong color format for 15 bpp and 16 bpp mode

        KO Myung-Hun <komh@chollian.net> 2007/12/23
            - Added support of SNAP

        KO Myung-Hun <komh@chollian.net> 2007/12/30
            - Changed return type of imgInit() from VOID to APIRET for error
              checking
            - Removed HAVE_AV_CONFIG_H definition

        KO Myung-Hun <komh@chollian.net> 2008/01/07
            - Moved init codes of mm_flags and sws_flags to kmp.c

        KO Myung-Hun <komh@chollian.net> 2008/01/09
            - Removed sws_rgb2rgb_init()

        KO Myung-Hun <komh@chollian.net> 2008/01/17
            - Added support YV12, YVU9

        KO Myung-Hun <komh@chollian.net> 2008/07/27
            - Modified the path of header to the relative path to the source
              root

        KO Myung-Hun <komh@chollian.net> 2008/10/12
            - Fixed the problem attributes are not conserved when changeing
              aspect ratio

        KO Myung-Hun <komh@chollian.net> 2008/10/22
            - Fixed the color space conversion to RGB format problem
              on 15 bpp and 16 bpp modes.

                Changed dst_pix_fmt from PIX_FMT_BGR565 to PIX_FMT_RGB565
                Changed dst_pix_fmt from PIX_FMT_BGR555 to PIX_FMT_RGB555

        KO Myung-Hun <komh@chollian.net> 2008/11/16
            - Disable screen saver on init, and enable it again on done.

        KO Myung-Hun <komh@chollian.net> 2009/02/27
            - Enable 2x2 dithering on DIVE mode.
            
        KO Myung-Hun <komh@chollian.net> 2011/01/27
            - Set m_sws to NULL on initialization and termination
            - Remove emms_c() call after sws_scale() call
*/

#include <os2.h>

#include <mmioos2.h>
#include <fourcc.h>

#include <stdlib.h>
#include <types.h>

#include "libavcodec/dsputil.h"
#include "libswscale/swscale.h"

#include "kva.h"

#include "kmp_img.h"

#include "libvo/fastmemcpy.h"

int sws_flags;

static KVASETUP m_kvas = { 0 };

static struct SwsContext *m_sws = NULL;

static APIRET imgConvertInit( LONG cx, LONG cy, FOURCC *pfcc )
{
    KVACAPS kvac;
    int     dst_pix_fmt = PIX_FMT_NONE;

    kvaCaps( &kvac );

    if( kvac.ulInputFormatFlags & KVAF_YV12 )
    {
        *pfcc = FOURCC_YV12;
        dst_pix_fmt = PIX_FMT_YUV420P;
    }
    else if( kvac.ulInputFormatFlags & KVAF_YUY2 )
    {
        *pfcc = FOURCC_Y422;
        dst_pix_fmt = PIX_FMT_YUYV422;
    }
    else if( kvac.ulInputFormatFlags & KVAF_YVU9 )
    {
        *pfcc = FOURCC_YVU9;
        dst_pix_fmt = PIX_FMT_YUV410P;
    }
    else if( kvac.ulInputFormatFlags & KVAF_BGR24 )
    {
        *pfcc = FOURCC_BGR3;
        dst_pix_fmt = PIX_FMT_BGR24;
    }
    else if( kvac.ulInputFormatFlags & KVAF_BGR16 )
    {
        *pfcc = FOURCC_R565;
        dst_pix_fmt = PIX_FMT_RGB565;
    }
    else if( kvac.ulInputFormatFlags & KVAF_BGR15 )
    {
        *pfcc = FOURCC_R555;
        dst_pix_fmt = PIX_FMT_RGB555;
    }

    m_sws = NULL;
    if( dst_pix_fmt != PIX_FMT_YUV420P )
        m_sws = sws_getContext( cx, cy, PIX_FMT_YUV420P,
                                cx, cy, dst_pix_fmt,
                                sws_flags, NULL, NULL, NULL );

    return 0;
}

static VOID imgConvertDone( VOID )
{
    if( m_sws )
        sws_freeContext( m_sws );
    m_sws = NULL;
}

static VOID imgConvert( PVOID pDstBuffer, ULONG ulDstBPL, YUV *pSrcBmp )
{
    uint8_t *dst[ 3 ] = { NULL, };
    int      dstStride[ 3 ] = { 0, };

    dst[ 0 ] = pDstBuffer;
    dstStride[ 0 ] = ulDstBPL;

    if( m_kvas.fccSrcColor == FOURCC_YVU9 )
    {
        // Compute V
        dst[ 2 ] = dst[ 0 ] + m_kvas.szlSrcSize.cy * dstStride[ 0 ];
        dstStride[ 2 ] = dstStride[ 0 ] / 4;

        // Compute U
        dst[ 1 ] = dst[ 2 ] + ( m_kvas.szlSrcSize.cy / 4 ) * dstStride[ 2 ];
        dstStride[ 1 ] = dstStride[ 2 ];
    }

    sws_scale( m_sws, pSrcBmp->data, pSrcBmp->linesize, 0, m_kvas.szlSrcSize.cy,
               dst, dstStride );
}

APIRET imgInit( ULONG kvaMode, HWND hwnd, ULONG ulKeyColor, LONG cx, LONG cy,
                ULONG ulRatio, PULONG pulAttrValue )
{
    int i;

    if( kvaInit( kvaMode, hwnd, ulKeyColor ))
        return -1;

    m_kvas.ulLength = sizeof( KVASETUP );
    m_kvas.szlSrcSize.cx = cx;
    m_kvas.szlSrcSize.cy = cy;
    m_kvas.rclSrcRect.xLeft = 0;
    m_kvas.rclSrcRect.yTop = 0;
    m_kvas.rclSrcRect.xRight = cx;
    m_kvas.rclSrcRect.yBottom = cy;
    m_kvas.ulRatio = ulRatio;
    m_kvas.fDither = TRUE;

    imgConvertInit( cx, cy, &m_kvas.fccSrcColor );

    if( kvaSetup( &m_kvas ))
        return -1;

    for( i = 0; i < KVAA_LAST; i++ )
        kvaSetAttr( i, &pulAttrValue[ i ] );

    kvaDisableScreenSaver();

    return 0;
}

VOID imgDone( VOID )
{
    kvaEnableScreenSaver();

    imgConvertDone();

    imgResetAttr();

    kvaDone();
}

YUV *imgCreateYUV( VOID )
{
    YUV  *p;
    LONG lSize = m_kvas.szlSrcSize.cx * m_kvas.szlSrcSize.cy;

    p = av_malloc( sizeof( YUV ));
    if( !p )
        return NULL;

    p->p = av_malloc(( lSize * 3 ) / 2 );

    if( !p->p )
    {
        av_free( p );

        return NULL;
    }

    p->linesize[ 0 ] = m_kvas.szlSrcSize.cx;
    p->linesize[ 1 ] = m_kvas.szlSrcSize.cx / 2;
    p->linesize[ 2 ] = m_kvas.szlSrcSize.cx / 2;
    p->linesize[ 3 ] = 0;

    p->data[ 0 ] = p->p;
    p->data[ 1 ] = p->data[ 0 ] + lSize;
    p->data[ 2 ] = p->data[ 1 ] + lSize / 4;
    p->data[ 3 ] = NULL;

    return p;
}

VOID imgFreeYUV( YUV *p )
{
    if( p )
    {
        if( p->p )
            av_free( p->p );

        av_free( p );
    }
}

VOID imgDisplayYUV( YUV *bmp )
{
    PVOID       pBuffer;
    ULONG       ulBPL;

    if( !kvaLockBuffer( &pBuffer, &ulBPL ))
    {
        if( m_kvas.fccSrcColor == FOURCC_YV12 )
        {
            PBYTE pbY, pbU, pbV;
            LONG  lW, lH;

            pbY = pBuffer;
            pbV = pbY + m_kvas.szlSrcSize.cy * ulBPL;
            pbU = pbV + ( m_kvas.szlSrcSize.cy * ulBPL ) / 4;

            lW = ulBPL;
            lH = m_kvas.szlSrcSize.cy;

            // Copy Y
            mem2agpcpy_pic( pbY, bmp->data[ 0 ], lW, lH, ulBPL, bmp->linesize[ 0 ]);

            lW /= 2; lH /= 2; ulBPL /= 2;

            // Copy U
            mem2agpcpy_pic( pbU, bmp->data[ 1 ], lW, lH, ulBPL, bmp->linesize[ 1 ]);

            // Copy V
            mem2agpcpy_pic( pbV, bmp->data[ 2 ], lW, lH, ulBPL, bmp->linesize[ 2 ]);
        }
        else
            imgConvert( pBuffer, ulBPL, bmp );

        kvaUnlockBuffer();
    }
}

VOID imgClearRect( PRECTL prcl )
{
    kvaClearRect( prcl );
}

VOID imgSetAspectRatio( ULONG ulRatio )
{
    ULONG   ulValue;
    int     i;

    m_kvas.ulRatio = ulRatio;

    kvaSetup( &m_kvas );

    // Setup initializes all attributes, so need to restore them.
    for( i = 0; i < KVAA_LAST; i++ )
    {
        kvaQueryAttr( i, &ulValue );
        kvaSetAttr( i, &ulValue );
    }
}

APIRET imgQueryAttr( ULONG ulAttr, PULONG pulValue )
{
    return kvaQueryAttr( ulAttr, pulValue );
}

APIRET imgSetAttr( ULONG ulAttr, PULONG pulValue )
{
    return kvaSetAttr( ulAttr, pulValue );
}

VOID imgResetAttr( VOID )
{
    kvaResetAttr();
}

