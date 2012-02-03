/*
    KMP : Text BitMap library for OS/2
    Copyright (C) 2007-2012 by KO Myung-Hun <komh@chollian.net>

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
        KO Myung-Hun <komh@chollian.net> 2007/06/03
            - Fixed crash when ulMaxHeight == 0

        KO Myung-Hun <komh@chollian.net> 2007/06/11
            - Fixed outline of font is not displayed cleanly on 15bpp and
              16bpp mode

        KO Myung-Hun <komh@chollian.net> 2007/06/11
            - Fixed crash in tbmDelete() if ptb == NULL

        KO Myung-Hun <komh@chollian.net> 2008/05/03
            - Implemented the feature for TBMA_TOP in tbmCreate()

        KO Myung-Hun <komh@chollian.net> 2011/01/27
            - if pbImgae is NULL, then use Gpi APIs in tbmDraw()
*/

#define INCL_WIN
#define INCL_GPI
#include <os2.h>

#include <stdlib.h>
#include <string.h>

#include "kmp_tbm.h"

extern void vo_draw_alpha_yv12( int w, int h, unsigned char *src, unsigned char *srca,
                                int srcstride, unsigned char *dstbase, int dststride );

static PTEXTLIST tlNew( PCSZ pcszText )
{
    PTEXTLIST ptlNew;

    if( !pcszText )
        return NULL;

    ptlNew = malloc( sizeof( TEXTLIST ));
    ptlNew->pszText = strdup( pcszText );
    ptlNew->ptlNext = NULL;

    return ptlNew;
}

static VOID tlLinkTo( PTEXTLIST ptlTo, PTEXTLIST ptlNew )
{
    if( !ptlTo || !ptlNew )
        return;

    ptlNew->ptlNext = ptlTo->ptlNext;
    ptlTo->ptlNext = ptlNew;
}

static VOID tlReplaceText( PTEXTLIST ptl, PCH pchText, LONG lLen )
{
    if( !ptl || !pchText )
        return;

    if( ptl->pszText )
        free( ptl->pszText );

    if( lLen == -1 )
        lLen = strlen( pchText );

    ptl->pszText = malloc( lLen + 1 );
    memcpy( ptl->pszText, pchText, lLen );
    ptl->pszText[ lLen ] = 0;
}

static BOOL tlSplitText( PTEXTLIST ptl )
{
    char *  temp;
    int     len;
    int     i;

    if( !ptl )
        return FALSE;

    len = strlen( ptl->pszText );
    if( len < 5 )
        return FALSE;

    temp = strdup( ptl->pszText );
    i = len / 2;

    while( temp[ i ] && temp[ i ] != ' ' )
        i++;

    if( !temp[ i ])
    {
        while( i && temp[ i ] != ' ' )
            i--;

        // FIXME : need to check DBCS chars.
        if( i == 0 )
            i = len / 2;
    }

    tlReplaceText( ptl, temp, i );

    while( temp[ i ] && temp[ i ] == ' ' )
        i++;

    tlLinkTo( ptl, tlNew( temp + i ));

    free( temp );

    return TRUE;
}

static VOID tlFreeAll( PTEXTLIST ptlStart )
{
    PTEXTLIST ptl, ptlNext;

    for( ptl = ptlStart; ptl; ptl = ptlNext )
    {
        ptlNext = ptl->ptlNext;

        if( ptl->pszText )
            free( ptl->pszText );

        free( ptl );
    }
}

PTEXTBITMAP tbmInit( HAB hab, PCSZ pcszFontNameSize, BOOL fUseImg )
{
    PTEXTBITMAP ptb;

    HPS     hps;
    HDC     hdc;
    char   *facename;
    SIZEL   sizl;

    ptb = malloc( sizeof( TEXTBITMAP ));
    memset( ptb, 0, sizeof( TEXTBITMAP ));

    hps = WinGetScreenPS( HWND_DESKTOP );
    hdc = GpiQueryDevice( hps );
    DevQueryCaps( hdc, CAPS_HORIZONTAL_FONT_RES, 1, &ptb->lHoriFontRes );
    DevQueryCaps( hdc, CAPS_VERTICAL_FONT_RES, 1, &ptb->lVertFontRes );
    WinReleasePS( hps );

    ptb->fxPointSize = MAKEFIXED( strtol( pcszFontNameSize, &facename, 0 ), 0 );
    strcpy( ptb->szFaceName, facename + 1 );

    ptb->hdc = DevOpenDC( hab, OD_MEMORY, "*", 0L, NULL, NULLHANDLE );

    sizl.cx = 0;
    sizl.cy = 0;
    ptb->hps = GpiCreatePS( hab, ptb->hdc, &sizl,
                         PU_PELS | GPIF_DEFAULT | GPIT_MICRO | GPIA_ASSOC );

    ptb->fUseImg = fUseImg;

    return ptb;
}

VOID tbmDone( PTEXTBITMAP ptb )
{
    if( !ptb )
        return;

    tbmDelete( ptb );

    if( ptb->ptlStart )
        tlFreeAll( ptb->ptlStart );

    GpiDestroyPS( ptb->hps );

    DevCloseDC( ptb->hdc );

    free( ptb );
}

#define SHADOW_DISTANCE     1
#define SHADOW_THICKNESS    2

static VOID outliner( HPS hps, INT x, INT y, PSZ szMsg,
                      LONG color, LONG colorOutline, LONG colorShadow, BOOL f3d )
{
    LONG        outlineColor[] = { 0x000000, };
    INT         outlineN = sizeof( outlineColor ) / sizeof( outlineColor[ 0 ] );
    INT         i, j;
    INT         len = strlen( szMsg );
    POINTL      ptl;
    LONG        oldColor;

    oldColor = GpiQueryColor( hps );

    outlineColor[ 0 ] = colorOutline;

    if( f3d )
    {
        for( i = 0; i < SHADOW_THICKNESS; i++ )
        {
            GpiSetColor( hps, colorShadow );
            ptl.x = x + ( outlineN + SHADOW_DISTANCE + i );
            ptl.y = y - ( outlineN + SHADOW_DISTANCE + i );
            GpiCharStringAt( hps, &ptl, len, szMsg );
        }
    }

    for( i = outlineN; i > 0; i-- )
    {

        GpiSetColor( hps, outlineColor[ i - 1 ]);

        for( j = -i; j <= i; j++ )
        {
            ptl.x = x - i;
            ptl.y = y + j;
            GpiCharStringAt( hps, &ptl, len, szMsg );

            ptl.x = x + i;
            ptl.y = y + j;
            GpiCharStringAt( hps, &ptl, len, szMsg );
        }

        for( j = -( i - 1 ); j <= ( i - 1 ); j++ )
        {
            ptl.x = x + j;
            ptl.y = y + i;
            GpiCharStringAt( hps, &ptl, len, szMsg );

            ptl.x = x + j;
            ptl.y = y - i;
            GpiCharStringAt( hps, &ptl, len, szMsg );
        }
    }

    GpiSetColor( hps, color );
    ptl.x = x;
    ptl.y = y;
    GpiCharStringAt( hps, &ptl, len, szMsg );

    GpiSetColor( hps, oldColor );
}

static VOID findFont( HPS hps, LONG lHoriFontRes, LONG lVertFontRes,
                      FIXED fxPointSize, PCSZ pcszFacename, PFATTRS pfat, PSIZEF psizf )
{
    LONG         cFonts = 0;
    PFONTMETRICS pfm;
    int          i;

    memset( pfat, 0, sizeof( FATTRS ));
    pfat->usRecordLength = sizeof( FATTRS );
    strcpy( pfat->szFacename, pcszFacename );

    memset( psizf, 0, sizeof( SIZEF ));

    cFonts = GpiQueryFonts( hps, QF_PUBLIC | QF_PRIVATE, pcszFacename, &cFonts,
                            sizeof( FONTMETRICS ), NULL );

    pfm = malloc( sizeof( FONTMETRICS ) * cFonts );

    GpiQueryFonts( hps, QF_PUBLIC | QF_PRIVATE, pcszFacename, &cFonts,
                   sizeof( FONTMETRICS ), pfm );

    for( i = 0; i < cFonts; i++ )
    {
        if( pfm[ i ].fsDefn & FM_DEFN_OUTLINE ) // outline font
            break;

        if(( pfm[ i ].sXDeviceRes == lHoriFontRes ) &&
           ( pfm[ i ].sYDeviceRes == lVertFontRes ) &&
           ( pfm[ i ].sNominalPointSize / 10 == FIXEDINT( fxPointSize )))
           break;
    }

    if( i < cFonts )    // found wanted font
    {
        if( pfm[ i ].fsDefn & FM_DEFN_OUTLINE ) // outline font
        {
            psizf->cx = (( fxPointSize / 72 * lHoriFontRes ) + 0x10000L ) & -0x20000L;
            psizf->cy = fxPointSize / 72 * lVertFontRes;
        }
        else    // raster font
        {
            pfat->lMaxBaselineExt = pfm[ i ].lMaxBaselineExt;
            pfat->lAveCharWidth = pfm[ i ].lAveCharWidth;

            psizf->cx = pfm[ i ].lEmInc;
            psizf->cy = pfm[ i ].lEmHeight;
        }
    }

    free( pfm );
}

#define COMPUTE_Y( rgb ) ( 0.299 * ((( rgb ) >> 16 ) & 0xFF ) + \
                           0.587 * ((( rgb ) >>  8 ) & 0xFF ) + \
                           0.114 *  (( rgb )         & 0xFF ))

VOID tbmCreate( PTEXTBITMAP ptb,
                LONG x, LONG y, ULONG ulMaxWidth, ULONG ulMaxHeight,
                ULONG ulColor, ULONG ulColorOutline, ULONG ulColorShadow,
                BOOL f3d, ULONG ulAlign )
{
    PTEXTLIST         ptlStart, ptl, ptl1;
    FATTRS            fat;
    SIZEF             sizf;
    BITMAPINFOHEADER2 bmih2;
    FONTMETRICS       fm;
    ULONG             ulHeight;
    POINTL            aptl[ TXTBOX_COUNT ];
    int               x1, y1;
    int               bmp_width, bmp_height;
    int               len, extent;
    int               i, j;

    if( !ptb || !ptb->ptlStart || ptb->hbm || !ulMaxWidth || !ulMaxHeight )
        return;

    ptlStart = tlNew( ptb->ptlStart->pszText );
    ptl = ptlStart;
    ptl1 = ptb->ptlStart->ptlNext;
    while( ptl1 )
    {
        tlLinkTo( ptl, tlNew( ptl1->pszText ));

        ptl = ptl->ptlNext;
        ptl1 = ptl1->ptlNext;
    }

    findFont( ptb->hps, ptb->lHoriFontRes, ptb->lVertFontRes,
              ptb->fxPointSize, ptb->szFaceName, &fat, &sizf );
    fat.fsFontUse = FATTR_FONTUSE_NOMIX;

    GpiCreateLogFont( ptb->hps, NULL, 1, &fat );
    GpiSetCharSet( ptb->hps, 1 );

    GpiSetCharBox( ptb->hps, &sizf );

    bmp_width = 0;

    for( ptl = ptlStart; ptl; )
    {
        len = strlen( ptl->pszText);
        GpiQueryTextBox( ptb->hps, len, ptl->pszText, TXTBOX_COUNT, aptl );
        // consider outline and shadow extent
        extent = aptl[ TXTBOX_CONCAT ].x + 2 + ( SHADOW_DISTANCE + SHADOW_THICKNESS - 1 );
        if( extent > ulMaxWidth )
        {
            if( tlSplitText( ptl ))
                continue;
        }

        if( bmp_width < extent )
            bmp_width = extent;

        ptl->lExtent = extent;

        ptl = ptl->ptlNext;
    }

    if( bmp_width > ulMaxWidth )
        bmp_width = ulMaxWidth;

    GpiQueryFontMetrics( ptb->hps, sizeof( FONTMETRICS ), &fm );
    // consider outline and shadow extent
    ulHeight = fm.lMaxBaselineExt + fm.lExternalLeading + 2 + ( SHADOW_DISTANCE + SHADOW_THICKNESS - 1 );

    bmp_height = 0;
    for( ptl = ptlStart; ptl; ptl = ptl->ptlNext )
        bmp_height += ulHeight;

    memset( &bmih2, 0, sizeof( BITMAPINFOHEADER2 ));

    bmih2.cbFix = sizeof( BITMAPINFOHEADER2 );
    bmih2.cx = bmp_width;
    bmih2.cy = bmp_height < ulMaxHeight ? bmp_height : ulMaxHeight;
    bmih2.cPlanes = 1;
    bmih2.cBitCount = 24;

    ptb->hbm = GpiCreateBitmap( ptb->hps, &bmih2, 0, NULL, NULL );

    GpiSetBitmap( ptb->hps, ptb->hbm );

    GpiCreateLogColorTable( ptb->hps, 0, LCOLF_RGB, 0, 0, NULL );

    if( ulAlign & TBMA_TOP )
        bmp_height = bmih2.cy;

    // consider outline and shadow
    y1 = fm.lMaxDescender + ( bmp_height - ulHeight ) + 1 + ( SHADOW_DISTANCE + SHADOW_THICKNESS - 1 );

    for( ptl = ptlStart; ptl; ptl = ptl->ptlNext )
    {
        x1 = 1; // 1 for left outline
        if( ulAlign & TBMA_CENTER )
            x1 += ( bmih2.cx - ptl->lExtent ) / 2;

        outliner( ptb->hps, x1, y1, ptl->pszText,
                  ulColor, ulColorOutline, ulColorShadow, f3d );

        y1 -= ulHeight;
    }

    if( ptb->fUseImg )
    {
        PBYTE pbBitsBase, pbBits;
        PBYTE pba, pbb;
        ULONG ulRGB;

        ptb->lStride = ( bmih2.cx + 7 ) & -8; // 8 bytes alignment

        ptb->pbBitmap = malloc( ptb->lStride * bmih2.cy );
        ptb->pbAlpha = malloc( ptb->lStride * bmih2.cy );

        pbBitsBase = malloc((( bmih2.cBitCount * bmih2.cx + 31 ) / 32 ) * 4 * bmih2.cPlanes );

        pbb = ptb->pbBitmap;
        pba = ptb->pbAlpha;

        for( i = bmih2.cy - 1; i >= 0; i-- )
        {
            GpiQueryBitmapBits( ptb->hps, i, 1, pbBitsBase, &bmih2 );

            pbBits = pbBitsBase;
            for( j = 0; j < bmih2.cx; j++ )
            {
                ulRGB = *( PULONG )pbBits & 0xFFFFFF;
                pbBits += 3;

                if( ulRGB )
                {
                    *pbb = COMPUTE_Y( ulRGB );
                    *pba = 1;
                }
                else
                {
                    *pbb = 0;
                    *pba = 0;
                }

                pbb++;
                pba++;
            }

            // padding
            while( j < ptb->lStride )
            {
                *pbb++ = 0;
                *pba++ = 0;
                j++;
            }
        }

        free( pbBitsBase );
    }

    tlFreeAll( ptlStart );

    ptb->rclCurrent.xLeft = x;
    if( ulAlign & TBMA_CENTER )
        ptb->rclCurrent.xLeft += ( ulMaxWidth - bmih2.cx ) / 2;
    ptb->rclCurrent.xRight = ptb->rclCurrent.xLeft + bmih2.cx;
    if( ulAlign & TBMA_TOP )
    {
        ptb->rclCurrent.yTop = y;
        ptb->rclCurrent.yBottom = ptb->rclCurrent.yTop - bmih2.cy;
    }
    else
    {
        ptb->rclCurrent.yBottom = y;
        ptb->rclCurrent.yTop = ptb->rclCurrent.yBottom + bmih2.cy;
    }

}

VOID tbmDelete( PTEXTBITMAP ptb )
{
    if( !ptb )
        return;

    if( ptb->fUseImg )
    {
        if( ptb->pbBitmap )
            free( ptb->pbBitmap );

        if( ptb->pbAlpha )
            free( ptb->pbAlpha );

        ptb->pbBitmap = NULL;
        ptb->pbAlpha = NULL;
    }

    GpiSetBitmap( ptb->hps, NULLHANDLE );
    GpiDeleteBitmap( ptb->hbm );
    ptb->hbm = NULLHANDLE;
}

VOID tbmDraw( PTEXTBITMAP ptbm, HPS hps, PBYTE pbImage, LONG lImgStride, ULONG ulImgHeight )
{
    if( !ptbm )
        return;

    if( ptbm->fUseImg && pbImage )
    {
        vo_draw_alpha_yv12( ptbm->rclCurrent.xRight - ptbm->rclCurrent.xLeft,
                            ptbm->rclCurrent.yTop - ptbm->rclCurrent.yBottom,
                            ptbm->pbBitmap, ptbm->pbAlpha, ptbm->lStride,
                            pbImage + ( ulImgHeight - ptbm->rclCurrent.yTop - 1 ) * lImgStride + ptbm->rclCurrent.xLeft,
                            lImgStride );
    }
    else
    {
        POINTL aptl[ 3 ];

        // target
        aptl[ 0 ].x = ptbm->rclCurrent.xLeft;
        aptl[ 0 ].y = ptbm->rclCurrent.yBottom;
        aptl[ 1 ].x = ptbm->rclCurrent.xRight;
        aptl[ 1 ].y = ptbm->rclCurrent.yTop;

        // source
        aptl[ 2 ].x = 0;
        aptl[ 2 ].y = 0;

        GpiBitBlt( hps, ptbm->hps, 3, aptl, ROP_SRCPAINT, BBO_IGNORE );
    }
}

VOID tbmAddText( PTEXTBITMAP ptb, PCSZ pcszText )
{
    PTEXTLIST ptlNew;

    if( !ptb || !pcszText )
        return;

    ptlNew = tlNew( pcszText );
    if( ptb->ptlStart )
    {
        tlLinkTo( ptb->ptlEnd, ptlNew );

        ptb->ptlEnd = ptlNew;
    }
    else
        ptb->ptlEnd = ptb->ptlStart = ptlNew;
}

VOID tbmClearText( PTEXTBITMAP ptb )
{
    if( !ptb || !ptb->ptlStart )
        return;

    tlFreeAll( ptb->ptlStart );

    ptb->ptlEnd = ptb->ptlStart = NULL;
}

