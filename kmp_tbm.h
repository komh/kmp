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
        KO Myung-Hun <komh@chollian.net> 2008/05/03
            - Removed TBMA_LEFT
            - Added TBMA_TOP
*/

#ifndef __KMP_TBM_H__
#define __KMP_TBM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <os2.h>

#define TBMA_CENTER 0x0001  /* if not set, LEFT */
#define TBMA_TOP    0x0002  /* if not set, BOTTOM */

typedef struct tagTEXTLIST TEXTLIST, *PTEXTLIST;

struct tagTEXTLIST
{
    PSZ         pszText;
    LONG        lExtent;
    PTEXTLIST   ptlNext;
};

typedef struct tagTEXTBITMAP
{
    HDC         hdc;
    HPS         hps;
    HBITMAP     hbm;
    RECTL       rclCurrent;
    LONG        lHoriFontRes;
    LONG        lVertFontRes;
    FIXED       fxPointSize;
    CHAR        szFaceName[ FACESIZE + 1 ];
    BOOL        fUseImg;;
    PBYTE       pbBitmap;
    PBYTE       pbAlpha;
    LONG        lStride;
    PTEXTLIST   ptlStart;
    PTEXTLIST   ptlEnd;
} TEXTBITMAP, *PTEXTBITMAP;

PTEXTBITMAP tbmInit( HAB hab, PCSZ pcszFontNameSize, BOOL fUseImg );
VOID        tbmDone( PTEXTBITMAP ptb );
VOID        tbmCreate( PTEXTBITMAP ptb,
                       LONG x, LONG y, ULONG ulMaxWidth, ULONG ulMaxHeight,
                       ULONG ulColor, ULONG ulColorOutline, ULONG ulColorShadow,
                       BOOL f3d, ULONG ulAlign );
VOID        tbmDelete( PTEXTBITMAP ptb );
VOID        tbmAddText( PTEXTBITMAP ptb, PCSZ pcszText );
VOID        tbmClearText( PTEXTBITMAP ptb );
VOID        tbmDraw( PTEXTBITMAP ptbm, HPS hps, PBYTE pbImage, LONG lImgStride, ULONG ulImgHeight );

#ifdef __cplusplus
}
#endif

#endif
