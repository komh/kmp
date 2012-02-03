/*
    KMP : OSD library for OS/2
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
        KO Myung-Hun <komh@chollian.net> 2007/05/23
            - Reworked using Text BitMap library
*/

#ifndef __KMP_OSD_H__
#define __KMP_OSD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <os2.h>

APIRET osdInit( HAB hab, PCSZ pcszFontNameSize, BOOL fUseImg );
VOID   osdDone( VOID );
VOID   osdDisplay( HPS hps, PBYTE pbImage, LONG lImgStride, ULONG ulImgHeight,
                   LONG x, LONG y, ULONG ulMaxWidth, ULONG ulMaxHeight,
                   ULONG ulColor, ULONG ulColorOutline, ULONG ulColorShadow,
                   BOOL f3d );
VOID   osdErase( HPS hps, ULONG ulColor );
FIXED  osdQueryFontSize( VOID );
FIXED  osdSetFontSize( FIXED fxPointSize );
PSZ    osdQueryFontName( PSZ pszFontName, ULONG ulLen );
VOID   osdSetFontName( PCSZ pcszFontName );
VOID   osdInvalidate( VOID );
VOID   osdSetText( PCSZ pcszText );
BOOL   osdOn( VOID );

#ifdef __cplusplus
}
#endif

#endif
