/*
    KMP : Subtitle library for OS/2
    Copyright (C) 2007 by KO Myung-Hun <komh@chollian.net>

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
        KO Myung-Hun <komh@chollian.net> 2007/02/22
            - Added subQueryNextFontSize() and subQueyPrevFontSize().

        KO Myung-Hun <komh@chollian.net> 2007/02/25
            - Added subQueryDelta() and subSetDelta()

        KO Myung-Hun <komh@chollian.net> 2007/05/23
        	- Reworked using Text BitMap library
        	- Merged SubQueryNextFontSize() and SubQueryPrevFontSize() into
        	  SubQueryFontSize()
*/

#ifndef __KMP_SUB_H__
#define __KMP_SUB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <os2.h>

#define SQFS_CURRENT 	0
#define SQFS_NEXT       1
#define SQFS_PREV       2

APIRET subInit( HAB hab, PCSZ pcszFileName, PCSZ pcszFontNameSize, float fps, BOOL fUseImg );
VOID   subDone( VOID );
BOOL   subInTime( ULONG ulTime );
VOID   subFindNext( ULONG ulTime );
VOID   subDisplay( HPS hps, PBYTE pbImage, LONG lImgStride, ULONG ulImgHeight,
                   LONG x, LONG y, ULONG ulMaxWidth, ULONG ulMaxHeight,
                   ULONG ulColor, ULONG ulColorOutline, ULONG ulColorShadow,
                   BOOL f3d );
VOID   subErase( HPS hps, ULONG ulColor );
FIXED  subQueryFontSize( ULONG ulQuery );
FIXED  subSetFontSize( FIXED fxPointSize );
PSZ    subQueryFontName( PSZ pszFontName, ULONG ulLen );
VOID   subSetFontName( PCSZ pszFontName );
VOID   subInvalidate( VOID );
LONG   subQueryDelta( VOID );
VOID   subSetDelta( LONG lDelta );

#ifdef __cplusplus
}
#endif

#endif
