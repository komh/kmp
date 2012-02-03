/*
    KMP : Image library for OS/2
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
        KO Myung-Hun <komh@chollian.net> 2007/02/25
            - Added prototypes for imgQueryAttr(), imgSetAttr() and
              imgResetAttr()
            - Changed imgInit() to initialize attribute

        KO Myung-Hun <komh@chollian.net> 2007/03/03
            - Removed the following variables in YUV struct
                int     w;
                int     h;

            - Changed the prototype of imgCreateYUV()

        KO Myung-Hun <komh@chollian.net> 2007/12/30
            - Modified to imgInit() return type to check error
*/

#ifndef __KMP_IMG_H__
#define __KMP_IMG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <os2.h>

#include <types.h>

#include "kva.h"

typedef struct YUV
{
    uint8_t *data[4];
    int     linesize[4];
    uint8_t *p;
} YUV;

extern int sws_flags;

APIRET imgInit( ULONG kvaMode, HWND hwnd, ULONG ulKeyColor, LONG cx, LONG cy,
                ULONG ulRatio, PULONG pulAttrValue );
VOID   imgDone( VOID );
YUV  * imgCreateYUV( VOID );
VOID   imgFreeYUV( YUV *p );
VOID   imgDisplayYUV( YUV *bmp );
VOID   imgClearRect( PRECTL prcl );
VOID   imgSetAspectRatio( ULONG ulRatio );
APIRET imgQueryAttr( ULONG ulAttr, PULONG pulValue );
APIRET imgSetAttr( ULONG ulAttr, PULONG pulValue );
VOID   imgResetAttr( VOID );

#ifdef __cplusplus
}
#endif

#endif

