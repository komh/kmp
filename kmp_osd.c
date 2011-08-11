/*
    KMP : OSD library for OS/2
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
        KO Myung-Hun <komh@chollian.net> 2007/03/25
            - Use CONCAT.x as osd bitmap width not ( CONCAT.x - BOTTOMLEFT.x )

        KO Myung-Hun <komh@chollian.net> 2007/05/23
            - Reworked using Text BitMap library

        KO Myung-Hun <komh@chollain.net> 2008/05/03
            - Use TBMA_TOP
*/

#define INCL_WIN
#define INCL_GPI
#include <os2.h>

#include <stdlib.h>
#include <string.h>

#include "kmp_tbm.h"
#include "kmp_osd.h"

static BOOL         m_fErase;
static RECTL        m_rclInvalidOSD;
static PTEXTBITMAP  m_ptbm;

APIRET osdInit( HAB hab, PCSZ pcszFontNameSize, BOOL fUseImg )
{
    m_fErase = FALSE;

    m_ptbm = tbmInit( hab, pcszFontNameSize, fUseImg );

    return 0;
}

VOID osdDone( VOID )
{
    tbmDone( m_ptbm );
}

VOID osdDisplay( HPS hps, PBYTE pbImage, LONG lImgStride, ULONG ulImgHeight,
                 LONG x, LONG y, ULONG ulMaxWidth, ULONG ulMaxHeight,
                 ULONG ulColor, ULONG ulColorOutline, ULONG ulColorShadow,
                 BOOL f3d )
{
    if( !m_ptbm->ptlStart )
        return;

    if( !m_ptbm->hbm )
        tbmCreate( m_ptbm, x, y, ulMaxWidth, ulMaxHeight, ulColor, ulColorOutline, ulColorShadow, f3d, TBMA_TOP );

    tbmDraw( m_ptbm, hps, pbImage, lImgStride, ulImgHeight );
}

VOID osdErase( HPS hps, ULONG ulColor )
{
    if( m_fErase )
    {
        GpiCreateLogColorTable( hps, 0, LCOLF_RGB, 0, 0, NULL );
        WinFillRect( hps, &m_rclInvalidOSD, ulColor );

        m_fErase = FALSE;
    }
}

FIXED osdQueryFontSize( VOID )
{
    return m_ptbm->fxPointSize;
}

FIXED osdSetFontSize( FIXED fxPointSize )
{
    m_ptbm->fxPointSize = fxPointSize;

    if( FIXEDINT( m_ptbm->fxPointSize ) < 6 )
        m_ptbm->fxPointSize = MAKEFIXED( 6, 0 );
    if( FIXEDINT( m_ptbm->fxPointSize ) > 144 )
        m_ptbm->fxPointSize = MAKEFIXED( 144, 0 );

    osdInvalidate();

    return m_ptbm->fxPointSize;
}

PSZ osdQueryFontName( PSZ pszFontName, ULONG ulLen )
{
    strncpy( pszFontName, m_ptbm->szFaceName, ulLen );
    pszFontName[ ulLen - 1 ] = 0;

    return pszFontName;
}

VOID osdSetFontName( PCSZ pcszFontName )
{
    strncpy( m_ptbm->szFaceName, pcszFontName, sizeof( m_ptbm->szFaceName ));
    m_ptbm->szFaceName[ sizeof( m_ptbm->szFaceName ) - 1 ] = 0;

    osdInvalidate();
}

VOID osdInvalidate( VOID )
{
    tbmDelete( m_ptbm );

    m_rclInvalidOSD = m_ptbm->rclCurrent;
    m_fErase = TRUE;

    tbmClearText( m_ptbm );
}

VOID osdSetText( PCSZ pcszText )
{
    osdInvalidate();

    tbmAddText( m_ptbm, pcszText );
}

BOOL osdOn( VOID )
{
    return ( BOOL )m_ptbm->ptlStart;
}
