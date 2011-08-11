/*
    KMP : Subtitle library for OS/2
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
        KO Myung-Hun <komh@chollian.net> 2007/02/20
            - Use BITMAPINFOHEADER2 instead of BITMAPINFOHEADER to avoid
              warning in sbmCreate()

        KO Myung-Hun <komh@chollian.net> 2007/02/22
            - Added findFont(), subQueryNextFontSize() and
              subQueryPrevFontSize()

        KO Myung-Hun <komh@chollian.net> 2007/02/25
            - Added subQueryDelta() and subSetDelta()
            - Supports subtitle sync correction

        KO Myung-Hun <komh@chollian.net> 2007/03/25
            - Fixed subtitle bitmap was truncated on right edge

        KO Myung-Hun <komh@chollian.net> 2007/05/01
            - Fixed subtitle doesn't disappear even though it is out of
              its time range when seeking

        KO Myung-Hun <komh@chollian.net> 2007/05/23
            - Reworked using Text BitMap library
            - Merged SubQueryNextFontSize() and SubQueryPrevFontSize() into
              SubQueryFontSize()

        KO Myung-Hun <komh@chollian.net> 2008/07/27
            - Modified the path of header to the relative path to the source
              root

        KO Myung-Hun <komh@chollian.net> 2009/04/26
            - Fixed the problem which recgonizes blank subtitle as the last
              subtitle.
*/

#define INCL_WIN
#define INCL_GPI
#include <os2.h>

#include <stdlib.h>
#include <string.h>

#include "libsub/subreader.h"

#include "kmp_tbm.h"
#include "kmp_sub.h"

static sub_data *   m_sd;
static subtitle *   m_cur_sub;
static int          m_cur_sub_index;
static BOOL         m_fErase;
static BOOL         m_fDisplayed;
static RECTL        m_rclInvalidSub;
static LONG         m_lDelta;
static PTEXTBITMAP  m_ptbm;

APIRET subInit( HAB hab, PCSZ pcszFileName, PCSZ pcszFontNameSize, float fps, BOOL fUseImg )
{
    char **subfilenames;

    m_sd = NULL;
    m_cur_sub = NULL;
    m_fErase = FALSE;
    m_fDisplayed = FALSE;
    m_lDelta = 0;
    m_ptbm = NULL;

    /* find subtitle file */
    subfilenames = sub_filenames( "", ( char * )pcszFileName );

    /* found subtitle file */
    if( *subfilenames )
    {
        char *name = strdup( *subfilenames );

        /* read subtitle file, use only first subtitle */
        m_sd = sub_read_file( name, fps );
        if( m_sd )
        {
            int i;

            m_cur_sub = m_sd->subtitles;

            m_ptbm = tbmInit( hab, pcszFontNameSize, fUseImg );

            for( i = 0; i < m_cur_sub->lines; i++ )
                tbmAddText( m_ptbm, m_cur_sub->text[ i ] );

        }
        else
            free( name );
    }

    /* free subtitle filename list */
    if( subfilenames )
    {
        char **temp = subfilenames;

        while( *temp )
            free( *temp++ );

        free( subfilenames );
    }

    return ( m_sd == NULL );
}

VOID subDone( VOID )
{
    tbmDone( m_ptbm );

    sub_free( m_sd );
}

BOOL subInTime( ULONG ulTime )
{
    return (( LONG )ulTime >= (( LONG )m_cur_sub->start + m_lDelta ) &&
            ( LONG )ulTime <= (( LONG )m_cur_sub->end + m_lDelta ));
}

VOID subFindNext( ULONG ulTime )
{
    subtitle *temp_sub = m_cur_sub;

    while( temp_sub != m_sd->subtitles &&
           ( LONG )ulTime < (( LONG )temp_sub->start + m_lDelta ))
        temp_sub--;

    while( temp_sub != m_sd->subtitles + m_sd->sub_num - 1 &&
           ( LONG )ulTime > (( LONG )temp_sub->end + m_lDelta ))
        temp_sub++;

    if( m_cur_sub != temp_sub )
    {
        int i;

        m_cur_sub = temp_sub;

        tbmClearText( m_ptbm );

        for( i = 0; i < m_cur_sub->lines; i++ )
            tbmAddText( m_ptbm, m_cur_sub->text[ i ] );
    }

    if( m_fDisplayed )
    {
        subInvalidate();

        m_fDisplayed = FALSE;
    }
}

VOID subDisplay( HPS hps, PBYTE pbImage, LONG lImgStride, ULONG ulImgHeight,
                 LONG x, LONG y, ULONG ulMaxWidth, ULONG ulMaxHeight,
                 ULONG ulColor, ULONG ulColorOutline, ULONG ulColorShadow,
                 BOOL f3d )
{
    if( !m_ptbm->ptlStart )
        return;

    if( !m_ptbm->hbm )
        tbmCreate( m_ptbm, x, y, ulMaxWidth, ulMaxHeight, ulColor, ulColorOutline, ulColorShadow, f3d, TBMA_CENTER );

    tbmDraw( m_ptbm, hps, pbImage, lImgStride, ulImgHeight );

    m_fDisplayed = TRUE;
}

VOID subErase( HPS hps, ULONG ulColor )
{
    if( m_fErase )
    {
        GpiCreateLogColorTable( hps, 0, LCOLF_RGB, 0, 0, NULL );
        WinFillRect( hps, &m_rclInvalidSub, ulColor );

        m_fErase = FALSE;
    }
}

#define NO_NEXT_POINT_SIZE  MAKEFIXED( 1000, 0 )
#define NO_PREV_POINT_SIZE  MAKEFIXED( 0, 0 )

FIXED subQueryFontSize( ULONG ulQuery )
{
    if( ulQuery == SQFS_NEXT || ulQuery == SQFS_PREV )
    {
        LONG         cFonts = 0;
        PFONTMETRICS pfm;
        FIXED        fxPointSize = ( ulQuery == SQFS_NEXT ) ? NO_NEXT_POINT_SIZE : NO_PREV_POINT_SIZE;
        FIXED        fxNominalPointSize;
        int          i;

        cFonts = GpiQueryFonts( m_ptbm->hps, QF_PUBLIC | QF_PRIVATE, m_ptbm->szFaceName, &cFonts,
                                sizeof( FONTMETRICS ), NULL );

        pfm = malloc( sizeof( FONTMETRICS ) * cFonts );

        GpiQueryFonts( m_ptbm->hps, QF_PUBLIC | QF_PRIVATE, m_ptbm->szFaceName, &cFonts,
                    sizeof( FONTMETRICS ), pfm );

        for( i = 0; i < cFonts; i++ )
        {
            if( pfm[ i ].fsDefn & FM_DEFN_OUTLINE ) // outline font
            {
                fxPointSize = m_ptbm->fxPointSize;
                if( ulQuery == SQFS_NEXT )
                    fxPointSize += MAKEFIXED( 1, 0 );
                else
                    fxPointSize -= MAKEFIXED( 1, 0 );

                break;
            }

            if(( pfm[ i ].sXDeviceRes == m_ptbm->lHoriFontRes ) &&
            ( pfm[ i ].sYDeviceRes == m_ptbm->lVertFontRes ))
            {
                fxNominalPointSize = MAKEFIXED( pfm[ i ].sNominalPointSize / 10, 0 );
                if( ulQuery == SQFS_NEXT )
                {
                    if(( fxNominalPointSize > m_ptbm->fxPointSize ) &&
                    ( fxNominalPointSize < fxPointSize ))
                        fxPointSize = fxNominalPointSize;
                }
                else
                {
                    if(( fxNominalPointSize < m_ptbm->fxPointSize ) &&
                    ( fxNominalPointSize > fxPointSize ))
                        fxPointSize = fxNominalPointSize;
                }
            }
        }


        if( fxPointSize == NO_NEXT_POINT_SIZE || fxPointSize == NO_PREV_POINT_SIZE )
            fxPointSize = m_ptbm->fxPointSize;

        free( pfm );

        return fxPointSize;
    }

    return m_ptbm->fxPointSize;
}

FIXED subSetFontSize( FIXED fxPointSize )
{
    m_ptbm->fxPointSize = fxPointSize;

    subInvalidate();

    return m_ptbm->fxPointSize;
}

PSZ subQueryFontName( PSZ pszFontName, ULONG ulLen )
{
    strncpy( pszFontName, m_ptbm->szFaceName, ulLen );
    pszFontName[ ulLen - 1 ] = 0;

    return pszFontName;
}

VOID subSetFontName( PCSZ pszFontName )
{
    strncpy( m_ptbm->szFaceName, pszFontName, sizeof( m_ptbm->szFaceName ));
    m_ptbm->szFaceName[ sizeof( m_ptbm->szFaceName ) - 1 ] = 0;

    subInvalidate();
}

VOID subInvalidate( VOID )
{
    tbmDelete( m_ptbm );

    m_rclInvalidSub = m_ptbm->rclCurrent;
    m_fErase = TRUE;
}

LONG subQueryDelta( VOID )
{
    return m_lDelta;
}

VOID subSetDelta( LONG lDelta )
{
    m_lDelta = lDelta;

    subInvalidate();
}


