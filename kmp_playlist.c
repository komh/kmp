/*
    KMP : Playlist library for OS/2
    Copyright (C) 2008 by KO Myung-Hun <komh@chollian.net>

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
        KO Myung-Hun <komh@chollian.net> 2008/10/23
            - Fixed compare() to check only name part without extension
*/

#define INCL_DOS
#define INCL_DOSERRORS
#include <os2.h>

#undef __STRICT_ANSI__  // for stricmp()
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "kmp_playlist.h"

PPLAYLIST plCreate( void )
{
    PPLAYLIST ppl;

    ppl = malloc( sizeof( PLAYLIST ));
    if( ppl )
    {
        ppl->count = 0;
        ppl->excludeExts = NULL;
        ppl->ppliStart = NULL;
        ppl->ppliEnd = NULL;
        ppl->ppliCurrent = NULL;
    }

    return ppl;
}

void plDestroy( PPLAYLIST ppl )
{
    PPLAYLISTITEM ppli, ppliNext;

    if( !ppl )
        return;

    for( ppli = ppl->ppliStart; ppli; ppli = ppliNext )
    {
        ppliNext = ppli->ppliNext;

        free( ppli->filename );
        free( ppli );
    }

    free( ppl->excludeExts );
    free( ppl );
}

void plAddFile( PPLAYLIST ppl, const char *filename, int fSort )
{
    PPLAYLISTITEM ppliNew;

    if( !ppl || !filename || !*filename )
        return;

    ppliNew = malloc( sizeof( PLAYLISTITEM ));
    if( !ppliNew )
        return;

    ppliNew->filename = strdup( filename );
    if( !ppliNew->filename )
    {
        free( ppliNew );

        return;
    }

    ppliNew->index = ppl->count;
    ppliNew->ppliPrev = NULL;
    ppliNew->ppliNext = NULL;

    if( ppl->count == 0 )
    {
        ppl->ppliStart = ppliNew;
        ppl->ppliEnd = ppliNew;
        ppl->ppliCurrent = ppliNew;
    }
    else if( !fSort )
    {
        ppl->ppliEnd->ppliNext = ppliNew;
        ppliNew->ppliPrev = ppl->ppliEnd;

        ppl->ppliEnd = ppliNew;
    }
    else
    {
        PPLAYLISTITEM ppli = ppl->ppliCurrent;

        while( ppli && stricmp( ppliNew->filename, ppli->filename ) < 0 )
            ppli = ppli->ppliPrev;

        if( !ppli )
        {
            ppl->ppliStart->ppliPrev = ppliNew;
            ppliNew->ppliNext = ppl->ppliStart;

            ppl->ppliStart = ppliNew;
        }
        else
        {
            while( ppli && stricmp( ppliNew->filename, ppli->filename ) > 0 )
                ppli = ppli->ppliNext;

            if( !ppli )
            {
                ppl->ppliEnd->ppliNext = ppliNew;
                ppliNew->ppliPrev = ppl->ppliEnd;

                ppl->ppliEnd = ppliNew;
            }
            else
            {
                ppliNew->ppliNext = ppli;
                ppliNew->ppliPrev = ppli->ppliPrev;
                ppli->ppliPrev->ppliNext = ppliNew;
                ppli->ppliPrev = ppliNew;
            }
        }

        ppl->ppliStart->index = 0;
        ppli = ppl->ppliStart->ppliNext;
        while( ppli )
        {
            ppli->index = ppli->ppliPrev->index + 1;
            ppli = ppli->ppliNext;
        }
    }

    ppl->count++;
}

static int isExcludeExt( const char *excludeExts, const char *filename )
{
    const char *excludeExt = excludeExts;
    const char *ext;

    if( !excludeExt )
        return 0;

    ext = strrchr( filename, '.');
    if( !ext )
        ext = filename + strlen( filename );
    else
        ext++;

    while( *excludeExt )
    {
        if( stricmp( excludeExt, ext ) == 0 )
            return 1;

        excludeExt = excludeExt + strlen( excludeExt ) + 1;
    }

    return 0;
}

static int compare( const char *name1, const char *name2, int tolerance )
{
    const char *lastDot1, *lastDot2;
    int error = 0;

    lastDot1 = strrchr( name1, '.');
    if( !lastDot1 )
        lastDot1 = name1 + strlen( name1 );

    lastDot2 = strrchr( name2, '.');
    if( !lastDot2 )
        lastDot2 = name2 + strlen( name2 );

    if( lastDot1 - name1 != lastDot2 - name2 )
        return 1;

    while( name1 < lastDot1 )
    {
        if( toupper( *name1 ) != toupper( *name2 ))
            error++;

        name1++;
        name2++;
    }

    return error > tolerance;
}

void plAddFileAuto( PPLAYLIST ppl, int tolerance )
{
    CHAR            szFileSpec[ CCHMAXPATHCOMP ];
    CHAR            szDir[ CCHMAXPATHCOMP ];
    CHAR            szFileName[ CCHMAXPATHCOMP ];
    char *          pch;
    HDIR            hdir = HDIR_CREATE;
    FILEFINDBUF3    findBuf3;
    ULONG           ulFindCount = 1;
    ULONG           rc;

    if( !ppl || ppl->count != 1 )
        return;

    strcpy( szFileSpec, ppl->ppliCurrent->filename );
    for( pch = szFileSpec; *pch; pch++ )
    {
        if( *pch == '/' )
            *pch = '\\';
    }

    pch = strrchr( szFileSpec, '\\');
    if( !pch )
        pch = strrchr( szFileSpec, ':');
    if( !pch )
        pch = szFileSpec;
    else
        pch++;
    strcpy( szFileName, pch );
    *pch = 0;
    strcpy( szDir, szFileSpec );
    strcpy( pch, "*.*");

    rc = DosFindFirst( szFileSpec, &hdir, FILE_NORMAL,
                       &findBuf3, sizeof( FILEFINDBUF3 ),
                       &ulFindCount, FIL_STANDARD );

    while( !rc )
    {
        if( !( findBuf3.attrFile & FILE_DIRECTORY ))
        {
            /* Skip the current file */
            if( stricmp( szFileName, findBuf3.achName ) != 0 )
            {
                if( !isExcludeExt( ppl->excludeExts, findBuf3.achName ) &&
                    compare( szFileName, findBuf3.achName, tolerance ) == 0 )
                {
                    CHAR szFullPath[ CCHMAXPATHCOMP ];

                    strcpy( szFullPath, szDir );
                    strcat( szFullPath, findBuf3.achName );
                    plAddFile( ppl, szFullPath, TRUE );
                }
            }
        }

        ulFindCount = 1;
        rc = DosFindNext( hdir, &findBuf3, sizeof( FILEFINDBUF3 ), &ulFindCount );
    }

    DosFindClose( hdir );
}

void plSetExcludeExts( PPLAYLIST ppl, const char *exts )
{
    if( !ppl )
        return;

    free( ppl->excludeExts );
    // additional 1 and 1 is needed for '\0\0' termination
    ppl->excludeExts = calloc( 1, strlen( exts ) + 1 + 1 );
    if( ppl->excludeExts )
    {
        char *pos;

        strcpy( ppl->excludeExts, exts );

        pos = strchr( ppl->excludeExts, ',');
        while( pos )
        {
            *pos = 0;

            pos = strchr( pos + 1, ',');
        }
    }
}

PPLAYLISTITEM plMovePrev( PPLAYLIST ppl )
{
    if( !ppl )
        return NULL;

    if( ppl->ppliCurrent && ppl->ppliCurrent != ppl->ppliStart )
        ppl->ppliCurrent = ppl->ppliCurrent->ppliPrev;
    else
        return NULL;

    return ppl->ppliCurrent;
}

PPLAYLISTITEM plMoveNext( PPLAYLIST ppl )
{
    if( !ppl )
        return NULL;

    if( ppl->ppliCurrent && ppl->ppliCurrent != ppl->ppliEnd )
        ppl->ppliCurrent = ppl->ppliCurrent->ppliNext;
    else
        return NULL;

    return ppl->ppliCurrent;
}


