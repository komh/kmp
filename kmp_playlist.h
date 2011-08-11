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
*/

#ifndef __KMP_PLAYLIST_H__
#define __KMP_PLAYLIST_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tagPLAYLISTITEM PLAYLISTITEM, *PPLAYLISTITEM;

struct tagPLAYLISTITEM
{
    const char *    filename;
    int             index;
    PLAYLISTITEM   *ppliPrev;
    PLAYLISTITEM   *ppliNext;
};

typedef struct tagPLAYLIST
{
    int             count;
    char *          excludeExts;
    PPLAYLISTITEM   ppliStart;
    PPLAYLISTITEM   ppliEnd;
    PPLAYLISTITEM   ppliCurrent;
} PLAYLIST, *PPLAYLIST;

PPLAYLIST       plCreate( void );
void            plDestroy( PPLAYLIST ppl );
void            plAddFile( PPLAYLIST ppl, const char *filename, int fSort );
void            plAddFileAuto( PPLAYLIST ppl, int tolerance );
void            plSetExcludeExts( PPLAYLIST ppl, const char *exts );
PPLAYLISTITEM   plMovePrev( PPLAYLIST ppl );
PPLAYLISTITEM   plMoveNext( PPLAYLIST ppl );

static inline const char *plQueryFileName( PPLAYLIST ppl )
{
    if( !ppl || !ppl->ppliCurrent )
        return NULL;

    return ppl->ppliCurrent->filename;
}

static inline int plQueryIndex( PPLAYLIST ppl )
{
    if( !ppl || !ppl->ppliCurrent )
        return 0;

    return ppl->ppliCurrent->index;
}

static inline int plQueryCount( PPLAYLIST ppl )
{
    if( !ppl )
        return 0;

    return ppl->count;
}

#ifdef __cplusplus
}
#endif

#endif

