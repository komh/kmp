/*
    KMP : Thread library for OS/2
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
        KO Myung-Hun <komh@chollian.net> 2008/08/24
            - Undefine __STRICT_ANSI__ to avoid 'implicit declaration'
              of _beginthread() and _endthread() with -std=c99 option.
*/

#define INCL_DOS
#define INCL_DOSERRORS
#include <os2.h>

#undef __STRICT_ANSI__  // for _beginthread() and _endthread()
#include <stdlib.h>

#include "kmp_thread.h"

Mutex CreateMutex( VOID )
{
    HMTX hmtx;

    DosCreateMutexSem( NULL, &hmtx, 0, FALSE );

    return ( Mutex )hmtx;
}

void DestroyMutex( Mutex mutex )
{
    DosCloseMutexSem(( HMTX )mutex );
}

ULONG LockMutex( Mutex mutex )
{
    return DosRequestMutexSem(( HMTX )mutex, SEM_INDEFINITE_WAIT );
}

ULONG UnlockMutex( Mutex mutex )
{
    return DosReleaseMutexSem(( HMTX )mutex );
}

Cond CreateCond( VOID )
{
    HEV hev;

    DosCreateEventSem( NULL, &hev, 0, FALSE );

    return ( Cond )hev;
}

void DestroyCond( Cond cond )
{
    DosCloseEventSem(( HEV )cond);
}

void CondSignal( Cond cond )
{
    DosPostEventSem(( HEV )cond );
}

void CondWait( Cond cond, Mutex mutex )
{
    ULONG ulPost;

    UnlockMutex( mutex );

    DosWaitEventSem( cond, SEM_INDEFINITE_WAIT );
    DosResetEventSem( cond, &ulPost );

    LockMutex( mutex );
}

Thread CreateThread( int ( *fn )( void * ), void *data )
{
    return _beginthread( fn, NULL, 256 * 1024, data );
}

void WaitThread( Thread tid )
{
    while( DosWaitThread( &tid, DCWW_WAIT ) == ERROR_INTERRUPT );
}

