/*
    KMP : Thread library for OS/2
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
*/

#ifndef __KMP_THREAD_H__
#define __KMP_THREAD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <os2.h>

typedef HMTX Mutex;
typedef HEV  Cond;
typedef TID  Thread;

Mutex  CreateMutex( VOID );
void   DestroyMutex( Mutex mutex );
void   LockMutex( Mutex mutex );
void   UnlockMutex( Mutex mutex );
Cond   CreateCond( VOID );
void   DestroyCond( Cond cond );
void   CondSignal( Cond cond );
void   CondWait( Cond cond, Mutex mutex );
Thread CreateThread( void ( *fn )( void * ), void *data );
void   WaitThread( Thread tid );

#ifdef __cplusplus
}
#endif

#endif

