/*
    KMP : Timer library for OS/2
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
        KO Myung-Hun <komh@chollian.net> 2007/03/05
            - Check ERROR_INTERRUPT for DosWaitEventSem() in tmrThread()

        KO Myung-Hun <komh@chollian.net> 2007/03/06
            - Changed termination method for tmrThread()

        KO Myung-Hun <komh@chollian.net> 2007/04/24
            - Use PRTYD_MAXIMUM instead of +31 for DosSetPriority()
*/

#define INCL_DOS
#define INCL_DOSERRORS
#include <os2.h>

#include "kmp_thread.h"

#include "kmp_timer.h"

static HEV    m_hev;
static Thread m_tid;
static BOOL   m_fQuit;

static void ( *m_callback )( void *arg );
static void *m_callback_arg;

static void tmrThread( void *arg )
{
    ULONG      ulPost;

    for(;;)
    {
        while( DosWaitEventSem( m_hev, SEM_INDEFINITE_WAIT ) == ERROR_INTERRUPT );
        DosResetEventSem( m_hev, &ulPost );

        if( m_fQuit )
            break;

        m_callback( m_callback_arg );
    }
}

VOID tmrInit( VOID )
{
    m_fQuit = FALSE;

    m_callback = NULL;
    m_callback_arg = NULL;

    DosCreateEventSem( NULL, &m_hev, DC_SEM_SHARED, FALSE );

    m_tid = CreateThread( tmrThread, NULL );
}

VOID tmrDone( VOID )
{
    m_fQuit = TRUE;

    DosPostEventSem( m_hev );

    WaitThread( m_tid );

    DosCloseEventSem( m_hev );
}

VOID tmrAsyncCall( ULONG ms, void ( *callback )( void * ), void *arg )
{
    PTIB   ptib;
    ULONG  ulClass;
    ULONG  ulDelta;
    HTIMER hTimer;

    m_callback = callback;
    m_callback_arg = arg;

    DosGetInfoBlocks( &ptib, NULL );
    ulClass = HIBYTE( ptib->tib_ptib2->tib2_ulpri );
    ulDelta = LOBYTE( ptib->tib_ptib2->tib2_ulpri );

    DosSetPriority( PRTYS_THREAD, PRTYC_TIMECRITICAL, PRTYD_MAXIMUM, 0 );
    DosAsyncTimer( ms, ( HSEM )m_hev, &hTimer );
    DosSetPriority( PRTYS_THREAD, ulClass, ulDelta, 0 );
}

VOID tmrDelay( ULONG ms )
{
    PTIB  ptib;
    ULONG ulClass;
    ULONG ulDelta;

    DosGetInfoBlocks( &ptib, NULL );
    ulClass = HIBYTE( ptib->tib_ptib2->tib2_ulpri );
    ulDelta = LOBYTE( ptib->tib_ptib2->tib2_ulpri );

    DosSetPriority( PRTYS_THREAD, PRTYC_TIMECRITICAL, PRTYD_MAXIMUM, 0 );
    DosSleep( ms );
    DosSetPriority( PRTYS_THREAD, ulClass, ulDelta, 0 );
}

