                             K Movie Player v0.7.2
                            -----------------------
                            
1. 소개
-------
  
  이 프로그램은 FFplay 를 바탕으로, 다양한 미디어 파일들을 OS/2 와 
eComStation 에서 재생하기 위해 만들어졌습니다.

2. 제작 환경
------------
  
  - 한국어판 OS/2 Warp v4 with FixPak #15
  
  - GCC v4.4.6
  
  - OS/2 ToolKit v4.5
  
  - OS2UNIX 환경 ( GNU Make v3.81r3, cp, rm, ... )

  - lxlite v1.3.3 ( 디버깅 정보 제거 )
  
  - libkva v1.2.0
  
  - libkai v1.1.1

  - git v1.7.3.2 ( 선택, 최신 FFmpeg 소스 받기 )
      
3. 실행에 필요한 환경
---------------------

  - 오디오 : UNIAUD, DART

  - 비디오 : SNAP, WarpOverlay!, VMAN, DIVE
  
  - RunTime : kLIBC v0.6.4

4. 시험 환경
------------

  한국어판 OS/2 Warp v4 with FixPak #15 와 eComStation 1.2MR 에서 
시험했습니다.

5. 설치
-------

  kmp.exe 를 아무 디렉토리에 넣어 두시면 됩니다. SNAP 오버레이를 쓰기 위해서는 
snapwarp.dll 이 필요합니다. kmp.exe 와 같은 곳 또는 LIBPATH 에 지정되어 있는 
곳에 넣어두시기 바랍니다.
  
6. 사용법 및 선택사항
---------------------

6.1 사용법
----------

6.1.1 실행하기
--------------
    kmp [선택사항] [@설정파일] 입력파일

6.1.2 실행 중에
---------------
q                   끝내기
f, Enter            전체화면/창화면 전환
p, SPC              잠시 멈춤/재개 전환
m                   오디오 켬/끔 전환
a                   오디오 채널 순환
v                   비디오 채널 순환
t                   자막 채널 순환
w                   오디오 파형을 보여줌
left/right          10초 앞뒤 이동
Ctrl+left/right     1분 앞뒤 이동
down/up             볼륨 1% 씩 낮추기/높이기
Ctrl+down/up        볼륨 5% 씩 낮추기/높이기
mouse click         화면비만큼 위치 이동 
+/-                 글꼴 크기 크게/작게
Alt-0/1/2/3         각각 화면비를 none/original/force43/force169 으로 바꾸기
8/9                 밝기 5 단계씩 어둡게/밝게 ( vman/dive 제외 )
i/o                 대비 5 단계씩 낮추기/높이기 ( vman/dive 제외 )
k/l                 채도 5 단계씩 낮추기/높이기 ( vman/dive 제외 )
,/.                 색상 5 단계씩 낮추기/높이기 ( vman/dive 제외 )
/                   밝기/대비/채도/색상 기본값으로 ( vman/dive 제외 )
[/]                 텍스트 파일 자막 싱크 0.5 초씩 빨리/느리게
PageUp/PageDown     이전/다음 파일 재생
;/'                 -/+ 0.1 초 단위로 오디오 싱크 수정(실험중)
d                   재생 파일 정보를 OSD 로 보여줌

6.2 선택사항
------------

6.2.1 주 선택사항
-----------------
-video driver       비디오 드라이버 설정 (driver=auto/snap/wo/vman/dive)
-audio driver       오디오 드라이버 설정 (driver=auto/uniaud/dart)
-vol level          처음 볼륨값을 퍼센트 단위로 지정
-subfont xx.name    자막 글꼴과 크기 지정 (xx=크기, name=글꼴 이름)
-osdfont xx.name    OSD 글꼴과 크기 지정 (xx=크기, name=글꼴 이름)
-aspect ratio       화면비율 지정 (ratio=none, original, force43, force169)
-hidemouse          전체화면에서 마우스 포인터를 감춤
-brightness level   밝기 지정, vman/dive 제외(level=0..255)
-contrast level     대비 지정, vman/dive 제외(level=0..255)
-saturation level   채도 지정, vman/dive 제외(level=0..255)
-hue level          색상 지정, vman/dive 제외(level=0..255)
-res48              48KHz 오디오를 44.1KHz 오디오로 변환(실험중)
-subimg             자막/OSD 를 이미지에 직접 출력
-subcolor f,o,s     자막색을 16진수 RGB 형태로 지정(f=글꼴, o=윤곽, s=그림자) *
-osdcolor f,o,s     OSD색을 16진수 RGB 형태로 지정(f=글꼴, o=윤곽, s=그림자) *
-autoadd            비슷한 이름의 파일을 자동으로 추가
-xext e1,e2,...     -autoadd 를 쓸 때, 해당 확장자를 가진 파일은 추가하지 않음

* 기본값은 f=FFFFFF, o=0F0F0F, s=0F0F0F

6.2.2 고급 선택사항
-------------------
-fixt23             이미지 축소 기능이 없는 비디오 카드(S3)의 문제 해결
-(no)audioshare     오디오를 (비)공유 모드로 엶. 기본은 공유모드
-fixsnap            snap 모드일 때, 동영상이 화면 크기보다 크면 자동으로 축소함
-audiobufsize size  오디오 버퍼의 크기를 바이트 단위로 지정
-seekinterval time  밀리초 단위로 탐색 시간 간격 설정

6.2.3 KMPOPT 환경변수
---------------------
  자주 쓰는 선택사항들은 KMPOPT 환경 변수에 지정해서 쓸 수 있습니다. 예로,
  
      SET KMPOPT=-vol 50
      KMP 비디오파일
   
  위 경우는 
     
      KMP -vol 50 비디오파일
      
  과 같습니다.

6.2.4. 설정 파일
----------------
  자주 쓰는 선택사항들은 설정 파일에 저장해서 쓸 수 있습니다. 설정 파일은 한 
줄에 하나의 인자만 있어야 합니다. 예로,
  
      ----- KMP.CFG start
      -vol
      50
      ----- KMP.CFG end
      
      KMP @KMP.CFG 비디오파일
  
  위 경우는
  
      KMP -vol 50 비디오파일
      
  과 같습니다.
     
7. 제한 사항 또는 알려진 벌레
-----------------------------

  - 일부 동영상 파일은 끝까지 재생이 안되거나, 자동종료가 안됩니다.

  - vman/dive 모드일 때, OSD 와 자막의 테두리와 그림자가 제대로 표시되지 
    않습니다. GpiBitBlt() 에 필요한 기능이 없습니다. -subimg 을 사용하십시오.
    
  - vman/dive 모드일 때, OSD 와 자막이 깜빡거립니다. -subimg 을 사용하십시오.

  - 자막 파일의 코드 페이지 변환을 지원하지 않습니다.
  
  - 지정한 이름과 크기를 가진 글꼴이 없을 경우, 시스템 기본 글꼴(WarpSans 
    Combined)이 사용됩니다. 하지만 이 경우에는 글꼴 크기를 조절할 수 없습니다.

  - 음파 모드에서 OSD 가 깜박 거립니다.
  
  - 잠시 멈춤 상태에서 갱신될 때 부하가 매우 큽니다.

  - 가끔, 동영상의 끝부분에서 이동 기능이 작동되지 않습니다.

  - Non-interleaved AVI 파일은 제대로 지원되지 않습니다.
  
  - UNIAUD 드라이버를 사용하는 경우, 특히 탐색을 할 때, 오디오 재생에 문제가 
    생길 수 있습니다. 다음 드라이버를 사용하시기 바랍니다.

     http://www.ecomstation.co.kr/komh/testcase/uniaud32_test.zip

  - 혜성 커서 기능을 켠 상태에서 vman/dive 모드로 재생을 하면, 마우스를 움직일 
    때마다 시스템이 일시적으로 멈춥니다. 둘 중에 하나만 사용하십시오.
    
8. 할 것들...
-------------

  - 슬라이더바 지원
  
9. 판번호 보기
--------------

  - v0.7.2 ( 2012/02/12 )
    .SDL 을 링크함. 고침( A. Doff 씨 제보 )
    .비디오 셋업이 실패하더라도 KMP 가 종료되지 않음. 고침
    
  - v0.7.1 ( 2012/02/11 )
    .BGR4 색공간이 쓰일 때, 빨간색과 파란색이 바뀜. 고침( pasha 씨 제보 )
    
  - v0.7.0 ( 2012/02/03 )
    .ffplay N-37502-gd1af5c2 소스 사용
    .VMAN 모드 지원( libkva v1.2.0 사용 )
    .vman/dive 모드에서 BGR4 색공간 지원( libkva v1.2.0 사용 )
    
  - v0.6.3 ( 2011/02/06 )
    .FFplay SVN-r26402 소스 사용
    .audio 모드일 때, -subimg 사용하면 프로그램 이상 종료. 고침
    .16bits dive 모드일 때 프로그램 이상 종료. 고침
    .uniaud, snap 조합일 때 프로그램 멈춤. 고침
    
  - v0.6.2 ( 2010/02/28 )
    .FFplay SVN-r21861 소스 사용
    .비디오 드라이버 설정이 잘못되어 비디오 재생이 안되면 오디오도 재생 안됨. 
     고침
    .재생정보( 파일이름, 재생순서, 자막여부 )를 OSD로 보여줌. 'd' 키 지원
    .LIBPATHSTRICT 호환성 개선( 실험중 )

  - v0.6.1 ( 2010/01/24 )
    .때때로 audio_thread() 가 끝나지 않음. 고침
    .탐색 시간이 오래 걸리면, 반복되는 소리가 남. 고침( libkai v1.0.1 사용)
    
  - v0.6.0 ( 2010/01/14 )
    .UNIAUD 지원( -audio, libkai v1.0.0 사용 )
    .오디오 싱크 조정 지원
    .탐색 시간 간격 설정 지원( -seekinterval 선택사항 )
    .재생하는 동안 Doodle's Screen Saver 끔( libkva v1.1.0 사용 )
    .DIVE 모드일 때 2x2 디더링 사용
    .자막 시간 정렬 기능 사용
    .연속 재생시 화면 속성 보존 안됨. 고침
    .자막파일이름이 ASCII 가 아니면, 잘못된 자막파일이 재생될 수 있음. 고침
    .빈자막을 마지막 자막으로 인식. 고침
    .닫기 버튼을 눌러도 다음 파일 재생. 고침
  
  - v0.5.1 ( 2008/10/23 )
    .YUVJ420P 동영상의 경우 swscaler 메세지가 지속적으로 출력됨. 고침
     ( PIX_FMT_YUVJ420P 를 PIX_FMT_YUV420P 로 다룸 )
    .15, 16 bpp 모드에서 RGB 로 색공간 변환에 이상이 있음. 고침
     ( BentN 씨 제보 )
    .확장자가 다르면 자동 추가되지 않음. 고침
    
  - v0.5.0 ( 2008/10/18 )
    .FFMpeg SVN-r15572 소스 사용
    .컴파일러를 GCC 4.3.2 로 변경( 스택 변수 정렬 경고 없어짐 )
    .여러 파일 재생하기 지원
    .비슷한 이름의 파일을 자동으로 추가하기 지원
     ( -autoadd, -xext 선택사항 )
    .snap 모드일 때, 동영상 크기가 화면 크기 이상이면 자동으로 이미지 축소 지원
     ( -fixsnap 선택사항 )
    .오디오 버퍼 크기 설정 지원( -audiobufsize 선택사항 )
    .설정 파일 지원
    .자동모드일 때, 하드웨어 가속 기능이 사용 중이면 DIVE 모드로 전환
    .SNAP 오버레이가 지원되지 않으면 처음 한 번만 실행되고 두 번째부터
     비정상적으로 종료. 고침.
    .때때로 비디오 재생시 SIGFPE 발생. 고침. 
     ( img_init() 을 video_thread() 에서 queue_picure() 로 옮김 )
    .-subimg 를 쓸 때 OSD 가 잘릴 경우, 비정상적 종료. 고침.
     ( OSD 아래 방향으로 출력 )
    .화면 비율 바꾸면 모든 화면 속성 초기화됨. 고침.
      
  - v0.4.1 ( 2007/01/24 )
    .FFMpeg SVN-r11604 소스 사용
    .쓰레드 기능 사용
    .-osdfont 선택사항 지원
    .AC3 오디오의 경우 최적화된 IMDCT 변환 지원
    .CPU 종류 및 SwScaler 플래그 표시
    .snap 모드일 때, YV12 동영상 재생 성능 향상
    .비디오 드라이버의 자동 선택 순서를 wo > snap > dive 에서 snap > wo > dive 
     로 바꿈.
    .snap 모드일 때, 속성 선택사항이 작동 안 함. 고침.
    .snap 모드일 때, 창을 최소화했다가 복원하면 속성이 사라짐. 고침.
    .snap 모드일 때, 16M 색상 모드가 아니면 동영상이 안 보임. 고침.
    .동영상이 YUV420P 포맷이 아니면 SwScaler 메세지가 계속 출력됨. 고침.
    .SSE 탐지 안됨. 고침.
    .비디오 재생이 안될 때 화면이 정리되지 않음. 고침.
    
  - v0.4.0 ( 2007/12/29 )
    .FFMpeg SVN-r11313 소스 사용
    .SNAP 오버레이 지원( Mike Forester 씨께 감사 )
    
  - v0.3.1 ( 2007/06/24 )
    .비디오 스트림이 없으면 음파 모드로 전환
    .오디오 독점 모드 지원
    .자막, OSD 색 지정 지원
    .비디오 스트림이 없으면 비정성적 종료. 고침( Franz Bakan 씨 제보 )
    .WO 모드일 때, 이미지 축소 기능이 없는 비디오 카드(S3)의 경우, 영상이 
     제대로 표시되지 않음. 고침( Franz Bakan 씨 제보 )
    .음악 파일을 재생할 때, 잠시 멈춤 상태에서도 화면이 지속적으로 갱신됨. 고침
    .-fs 선택사항 제대로 작동하지 않음. 고침
    .높이가 0 이 되면 비정상적 종료. 고침
    .DIVE 모드일 때, 15/16 비트 컬러 모드에서 색상이 다르게 나타남. 고침
    .15/16 비트 컬러 모드에서 -subimg 쓰면 자막이 깨끗하지 않음. 고침
    .15/16 비트 컬러 모드에서 글자 테두리 표시 안됨. -subcolor, -osdcolor 로 
     테두리와 그림자 색을 적당하게 지정하십시오.
     예를 들면 -subcolor ,0F0F0F,0F0F0F -osdcolor ,0F0F0F,0F0F0F
    .osdInvalidate() 에서 SIGSEGV 발생. 고침
    .tbDelete() 에서 SIGSEGV 발생. 고침
    .때때로 창크기가 동영상 크기와 상관없이 640x480 으로 설정됨. 고침
    
  - v0.3.0 ( 2007/05/27 )
    .FFmpeg 2007/05/24 소스 사용
    .자막/OSD 를 이미지에 직접 출력 지원( DIVE 모드일 때 유용 )
    .AAC 오디오 포맷 지원
    .48KHz 오디오를 44.1KHz 로 변환 지원( 실험중 )
    .libyuv 대신에 libswcale 사용
    .fastmemcpy() 사용
    .자막/OSD 비트맵 오른쪽이 잘림. 고침.
    .이동시 자막이 지워지지 않는 경우가 있음. 고침.
    
  - v0.2.2 ( 2007/03/14 )
    .부동 소숫점 연산이 때때로 실패함. 고침.
    .'/' 눌렀을 때 해당 OSD 출력 안 됨. 고침.
    .자동 종료 확인 방법 바꿈

  - v0.2.1 ( 2007/03/04 )
    .코덱의 이미지 포맷이 YUV420P 일 때, 성능 향상
    .잠시 멈춤 상태에서 화면 갱신 안됨. 고침.
    .음파 모드에서 OSD 출력 안됨. 고침.
    .wo 모드에서 윈도의 보이는 부분이 달라지면, 화면이 깜박 거림. 고침
    .v0.2.0 에서 '-hidemouse' 선택사항의 기여자가 Dmitry Froloff 씨였음을 
     빠뜨렸음. 고침.
    
  - v0.2.0 ( 2007/02/26 )
    .각종 정보를 TitleBar 에 표시하지 않고, OSD 로 표시
    .framedrop 기능 지원
    .전체화면일 때 마우스 포인터 감추기 기능 지원( by Dmitry Froloff )
    .밝기/대비/채도/색상 조절 기능 지원( wo 에서만 )
    .텍스트 파일 자막 싱크 조절 기능 지원
    .WarpCenter 가 있을 때 전체화면으로 바꾸면, WarpCenter 가 사라지지 않음. 
     고침( by Dmitry Froloff )
    .비트맵 글꼴 선택이 안됨. 고침
    
  - v0.1.0 ( 2007/02/12 )
    .재생이 끝나면 자동 종료
    .텍스트 파일 자막 지원( SAMI 파일만 시험했음 )
    .글꼴 크기 바꾸기 지원
    .화면 비율 바꾸기 지원
    .재생 정보를 타이틀바에 표시
    .이동 기능 개선
    .소스 파일 모듈화
    
  - 시험판 ( 2007/02/02 )
    .FFplay 에서 SDL 을 사용하지 않도록 하고, 명칭을 KMP 로 변경.
    .볼륨 조절 기능 추가
    .오디오 켬/끔 기능 추가
    .몇 가지 글쇠 기능 조정
    .잠시 멈춤 기능 개선(잠시 멈춤 상태로 오랜 시간이 지나면 싱크가 맞지 않음)
      
10. 소스 컴파일 하기
--------------------

  소스 파일들을 원하는 디렉토리에 풀고, C_INCLUDE_PATH, CPLUS_INCLUDE_PATH, 
LIBRARY_PATH 에 OS/2 ToolKit 4.5 관련 설정을 해주십시오.
  
  이제 configure.cmd 를 실행시킵니다.
  
  작업이 끝났으면 다음처럼 make 를 실행시킵니다.
  
      make SHELL=/bin/sh

11. 모듈 설명
-------------

  kmp.exe      : K Movie Player 실행 파일입니다.
  snapwrap.dll : SNAP 오버레이 지원을 위한 파일입니다.

12. 참조한 프로젝트
-------------------

  FFplay      : 기본 플레이어
  WarpVision  : libdart, libsub
  MPlayer     : libfaad, libvo
  
13. 고마운 분들...
------------------

  Dave Yeo : FFmpeg 에 OS/2 관련 패치 적용

14. 알아두기
------------

  KMP 는 Morphing 기법을 쓰기 때문에, 항상 텍스트 창이 열립니다. 그래서 파일 
연결 기능을 쓸 때, 항상 열리는 텍스트 창이 성가십니다. 이럴 때는 이렇게 
하시면 됩니다.

  일단, ToolKit 4.5 에 있는 MARKEXE.EXE 가 필요합니다. 준비가 되었으면 이렇게 
하십시오.

    copy kmp.exe kmppm.exe
    markexe windowapi kmppm.exe

  또는 markexe.exe 대신에 emxbind.exe 를 이용할 수도 있습니다.

    emxbind -e -p kmppm.exe
      
  이제 kmppm.exe 를 파일 연결에 사용하시면 됩니다. 이렇게 하면, 텍스트 창이 
열리지 않고, KMP 가 실행이 됩니다. 물론 도움말이라든지 기타 정보는 화면에 
표시되지 않습니다. 파일 연결 기능에만 쓰십시오. ^^

15. 후원하기
------------

  이 프로그램이 마음에 들어 후원하고 싶으시면, 아래 URL 을 방문해 주십시오.

    http://www.ecomstation.co.kr/komh/donate.html


16. 하고 싶은 말이 있을 때...
-----------------------------

  e-mail : komh@chollian.net
  ICQ    : 124861818
  MSN    : komh@chollian.net
  IRC    : lvzuufx, #os2 at HANIRC(irc.hanirc.org)
               
                                                       만든이 : 고명훈
               
