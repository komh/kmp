extproc sh

./configure --prefix=/usr \
            --enable-gpl \
            --enable-postproc \
            --enable-os2threads \
            --disable-pthreads \
            --enable-runtime-cpudetect \
            --extra-cflags="-D_GNU_SOURCE -DCONFIG_FASTMEMCPY=1" \
            $@
