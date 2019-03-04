/Users/joshua/work/cmt/stm32/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-gdb _build/nrf52840_xxaa.out -ex 'target remote localhost:3333' -ex 'dump binary memory micbuf.bin mic_buf mic_buf+32000' -ex 'detach' -ex 'q'
sox -t s16 -L -c 1 -r 16000 micbuf.bin micbuf.wav
