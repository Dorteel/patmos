#MAIN?=test/motor_test
#MAIN?=test/baroTest
#MAIN?=blinking
MAIN?=test/imu_test
SERIAL?=/dev/ttyUSB0
I2C?=../i2c-master/i2c_master
BARO?=baro/baro
# This is the trivial Hello World example, distributed over two .c files

LDFLAGS?= \
        -mpatmos-method-cache-size=0x1000 \
        -mpatmos-stack-base=0x080000 -mpatmos-shadow-stack-base=0x078000 \
        -Xgold --defsym -Xgold __heap_end=0x070000

all:
# Original
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(BARO).c  $(MAIN).c -o de10-nano.elf -lm
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(BARO).c  $(MAIN).c -o ~/t-crest/patmos/tmp/de10-nano.elf -lm
download:
	patserdow -v $(SERIAL) de10-nano.elf

