#===== TEST
#MAIN?=test/TestCode-BaroMeter
MAIN?=test/TestCode-IMU
#MAIN?=test/TestCode-Receiver
#MAIN?=test/TestCode-BaroMeter
#MAIN?=test/TestCode-Compass
#MAIN?=test/compass_test
# MAIN?=test/imu_test


#===== MAIN
#MAIN?=Flight_controller_v2

#===== SUPPLEMENT 
SOURCES ?= basic_lib/i2c_master.c test/imu/imu.c gps/gps.c test/baro/baro.c
# SOURCES ?= basic_lib/i2c_master.c gps/gps.c
SERIAL?=/dev/ttyUSB0

# This is the trivial Hello World example, distributed over two .c files

LDFLAGS?= \
        -mpatmos-method-cache-size=0x1000 \
        -mpatmos-stack-base=0x080000 -mpatmos-shadow-stack-base=0x078000 \
        -Xgold --defsym -Xgold __heap_end=0x070000

all:
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(SOURCES) $(MAIN).c -o de10-nano.elf -lm
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(SOURCES) $(MAIN).c -o ~/t-crest/patmos/tmp/de10-nano.elf -lm

# 	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(GPS).c $(MAIN).c -o de10-nano.elf -lm
# 	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(GPS).c $(MAIN).c -o ~/t-crest/patmos/tmp/de10-nano.elf -lm
#	IMU	
# 	patmos-clang -I ../.. -O2 $(LDFLAGS) $(IMU).c $(I2C).c $(GPS).c $(MAIN).c -o de10-nano.elf -lm
# 	patmos-clang -I ../.. -O2 $(LDFLAGS) $(IMU).c $(I2C).c $(GPS).c $(MAIN).c -o ~/t-crest/patmos/tmp/de10-nano.elf -lm
# 	BARO
# 	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(BARO).c $(MAIN).c -o de10-nano.elf -lm
# 	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(BARO).c $(MAIN).c -o ~/t-crest/patmos/tmp/de10-nano.elf -lm
download:
	patserdow -v $(SERIAL) de10-nano.elf

