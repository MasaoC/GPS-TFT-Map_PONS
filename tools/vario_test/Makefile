# imu.cpp（3状態バリオ KF・加速度ガード・MS5611 復帰 reinit・供給元選択）を PC 上で検証する。
CXXFLAGS = -std=c++17 -O2 -Wall -I. -I../..
DEPS = ../../imu.cpp ../../imu.h ../../attitude.cpp ../../attitude.h ../../settings.h

all: test
	@echo "--- バリオ KF・加速度ガード・MS5611 途絶復帰テスト ---" && ./test

test: main.cpp $(DEPS)
	g++ $(CXXFLAGS) -o test main.cpp ../../imu.cpp ../../attitude.cpp

clean:
	rm -f test
.PHONY: all clean
