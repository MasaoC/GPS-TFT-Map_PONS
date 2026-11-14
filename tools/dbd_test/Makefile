# gnss_dbd.cpp（MGA-DBD 検証・吸い出し・復元状態遷移・時刻注入）を PC 上で検証する。
CXXFLAGS = -std=c++17 -O2 -Wall -I. -I../..
DEPS = ../../gnss_dbd.cpp ../../gnss_dbd.h ../../settings.h

all: test
	@echo "--- GNSS DBD フレーム検証・復元状態遷移・時刻注入テスト ---" && ./test

test: main.cpp $(DEPS)
	g++ $(CXXFLAGS) -o test main.cpp ../../gnss_dbd.cpp

clean:
	rm -f test
.PHONY: all clean
