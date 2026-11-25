# src/sd_wipe_rules.h（Re:PONS から SD のログを消すときの「どれを消してよいか」・パスワード・指紋・消す手順）を PC 上で検証する。
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I. -I../..
DEPS = ../../src/sd_wipe_rules.h

all: test
	@echo "--- SD のログの消去（消す名前の形・パスワード・指紋・16 件ずつ消す手順）---" && ./test

test: main.cpp $(DEPS)
	g++ $(CXXFLAGS) -o test main.cpp

clean:
	rm -f test
.PHONY: all clean
