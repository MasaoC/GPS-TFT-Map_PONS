# attitude.cpp の数学と異常系を PC 上で検証する。変更したら必ず実行すること。
CXXFLAGS = -std=c++17 -O2 -I. -I../..
COMMON = testclock.cpp ../../attitude.cpp
DEPS = ../../attitude.cpp ../../attitude.h ../../settings.h

all: test decl_test dead_test
	@echo "--- 協調旋回（旋回中のバンク復元）---" && ./test
	@echo "" && echo "--- 偏角補正（磁北→真北の符号）---" && ./decl_test
	@echo "" && echo "--- BNO085 断線・未接続（安全側に倒れるか）---" && ./dead_test

test: main.cpp $(DEPS)
	g++ $(CXXFLAGS) -o test main.cpp $(COMMON)
decl_test: decl_test.cpp $(DEPS)
	g++ $(CXXFLAGS) -o decl_test decl_test.cpp $(COMMON)
dead_test: dead_test.cpp $(DEPS)
	g++ $(CXXFLAGS) -o dead_test dead_test.cpp $(COMMON)

clean:
	rm -f test decl_test dead_test
.PHONY: all clean
