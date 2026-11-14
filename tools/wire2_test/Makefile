# lora_link/link_wire2.h と link_beacon.h の符号化・復号・境界値を PC 上で検証する。
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I../.. -I../../lora_link
DEPS = ../../lora_link/link_wire2.h ../../lora_link/link_beacon.h ../../lora_link/link_proto.h

all: test
	@echo "--- Wire2 / Beacon フレーム往復・境界値テスト ---" && ./test

test: main.cpp $(DEPS)
	g++ $(CXXFLAGS) -o test main.cpp

clean:
	rm -f test
.PHONY: all clean
