# attitude.cpp の数学を PC 上で検証する

`attitude.cpp`（機上の姿勢 ESKF）を実機に焼かずに検証するためのハーネス。
協調旋回の合成データを流し、真のバンク角を復元できるかを見る。

`attitude.cpp` を変更したら必ず実行すること。移植ミス・符号ミスはここで捕まる。

```
cd tools/attitude_test
make            # または: g++ -std=c++17 -O2 -I. -I../.. -o test main.cpp ../../attitude.cpp
./test
```

期待される出力:

```
真のバンク角 : 20.00 deg
ESKF (sensor_roll) : 19.65 deg  (誤差 -0.35)
結果: PASS — C++ 版も真のバンクを復元した
```

## 仕組み

- `Arduino.h` / `settings.h` は実機ヘッダの最小スタブ。`attitude.cpp` を無改造でコンパイルするために置いてある。
  settings.h の定数は本物と同じ値にすること（変えたら比較が成立しない）。
- 比較は **マウント補正前の軸** で行う。`attitude_get_euler_raw()` は
  `roll = sensor_pitch` / `pitch = sensor_roll - 90` を返すので、
  シミュレーションの機体バンク（= sensor_roll）は `pitch + 90` で取り出す。
  ここを取り違えると 19.65 のかわりに 1.12 が出て FAIL に見える（実際に一度やった）。
- PC 側の同等品は `tools/imulog/test_eskf.py`。両者は同じ数式なので結果も一致するはず。
