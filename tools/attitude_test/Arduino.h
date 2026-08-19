// テスト用の最小スタブ（実機の Arduino.h の代わり）。
//
// time_us_32() は attitude.cpp の IMU 途絶判定（imu_fresh）と
// サンプル周期推定に使われるため、テスト側から進められる必要がある。
// 固定値を返すスタブにすると常に「途絶」と判定され、GNSS 観測が
// 一切適用されないまま테스트が通ってしまう（実際に一度そうなった）。
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>

extern uint32_t g_test_us;                 // テストが進める仮想時刻 [µs]
static inline uint32_t time_us_32() { return g_test_us; }
static inline void test_set_us(uint32_t t) { g_test_us = t; }
static inline void test_advance_us(uint32_t d) { g_test_us += d; }
