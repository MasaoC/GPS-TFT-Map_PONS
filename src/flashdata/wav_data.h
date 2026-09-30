// ============================================================
// File    : wav_data.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 本体 FLASH に内蔵した音声（8bit unsigned PCM / 16kHz / mono）の**宣言**。
//           実体は wav_data.cpp（tools/gen_wav_flash.py の生成物）。
//
//           0.982 で音声を SD から FLASH へ移した。SD が無くても・壊れていても
//           警報の音声が鳴るようにするため。SD からの WAV 再生は廃止した。
//
// ■ なぜ .h に実体を置かないのか
//   名前空間スコープの const は内部リンケージなので、.h に実体を書くと
//   include した .cpp ごとに複製される（インクルードガードでは止まらないし、
//   リンクエラーにもならないので気づけない）。logo_data.h と同じ理由。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/01
// ============================================================
#ifndef WAV_DATA_H
#define WAV_DATA_H
#include <Arduino.h>

// 音声 1 本ぶんの索引。
// name は **呼び出し側が書いている文字列リテラルと同じ**（例 "wav/opening.wav"）。
// SD 時代のパスをそのまま鍵に使うことで、createPlayWavTask() の呼び出しを
// 一切書き換えずに供給元だけ FLASH へ移せる。
struct WavEntry {
  const char* name;  // 音声名（＝呼び出し側の文字列）
  uint32_t    off;   // WAV_BLOB 内のオフセット [byte]
  uint32_t    len;   // 音声データ長 [byte]（= サンプル数。16kHz なので /16000 で秒）
};

// 生成時のサンプルレート [Hz]。
// ★ マクロにしてあるのは、sound.cpp が static_assert で自分の sampleRate と
//   突き合わせられるようにするため。extern const では別の翻訳単位の値なので
//   コンパイル時に比べられず、**取り違えたまま黙って通る**。
#define WAV_SAMPLE_RATE 16000

extern const uint8_t  WAV_BLOB[];      // 全音声を連結したもの
extern const uint32_t WAV_BLOB_SIZE;   // WAV_BLOB の総バイト数（詰め物を含む）
extern const WavEntry WAV_ENTRIES[];   // name の昇順に並んでいる
extern const uint16_t WAV_COUNT;       // WAV_ENTRIES の要素数

#endif // WAV_DATA_H
