// ============================================================
// File    : tools/repons/test/linktelem_vectors.cpp
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : 本体の lora_link/link_proto.h の LinkTelem に値を詰め、メモリのバイト列（16 進）と値を JS の形で書き出す。
//           Re:PONS の live.js の decodeLT() がこのバイト列から同じ値を読めるかを、jsc で確かめるため
//           （run_live_tests.py。設計書 docs/pons_live_stream_plan.md §5.2）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-26
// ============================================================
#include <cstdio>
#include <cstdint>
#include <cstring>
#include "../../../lora_link/link_proto.h"

static void dump(const LinkTelem& t) {
  const uint8_t* p = (const uint8_t*)&t;
  printf("  { hex: '");
  for (size_t i = 0; i < sizeof(t); i++) printf("%02x", p[i]);
  printf("', v: { ver: %u, seq: %u, group: %u, scale_index: %u, dest_id: %u, year: %u, month: %u, day: %u, hour: %u, minute: %u, "
         "second: %u, centi: %u, lat_1e7: %d, lon_1e7: %d, gs_cms: %u, track_cdeg: %u, gnss_alt_dm: %d, kf_alt_dm: %d, kf_vs_cms: %d, "
         "press_dpa: %u, roll_cdeg: %d, pitch_cdeg: %d, yaw_cdeg: %u, pitch_avg_cdeg: %d, roll_trim_cdeg: %d, yaw_acc95_deg: %u, "
         "wind_dmps: %u, wind_dir_cdeg: %u, tas_dmps: %u, tas_flags: %u, batt: %u, numsat: %u, hacc_dm: %u, sacc_cms: %u, "
         "fixflags: %u, dest_index: %d, nav_mode: %u, have: %u, status: %u } },\n",
         t.ver, t.seq, t.group, t.scale_index, t.dest_id, t.year, t.month, t.day, t.hour, t.minute, t.second, t.centi,
         t.lat_1e7, t.lon_1e7, t.gs_cms, t.track_cdeg, t.gnss_alt_dm, t.kf_alt_dm, t.kf_vs_cms, t.press_dpa,
         t.roll_cdeg, t.pitch_cdeg, t.yaw_cdeg, t.pitch_avg_cdeg, t.roll_trim_cdeg, t.yaw_acc95_deg,
         t.wind_dmps, t.wind_dir_cdeg, t.tas_dmps, t.tas_flags, t.batt, t.numsat, t.hacc_dm, t.sacc_cms,
         t.fixflags, t.dest_index, t.nav_mode, t.have, t.status);
}

int main() {
  printf("// tools/repons/test/linktelem_vectors.cpp が書いた（LINK_PROTO_VER %d・%u バイト）\n", LINK_PROTO_VER, (unsigned)sizeof(LinkTelem));
  printf("var LT_VECTORS = { proto: %d, size: %u, list: [\n", LINK_PROTO_VER, (unsigned)sizeof(LinkTelem));
  // 1) どの欄も違う値（並びのずれを見つけるため、隣どうしが同じにならないように）
  LinkTelem a; memset(&a, 0, sizeof(a));
  a.magic0 = 'P'; a.magic1 = 'L'; a.ver = LINK_PROTO_VER; a.type = 0;
  a.seq = 0xBEEF; a.group = 2; a.scale_index = 4; a.dest_id = 0x1234;
  a.year = 2026; a.month = 11; a.day = 23; a.hour = 21; a.minute = 59; a.second = 58; a.centi = 70;
  a.lat_1e7 = 352942220; a.lon_1e7 = 1362543330; a.gs_cms = 723; a.track_cdeg = 30512;
  a.gnss_alt_dm = -123; a.kf_alt_dm = 845; a.kf_vs_cms = -37; a.press_dpa = 10082;
  a.roll_cdeg = -455; a.pitch_cdeg = 312; a.yaw_cdeg = 34211; a.pitch_avg_cdeg = 341; a.roll_trim_cdeg = -12;
  a.yaw_acc95_deg = 7; a.wind_dmps = 18; a.wind_dir_cdeg = 25000; a.tas_dmps = 78; a.tas_flags = 3;
  a.batt = 102 + 87; a.numsat = 21; a.hacc_dm = 14; a.sacc_cms = 0xFFFF; a.fixflags = 1;
  a.dest_index = -1; a.nav_mode = 2; a.have = 0x7FFF; a.status = 0x1A85; a.crc16 = 0x55AA;
  dump(a);
  // 2) 端の値（符号の付いた欄の最小・最大、無しの印）
  LinkTelem b; memset(&b, 0, sizeof(b));
  b.ver = LINK_PROTO_VER; b.seq = 1; b.lat_1e7 = -900000000; b.lon_1e7 = 1800000000; b.gs_cms = 65535;
  b.track_cdeg = 35999; b.gnss_alt_dm = -32768; b.kf_alt_dm = 32767; b.kf_vs_cms = -32768; b.roll_cdeg = 32767;
  b.pitch_cdeg = -32768; b.yaw_cdeg = 35999; b.yaw_acc95_deg = 255; b.batt = 255; b.dest_index = 127; b.have = 0; b.status = 0xFFFF;
  dump(b);
  printf("] };\n");
  return 0;
}
