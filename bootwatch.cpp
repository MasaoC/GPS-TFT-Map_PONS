// ============================================================
// File    : bootwatch.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 起動の見張りと、水晶の立ち上がり待ち。説明は bootwatch.h。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/06
// ============================================================
#include "bootwatch.h"
#include "settings.h"
#include "mysd.h"                       // log_sdf()（Core1）、enqueueTask()（Core0）
#include "pico/multicore.h"             // Core0 の割り込みが生きているかを FIFO で確かめる
#include "hardware/structs/watchdog.h"  // watchdog_hw->scratch（watchdog の再起動をまたいで残す）
#include "hardware/structs/powman.h"    // CHIP_RESET（SW2 = HAD_RUN_LOW、DOUBLE_TAP）
#include "hardware/structs/ticks.h"     // hardware/ticks.h はスケッチの include パスに無いので直接書く
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/vreg.h"

// ---- 再起動をまたいで残す記録 ----
// ★ SW2（RUN）のリセットでは SRAM も watchdog の SCRATCH も消える（POWMAN ごと初期化される）。
//   watchdog の再起動（pico-sdk の watchdog_enable()。PSM だけのリセット）でも SRAM は残らない。
//   残るのは次の 2 つなので、これを使う。
//   (1) watchdog の SCRATCH0/1: watchdog の再起動では残る。止まった段階を置く。
//   (2) POWMAN CHIP_RESET の DOUBLE_TAP（1 bit。RW）: SW2 では消えない（POR/BOR でだけ消える）。
//       「起動処理に入ったが "BOOT PONS" まで来ていない」の印に使う。ブート ROM がこの bit を
//       見るのは OTP の BOOT_FLAGS1.DOUBLE_TAP が有効なときだけ（既定は無効）なので、立てても
//       BOOTSEL には入らない。
//
//   SCRATCH0: [31:16] 印 0xB007、bit15 = この起動は "BOOT PONS" まで来ていない、
//             bit14 = Core1 の見張りが再起動した、[7:0] = 続けて再起動した回数
//   SCRATCH1: [7:0] Core0 の段階、[15:8] Core1 の段階、[31:16] Core0 の段階の時刻 [ms]（65535 で頭打ち）
//   （SCRATCH2〜7 はブート ROM が使う）
#define BW_WD_MAGIC        0xB0070000u
#define BW_WD_MAGIC_MASK   0xFFFF0000u
#define BW_WD_INPROGRESS   0x00008000u
#define BW_WD_BY_CORE1     0x00004000u
#define BW_WD_COUNT_MASK   0x000000FFu

// HardFault の記録（今回の起動の中だけ。Core1 の見張りが読んで log.txt に書く）
#define BW_FAULT_MAGIC 0xFA17C0DEu
struct BwFault {
  volatile uint32_t magic, core, pc, lr, xpsr, exc_return, cfsr, hfsr, mmfar, bfar, sfsr;
};
static BwFault s_fault[2];   // コアごと

// Core1 の見張り（bootwatch_report_core1）が読む Core0 の段階と時刻
static volatile uint8_t  s_core0_stage = BW_STAGE_NONE;
static volatile uint32_t s_core0_ms    = 0;

// 起動処理の最初（bootwatch_rt_start）で読んだ前回の様子。"BOOT PONS" の後でログに出す。
enum BwSw2State : uint8_t {
  BW_SW2_NONE = 0,     // 今回は SW2 のリセットではない
  BW_SW2_CLEAN,        // SW2。前回は最後まで来た、または起動処理より前で止まった
  BW_SW2_UNFINISHED,   // SW2。前回は起動処理に入ったが "BOOT PONS" まで来なかった
};
static BwSw2State s_sw2_state    = BW_SW2_NONE;
static uint32_t   s_reset_reason = 0;       // 今回の POWMAN CHIP_RESET
static bool       s_wd_prev      = false;   // watchdog で戻った起動で、前回が途中だった
static uint32_t   s_wd_prev0 = 0, s_wd_prev1 = 0;   // そのときの SCRATCH0/1
static uint32_t   s_wd_count     = 0;       // 今回の起動までに続けて再起動した回数

static inline uint32_t clamp_ms16(uint32_t ms) { return ms > 0xFFFFu ? 0xFFFFu : ms; }

// ---- Core0 / Core1 の段階 ----
// SCRATCH1 は両方のコアが書くので、hw_write_masked（XOR のエイリアス。自分の bit だけ変える）で書く。
static inline void put_core0_stage(BootWatchStage stage, uint32_t ms) {
  s_core0_ms    = ms;
  s_core0_stage = stage;
  hw_write_masked(&watchdog_hw->scratch[1], (uint32_t)stage | (clamp_ms16(ms) << 16), 0xFFFF00FFu);
}

void bootwatch_stage(BootWatchStage stage) {
  put_core0_stage(stage, millis());
}

void bootwatch_stage1(BootWatchStage stage) {
  hw_write_masked(&watchdog_hw->scratch[1], (uint32_t)stage << 8, 0x0000FF00u);
}

// main() より前の段階。タイマーがまだ動いていない（リセット中）ので millis() は呼ばない。
static inline void rt_stage(BootWatchStage stage) {
  put_core0_stage(stage, 0);
}

// ---- main() より前（pico-sdk の起動処理）----
// 起動処理（crt0 → runtime_init → main）は .preinit_array の関数を名前の順に呼ぶ
// （pico/runtime.h の PICO_RUNTIME_INIT_FUNC と同じ仕組み。00050 = ブート ROM の後始末、
// 00300 = memcpy などの準備、00500 = クロック、YYYYY 以降 = コアごと）。
// ★ PICO_RUNTIME_INIT_FUNC マクロは使わない。C++ では関数のアドレスを uintptr_t にする初期化が
//   「動的な初期化」（コンストラクタ）にされうる。そうなると .preinit_array の中身は呼ばれる時点で
//   0 のままで、0 番地を呼んで落ちる。関数ポインタ型なら静的に埋まる。
// ★ 00300 より前なので memcpy / memset は使わない（arduino-pico の __wrap_memcpy は準備前）。
//   .bss は crt0 が runtime_init より前に 0 で埋め終えているので、static 変数には書いてよい。
static void bootwatch_rt_start(void) {
  const bool by_wd = (watchdog_hw->reason & (WATCHDOG_REASON_TIMER_BITS | WATCHDOG_REASON_FORCE_BITS)) != 0;

  // (2) SW2 をまたぐ 1 bit
  // ★ watchdog のリセットはチップ全体のリセットではないので、CHIP_RESET の HAD_* は前回の
  //   チップリセット（SW2 など）のまま残る。watchdog で戻った起動は見ない（下の (1) が出す）。
  s_reset_reason = powman_hw->chip_reset;
  if (!by_wd && (s_reset_reason & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS)) {
    s_sw2_state = (s_reset_reason & POWMAN_CHIP_RESET_DOUBLE_TAP_BITS) ? BW_SW2_UNFINISHED : BW_SW2_CLEAN;
  }
  powman_hw->chip_reset = POWMAN_PASSWORD_BITS | POWMAN_CHIP_RESET_DOUBLE_TAP_BITS;   // 「起動処理に入った」

  // (1) watchdog で戻った起動か
  const uint32_t sc0 = watchdog_hw->scratch[0];
  const uint32_t sc1 = watchdog_hw->scratch[1];
  uint32_t count = 0;
  if (by_wd && (sc0 & BW_WD_MAGIC_MASK) == BW_WD_MAGIC && (sc0 & BW_WD_INPROGRESS)) {
    count      = (sc0 & BW_WD_COUNT_MASK) + 1;
    s_wd_prev  = true;
    s_wd_prev0 = sc0;
    s_wd_prev1 = sc1;
  }
  s_wd_count = count;
  watchdog_hw->scratch[1] = 0;
  watchdog_hw->scratch[0] = BW_WD_MAGIC | BW_WD_INPROGRESS | (count & BW_WD_COUNT_MASK);
  rt_stage(BW_STAGE_RT_START);

#if BOOTWATCH_WDT_MS && defined(RELEASE)
  // 続けて止まる故障で再起動をくり返さないよう、上限を超えたら掛けない（固まったままにする）。
  // ★ RELEASE でないビルドは掛けない。setup() がシリアルモニタの接続を while (!Serial) で待つので、
  //   掛けると BOOTWATCH_WDT_MS ごとに再起動してしまう。
  if (count < BOOTWATCH_MAX_RESTARTS) {
    // ★ この時点では tick の発生器がまだ動いていない（runtime_init_clocks の最後で動かす）。
    //   watchdog は tick で数えるので、先に動かす。clk_ref はまだリング発振器（約 12MHz、ばらつく）
    //   なので、ここでの時間はおおよそ。クロックの設定後に正しい値（12）で設定し直される。
    ticks_hw->ticks[TICK_WATCHDOG].cycles = 12u << TICKS_WATCHDOG_CYCLES_LSB;
    ticks_hw->ticks[TICK_WATCHDOG].ctrl   = TICKS_WATCHDOG_CTRL_ENABLE_BITS;
    watchdog_enable(BOOTWATCH_WDT_MS, true);
  }
#endif
}
static void bootwatch_rt_done(void) {
  rt_stage(BW_STAGE_RT_DONE);
}
static void (*const bw_preinit_start)(void)
    __attribute__((used, section(".preinit_array.00010"))) = bootwatch_rt_start;
static void (*const bw_preinit_done)(void)
    __attribute__((used, section(".preinit_array.99990"))) = bootwatch_rt_done;

// ---- クロックの立ち上げ（pico-sdk の runtime_init_clocks() の写し）----
// 元は pico-sdk/src/rp2_common/pico_runtime_init/runtime_init_clocks.c の弱い定義（arduino-pico
// 4.5.1 同梱）。元から変えたのは 3 点だけ: 待ちのある手順の前の rt_stage()、水晶の立ち上がり待ち
// （BOOTWATCH_XOSC_SETTLE_MS）、PLL のロック外れの印を消すこと。
// ★ arduino-pico を上げたら、元のファイルと見比べて写し直すこと。
// start_all_ticks() と tick_start()（hardware_ticks/ticks.c の RP2350 の側）を展開したもの。
static void bw_start_all_ticks(void) {
  uint32_t cycles = clock_get_hz(clk_ref) / MHZ;
  for (int i = 0; i < (int)TICK_COUNT; ++i) {
    ticks_hw->ticks[i].cycles = cycles << TICKS_WATCHDOG_CYCLES_LSB;
    ticks_hw->ticks[i].ctrl   = TICKS_WATCHDOG_CTRL_ENABLE_BITS;
  }
}

extern "C" void runtime_init_clocks(void) {
  if (running_on_fpga()) {
    for (uint i = 0; i < CLK_COUNT; i++) {
      clock_set_reported_hz((clock_handle_t)i, 48 * MHZ);   // C++ なので型を明示する
    }
    clock_set_reported_hz(clk_ref, 12 * MHZ);
  } else {
    clocks_hw->resus.ctrl = 0;

    rt_stage(BW_STAGE_CLK_XOSC);
    xosc_init();
#if BOOTWATCH_XOSC_SETTLE_MS
    // ★ 水晶の立ち上がり待ち（docs/hardware.md §2「起動時に固まる件」）。xosc_init() の STABLE は
    //   「決まった回数を数えた」印で、振幅が育ちきった保証ではない。すぐ PLL を起こして 150MHz に
    //   上げると落ちる基板がある。この時点の clk_sys はリング発振器（約 12MHz）なので、時間はおおよそ。
    busy_wait_at_least_cycles((uint32_t)BOOTWATCH_XOSC_SETTLE_MS * 12000u);
#endif

    rt_stage(BW_STAGE_CLK_SWITCH);
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1)
      tight_loop_contents();
    hw_clear_bits(&clocks_hw->clk[clk_ref].ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_ref].selected != 0x1)
      tight_loop_contents();

    rt_stage(BW_STAGE_CLK_PLL_SYS);
    pll_init(pll_sys, PLL_SYS_REFDIV, PLL_SYS_VCO_FREQ_HZ, PLL_SYS_POSTDIV1, PLL_SYS_POSTDIV2);
    rt_stage(BW_STAGE_CLK_PLL_USB);
    pll_init(pll_usb, PLL_USB_REFDIV, PLL_USB_VCO_FREQ_HZ, PLL_USB_POSTDIV1, PLL_USB_POSTDIV2);
    // ロックした後で一度でもロックが外れたら立つ印（LOCK_N_STICKY）を、ここで消しておく。
    // ロックするまでの間に立っているので、消さないと必ず 1 になる。"BOOT PONS" の後で見る。
    pll_sys_hw->intr = PLL_INTR_LOCK_N_STICKY_BITS;
    pll_usb_hw->intr = PLL_INTR_LOCK_N_STICKY_BITS;

    rt_stage(BW_STAGE_CLK_REF);
    clock_configure_undivided(clk_ref,
                              CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                              0,
                              XOSC_HZ);
#if SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST && defined(SYS_CLK_VREG_VOLTAGE_MIN)
    if (vreg_get_voltage() < SYS_CLK_VREG_VOLTAGE_MIN) {
      vreg_set_voltage(SYS_CLK_VREG_VOLTAGE_MIN);
      busy_wait_at_least_cycles((uint32_t)((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t)XOSC_HZ) / 1000000));
    }
#endif
    rt_stage(BW_STAGE_CLK_SYS);
    clock_configure_undivided(clk_sys,
                              CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                              CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                              SYS_CLK_HZ);
    rt_stage(BW_STAGE_CLK_USB);
    clock_configure_undivided(clk_usb,
                              0,
                              CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                              USB_CLK_HZ);
    rt_stage(BW_STAGE_CLK_ADC);
    clock_configure_undivided(clk_adc,
                              0,
                              CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                              USB_CLK_HZ);
    rt_stage(BW_STAGE_CLK_PERI);
    clock_configure_undivided(clk_peri,
                              0,
                              CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                              SYS_CLK_HZ);
#if HAS_HSTX
    rt_stage(BW_STAGE_CLK_HSTX);
    clock_configure_undivided(clk_hstx,
                              0,
                              CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS,
                              SYS_CLK_HZ);
#endif
  }
  rt_stage(BW_STAGE_CLK_TICKS);
  bw_start_all_ticks();
  rt_stage(BW_STAGE_RT_CLOCKS);
}

// main() が USB を起こす前に呼ぶ（arduino-pico の main.cpp にある弱い定義を置き換える）。
// C++ のコンストラクタと rp2040.begin() の後。この時点ではまだ Core1 は起動していない。
void initVariant() {
  bootwatch_stage(BW_STAGE_MAIN);
}

static const char* stage_name(uint8_t s) {
  switch (s) {
    case BW_STAGE_NONE:        return "none";
    case BW_STAGE_RT_START:    return "sdk init start";
    case BW_STAGE_CLK_XOSC:    return "waiting xosc stable";
    case BW_STAGE_CLK_SWITCH:  return "switching clk_sys/ref";
    case BW_STAGE_CLK_PLL_SYS: return "waiting pll_sys lock";
    case BW_STAGE_CLK_PLL_USB: return "waiting pll_usb lock";
    case BW_STAGE_CLK_REF:     return "clk_ref -> xosc";
    case BW_STAGE_CLK_SYS:     return "clk_sys -> pll_sys 150MHz";
    case BW_STAGE_CLK_USB:     return "clk_usb (cpu at 150MHz)";
    case BW_STAGE_CLK_ADC:     return "clk_adc";
    case BW_STAGE_CLK_PERI:    return "clk_peri";
    case BW_STAGE_CLK_HSTX:    return "clk_hstx";
    case BW_STAGE_CLK_TICKS:   return "starting ticks";
    case BW_STAGE_RT_CLOCKS:   return "clocks done";
    case BW_STAGE_RT_DONE:     return "sdk init done, c++ ctors";
    case BW_STAGE_MAIN:        return "main, before USB";
    case BW_STAGE_SETUP:       return "setup entered";
    case BW_STAGE_SERIAL:      return "Serial.begin done";
    case BW_STAGE_PINS:        return "pins done";
    case BW_STAGE_POWER:       return "power_setup done";
    case BW_STAGE_BACKLIGHT:   return "backlight_setup done";
    case BW_STAGE_TFT:         return "setup_tft done";
    case BW_STAGE1_SETUP1:     return "setup1 entered";
    case BW_STAGE1_LATCH:      return "latch read";
    case BW_STAGE1_DEST:       return "destinations done";
    case BW_STAGE1_SOUND:      return "setup_sound done";
    case BW_STAGE1_WAIT:       return "waiting core0";
    case BW_STAGE1_SD:         return "setup_sd running";
    case BW_STAGE1_SD_DONE:    return "setup_sd done";
    case BW_STAGE1_DONE:       return "setup1 done";
    default:                   return "?";
  }
}

// ---- HardFault の記録 ----
// ★ 既定の isr_hardfault（crt0.S の弱い定義）は bkpt で止まるだけで、何も残らない。
//   ここで置き換え、例外のスタックフレームから PC・LR と、フォールトの理由のレジスタを残す。
//   XIP（フラッシュ）の読み違いで落ちた場合も動くよう、関数は RAM に置く。
//   記録したら止まったままにする（Core1 の見張りが log.txt に書いてから再起動する。Core1 も
//   止まっていれば watchdog が再起動する。そのときは記録は残らない）。
extern "C" void __attribute__((used, noinline, section(".time_critical.bootwatch")))
bootwatch_fault_capture(uint32_t* frame, uint32_t exc_return) {
  // frame: r0 r1 r2 r3 r12 lr pc xpsr（Cortex-M33 の基本フレーム）
  BwFault& f = s_fault[get_core_num() & 1];
  f.core       = get_core_num();
  f.lr         = frame[5];
  f.pc         = frame[6];
  f.xpsr       = frame[7];
  f.exc_return = exc_return;
  f.cfsr  = *(volatile uint32_t*)0xE000ED28;   // MMFSR | BFSR | UFSR
  f.hfsr  = *(volatile uint32_t*)0xE000ED2C;
  f.mmfar = *(volatile uint32_t*)0xE000ED34;
  f.bfar  = *(volatile uint32_t*)0xE000ED38;
  f.sfsr  = *(volatile uint32_t*)0xE000EDE4;   // SecureFault（RP2350 は Secure で動いている）
  __dmb();
  f.magic = BW_FAULT_MAGIC;
  for (;;) {
    __asm volatile("nop");
  }
}

// EXC_RETURN の bit2 で、例外の直前に使っていたスタック（MSP / PSP）を選ぶ。
extern "C" void __attribute__((naked, used, section(".time_critical.bootwatch"))) isr_hardfault(void) {
  __asm volatile(
      "tst lr, #4                  \n"
      "bne 1f                      \n"
      "mrs r0, msp                 \n"
      "b 2f                        \n"
      "1:                          \n"
      "mrs r0, psp                 \n"
      "2:                          \n"
      "mov r1, lr                  \n"
      "b bootwatch_fault_capture   \n");
}

// ---- Core1 の見張り ----
static bool s_timed_out = false;   // tft_pio_ready の待ちが切れた

void bootwatch_wait_core0(volatile bool* ready, uint32_t wait_ms) {
  bootwatch_stage1(BW_STAGE1_WAIT);
  const uint32_t t = millis();
  while (!*ready && millis() - t < wait_ms) {
    delay(1);
  }
  if (!*ready) s_timed_out = true;
  bootwatch_stage1(BW_STAGE1_SD);   // 呼び出し側はこの後 setup_sd() に進む
}

// Core0 の割り込みが生きているか。arduino-pico は Core0 に FIFO 割り込み（RP2040Support.h の _irq）を
// 付けていて、届いた値を読み捨てる（_GOTOSLEEP の 0xC0DED02E 以外）。FIFO を満杯にして少し待ち、
// 空いていれば Core0 は割り込みを受け付けている。空かなければ、割り込みを止めたまま回っているか、
// HardFault で止まっている。
#define BW_PROBE_WORD 0x5A5A0B0Bu
static bool core0_irq_alive() {
  for (int i = 0; i < 16; i++) {
    if (!multicore_fifo_push_timeout_us(BW_PROBE_WORD, 0)) break;
  }
  delay(5);
  return multicore_fifo_wready();
}

void bootwatch_report_core1(volatile bool* ready) {
  bootwatch_stage1(BW_STAGE1_SD_DONE);
  if (!s_timed_out) {
    bootwatch_stage1(BW_STAGE1_DONE);
    return;
  }

  // 待ちが切れたあとも、Core0 が遅いだけかもしれない。段階が BOOTWATCH_STUCK_MS 動かないか、
  // 準備ができるまで待つ（上限 5 秒）。
  const uint32_t t0 = millis();
  for (;;) {
    if (*ready) {
      log_sdf("BOOT WATCH: core0 slow but alive (ready at %lums since boot, stage %u)",
              (unsigned long)millis(), (unsigned)s_core0_stage);
      bootwatch_stage1(BW_STAGE1_DONE);
      return;
    }
    if (millis() - s_core0_ms >= BOOTWATCH_STUCK_MS || millis() - t0 >= 5000) break;
    delay(10);
  }

  const uint8_t  stage   = s_core0_stage;
  const uint32_t stuck   = millis() - s_core0_ms;
  const bool     faulted = (s_fault[0].magic == BW_FAULT_MAGIC);
  const bool     irq     = core0_irq_alive();
  log_sdf("BOOT WATCH: core0 stuck at stage %u (%s) for %lums, irq=%s, fault=%s",
          (unsigned)stage, stage_name(stage), (unsigned long)stuck,
          irq ? "alive" : "dead", faulted ? "yes" : "none");
  if (faulted) {
    // 2 行に分けてある（log_sd() が 128 バイトで切っていたころの名残。いまは本文 256 バイトまで入る）。
    const BwFault& f = s_fault[0];
    log_sdf("BOOT WATCH fault: core=%lu pc=0x%08lx lr=0x%08lx xpsr=0x%08lx exc=0x%08lx",
            (unsigned long)f.core, (unsigned long)f.pc, (unsigned long)f.lr,
            (unsigned long)f.xpsr, (unsigned long)f.exc_return);
    log_sdf("BOOT WATCH fault: cfsr=0x%08lx hfsr=0x%08lx bfar=0x%08lx mmfar=0x%08lx sfsr=0x%08lx",
            (unsigned long)f.cfsr, (unsigned long)f.hfsr, (unsigned long)f.bfar,
            (unsigned long)f.mmfar, (unsigned long)f.sfsr);
  }

#if BOOTWATCH_AUTO_RESTART
  // 回数は次の起動の bootwatch_rt_start() が数える（watchdog の時間切れと同じ扱い）。
  if (s_wd_count >= BOOTWATCH_MAX_RESTARTS) {
    log_sdf("BOOT WATCH: not restarting (%lu restarts in a row already)", (unsigned long)s_wd_count);
    bootwatch_stage1(BW_STAGE1_DONE);
    return;
  }
  hw_set_bits(&watchdog_hw->scratch[0], BW_WD_BY_CORE1);
  log_sdf("BOOT WATCH: restarting (%lu/%u)", (unsigned long)(s_wd_count + 1), (unsigned)BOOTWATCH_MAX_RESTARTS);
  delay(20);          // log.txt は log_sd() の中で flush 済み。念のため少し待つ
  rp2040.reboot();    // watchdog による再起動（戻ってこない）
#else
  bootwatch_stage1(BW_STAGE1_DONE);
#endif
}

void bootwatch_boot_done_core0() {
#if BOOTWATCH_WDT_MS
  watchdog_disable();
#endif
  powman_hw->chip_reset = POWMAN_PASSWORD_BITS;   // DOUBLE_TAP を下ろす（この起動は最後まで来た）
  watchdog_hw->scratch[0] = BW_WD_MAGIC;          // 途中の印と回数を消す
  watchdog_hw->scratch[1] = 0;

  // 起動から今までに PLL のロックが一度でも外れたか（runtime_init_clocks でロック直後に消してある）。
  // 外れても CPU が生き残った起動だけがここに来る。立っていれば水晶か電源が不安定。
  const bool sys_lost = (pll_sys_hw->intr & PLL_INTR_LOCK_N_STICKY_BITS) != 0;
  const bool usb_lost = (pll_usb_hw->intr & PLL_INTR_LOCK_N_STICKY_BITS) != 0;
  if (sys_lost || usb_lost) {
    enqueueTask(createLogSdfTask("BOOT WATCH: PLL lost lock during boot (sys=%d usb=%d)", sys_lost, usb_lost));
  }

  if (s_sw2_state != BW_SW2_NONE) {
    enqueueTask(createLogSdfTask("BOOT WATCH: SW2 reset (chip_reset=0x%08lx): last boot %s",
                                 (unsigned long)s_reset_reason,
                                 s_sw2_state == BW_SW2_UNFINISHED ? "entered SDK init but did NOT finish"
                                                                  : "finished, or stopped BEFORE SDK init"));
  }
  if (s_wd_prev) {
    const uint32_t st0 = s_wd_prev1 & 0xFFu, st1 = (s_wd_prev1 >> 8) & 0xFFu, ms0 = s_wd_prev1 >> 16;
    enqueueTask(createLogSdfTask("BOOT WATCH: restarted by %s (%lu in a row)",
                                 (s_wd_prev0 & BW_WD_BY_CORE1) ? "core1 watch" : "WATCHDOG timeout",
                                 (unsigned long)s_wd_count));
    enqueueTask(createLogSdfTask("BOOT WATCH: last boot core0 stage %lu (%s) at %lums",
                                 (unsigned long)st0, stage_name((uint8_t)st0), (unsigned long)ms0));
    enqueueTask(createLogSdfTask("BOOT WATCH: last boot core1 stage %lu (%s)",
                                 (unsigned long)st1, stage_name((uint8_t)st1)));
  }
}
