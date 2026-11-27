// TAS の経路角補正を公開 API 経由で検証する。
// V/S は平均ピッチと同期した 10 秒窓で評価し、無効時は TAS を欠測とする。
#include "attitude.h"
#include <cmath>
#include <cstdio>

static const double G = 9.80665;

static void body_to_sensor(const double b[3], float out[3]) {
    out[0] = (float)-b[2];
    out[1] = (float)b[1];
    out[2] = (float)b[0];
}

static void feed_sample(float vs, bool valid, float pitch_deg, uint32_t &t_us) {
    const int hz = IMU_RATE_GYRO_HZ;
    const double dt = 1.0 / (double)hz;
    const double pitch = (double)pitch_deg * M_PI / 180.0;
    // 静止した機体姿勢を重力方向で合成する。機首上げは正のピッチ。
    const double body_a[3] = {G * sin(pitch), 0.0, G * cos(pitch)};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3]; body_to_sensor(body_a, accel);
    t_us += (uint32_t)(dt * 1e6);
    test_set_us(t_us);
    attitude_on_accel(accel);
    attitude_set_vspeed(vs, valid);
    attitude_on_gyro(gyro, t_us);
}

static bool feed_seconds(float vs, bool valid, float pitch_deg, int seconds, uint32_t &t_us) {
    const int hz = IMU_RATE_GYRO_HZ;
    for (int n = 0; n < seconds * hz; n++) feed_sample(vs, valid, pitch_deg, t_us);
    return true;
}

static bool test_wind_after_dropout() {
    attitude_setup();
    uint32_t t = 0;
    const auto feed_wind = [&](bool valid, float gs, int seconds) {
        for (int i = 0; i < seconds * IMU_RATE_GYRO_HZ; ++i) {
            feed_sample(0.0f, valid, 0.0f, t);
            if (i % (IMU_RATE_GYRO_HZ / 5) == 0) attitude_on_gnss_velocity(gs, 0.0f, 0.0f, 0.08f);
        }
    };
    float wind, direction, tas;
    bool clamped;
    feed_wind(true, 8.0f, 35);
    const bool before = attitude_get_wind(wind, direction);
    feed_wind(false, 10.0f, 25);
    const bool lost = !attitude_get_wind(wind, direction) && !attitude_get_tas(tas, clamped);
    feed_wind(true, 10.0f, 12);
    const bool refilling = !attitude_get_wind(wind, direction) && attitude_get_tas(tas, clamped);
    feed_wind(true, 10.0f, 12);
    const bool recovered = attitude_get_wind(wind, direction);
    printf("Wind before/lost/refilling/recovered: %d/%d/%d/%d\n", before, lost, refilling, recovered);
    return before && lost && refilling && recovered;
}

int main() {
    bool ok = true;
    attitude_setup();
    uint32_t t_us = 0;
    feed_seconds(0.0f, true, 0.0f, 5, t_us);
    float partial = 0.0f; bool partial_clamped = false;
    const bool hidden_before_full = !attitude_pitch_avg_valid() &&
                                    !attitude_get_tas(partial, partial_clamped);
    feed_seconds(0.0f, true, 0.0f, 7, t_us);
    float base = 0.0f, corrected = 0.0f;
    bool clamped = false;
    float alpha = 0.0f;
    const bool have_base = attitude_get_tas(base, clamped);
    bool gamma_event = false;
    const bool has_event = attitude_take_gamma_event(gamma_event);
    ok &= hidden_before_full && have_base && attitude_pitch_avg_valid() && has_event && gamma_event;

    // 迎角一定で上昇・降下すると、機首ピッチには ±γ が混ざる。補正後は水平時の TAS に戻る。
    const float gamma20 = atan2f(0.20f, base) * 180.0f / (float)M_PI;
    feed_seconds(0.20f, true, gamma20, 20, t_us); // 姿勢と 10 秒窓の両方を新しい値へ落ち着かせる
    const bool have_up = attitude_get_tas(corrected, clamped, &alpha);
    const float alpha_up = alpha;
    const float up = corrected;
    const float pitch_up = attitude_get_pitch_avg_deg();
    const bool climb_correct = have_up && fabsf(up - base) < 0.10f;
    feed_seconds(-0.20f, true, -gamma20, 20, t_us);
    const bool have_down = attitude_get_tas(corrected, clamped, &alpha);
    const float alpha_down = alpha;
    const float down = corrected;
    const float pitch_down = attitude_get_pitch_avg_deg();
    const bool descent_correct = have_down && fabsf(down - base) < 0.10f;
    const bool alpha_ok = fabsf(alpha_up) < 0.05f && fabsf(alpha_down) < 0.05f &&
                          fabsf(down - attitude_airspeed_from_pitch(alpha_down)) < 0.001f;
    ok &= alpha_ok;

    // V/S の供給が切れた瞬間に補正を止める。無効区間は窓に残り、再取得後も窓が埋まるまで補正しない。
    attitude_set_vspeed(0.0f, false);
    bool unavailable_event = true;
    const bool got_unavailable_event = attitude_take_gamma_event(unavailable_event);
    const bool immediate_unavailable = !attitude_get_tas(corrected, clamped) &&
                                       !attitude_get_alpha_avg(alpha) && attitude_pitch_avg_valid();
    feed_sample(0.0f, false, attitude_get_pitch_avg_deg(), t_us); // 0.1 秒窓が閉じる前の短い欠測
    attitude_set_vspeed(0.20f, true);
    const bool brief_dropout_unavailable = !attitude_get_tas(corrected, clamped);
    attitude_set_vspeed(0.0f, false);
    feed_seconds(0.0f, false, 0.0f, 12, t_us);
    const bool invalid_window_unavailable = !attitude_get_tas(corrected, clamped);
    feed_seconds(0.20f, true, gamma20, 5, t_us);
    const bool still_waiting_for_full_window = !attitude_get_tas(corrected, clamped);
    feed_seconds(0.20f, true, gamma20, 7, t_us);
    const bool recovered = attitude_get_tas(corrected, clamped) && fabsf(corrected - base) < 0.10f;
    bool available_event = false;
    const bool got_available_event = attitude_take_gamma_event(available_event);
    const bool unavailable = immediate_unavailable && brief_dropout_unavailable &&
                             invalid_window_unavailable && still_waiting_for_full_window;
    ok &= unavailable;
    ok &= got_unavailable_event && !unavailable_event && got_available_event && available_event;

    ok &= climb_correct && descent_correct && recovered;
    // 設計総重量の基準曲線を保ち、実重量は γ の計算にも反映する。
    const float design_kg = AIRSPEED_DESIGN_WEIGHT_KG;
    const float design_curve = attitude_airspeed_from_pitch(2.0f);
    const float light_kg = 88.0f;
    attitude_set_airspeed_weight_kg(light_kg);
    const bool light_ok = fabsf(attitude_airspeed_from_pitch(2.0f) -
                                design_curve * sqrtf(light_kg / design_kg)) < 0.001f;
    const float heavy_kg = 100.0f;
    attitude_set_airspeed_weight_kg(heavy_kg);
    const float heavy_speed = base * sqrtf(heavy_kg / design_kg);
    const float heavy_gamma = atan2f(0.20f, heavy_speed) * 180.0f / (float)M_PI;
    feed_seconds(0.20f, true, heavy_gamma, 20, t_us);
    const bool heavy_ok = attitude_get_tas(corrected, clamped) && fabsf(corrected - heavy_speed) < 0.01f;
    attitude_set_airspeed_weight_kg(0.0f);
    const bool invalid_rejected = attitude_get_airspeed_weight_kg() == heavy_kg;
    const bool limit_ok = attitude_airspeed_from_pitch(-6.0f, &clamped) == AIRSPEED_MAX_MPS && clamped;
    const float original_max = attitude_get_airspeed_max();
    attitude_set_airspeed_max(6.9f);
    const bool nan_clamp = attitude_airspeed_from_pitch(NAN, &clamped) == 6.9f && clamped;
    attitude_set_airspeed_max(original_max);
    attitude_set_airspeed_weight_kg(design_kg);
    const bool baseline_restored = attitude_airspeed_from_pitch(2.0f) == design_curve;
    ok &= light_ok && heavy_ok && invalid_rejected && limit_ok && nan_clamp && baseline_restored;
    printf("Weight correction: light/heavy+gamma/invalid/limit/restore %d/%d/%d/%d/%d\n",
           (int)light_ok, (int)heavy_ok, (int)invalid_rejected, (int)limit_ok, (int)baseline_restored);
    printf("TAS gamma: level %.3f, climb %.3f, descent %.3f; alpha up/down %+.3f/%+.3f; unavailable/recovery %d/%d\n",
           base, up, down, alpha_up, alpha_down, (int)unavailable, (int)recovered);
    printf("window before full: %d; pitch climb/descent %.3f/%.3f deg (expected +/- %.3f); within 0.10 m/s: %d/%d\n",
           (int)hidden_before_full, pitch_up, pitch_down, gamma20, (int)climb_correct, (int)descent_correct);
    printf("initial event %d/%d; unavailable/recovered events %d/%d %d/%d; immediate/brief/window/wait unavailable %d/%d/%d/%d\n",
           (int)has_event, (int)gamma_event, (int)got_unavailable_event, (int)unavailable_event,
           (int)got_available_event, (int)available_event,
           (int)immediate_unavailable, (int)brief_dropout_unavailable, (int)invalid_window_unavailable,
           (int)still_waiting_for_full_window);
    ok &= test_wind_after_dropout();
    printf("結果: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
