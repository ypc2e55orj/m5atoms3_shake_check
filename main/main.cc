// C++
#include <array>
#include <cmath>
#include <limits>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include <M5Unified.h>

#include "fft.h"

// FFTサンプル数
constexpr size_t kNumFftSamples = 1024;

struct Context {
  std::array<float, kNumFftSamples> samples;
  std::array<float, kNumFftSamples> db;

  TaskHandle_t task_update_sensor;
  TaskHandle_t task_calculate_fft;
  TaskHandle_t task_draw_display;
};
Context *ctx;

// センサ更新タスク
[[noreturn]] void updateSensorTask(void *) {
  // 収集したサンプルの数
  size_t num_samples = 0;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

    // 値を更新
    M5.Imu.update();

    // 値をセット
    float dummy;
    M5.Imu.getAccel(&dummy, &dummy, &dummy);
    M5.Imu.getGyro(&dummy, &dummy, &ctx->samples[num_samples]);

    if (++num_samples >= kNumFftSamples) {
      num_samples = 0;
      // FFT計算タスクに通知
      xTaskNotifyGive(ctx->task_calculate_fft);

      // 完了通知が来るまで待つ
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    }
  }
}

// FFT計算タスク
[[noreturn]] void calculateFftTask(void *) {
  fft_config_t *real_fft_plain = fft_init(kNumFftSamples, FFT_REAL, FFT_FORWARD, nullptr, nullptr);

  while (true) {
    // センサ取得完了通知が来るまで待つ
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    for (auto n = 0; n < real_fft_plain->size; n++) {
      real_fft_plain->input[n] = ctx->samples[n];
    }
    fft_execute(real_fft_plain);

    // 0にDC成分, 1にスペクトル中心?
    ctx->db[0] = real_fft_plain->output[0];
    ctx->db[1] = real_fft_plain->output[1];
    for (auto n = 1; n < real_fft_plain->size / 2; n++) {
      float real = real_fft_plain->output[2 * n] * real_fft_plain->output[2 * n];
      float imag = real_fft_plain->output[2 * n + 1] * real_fft_plain->output[2 * n + 1];
      ctx->db[n] = std::sqrt(real + imag);
    }

    // 計算完了・再開通知
    xTaskNotifyGive(ctx->task_update_sensor);
  }
  // fft_destroy(real_fft_plain);
}

// 描画タスク
[[noreturn]] void drawDisplayTask(void *) {
  M5Canvas canvas;

  // ディスプレイを初期化
  M5.Display.fillScreen(TFT_BLACK);

  // Canvasにディスプレイサイズを設定
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    // 描画データ準備
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    for (auto n = 2; n < kNumFftSamples; n++) {
      auto &s = ctx->db[n];
      min = std::min(min, s);
      max = std::max(max, s);
    }
    printf("min : %f\n", min);
    printf("max : %f\n", max);

    // 描画開始
    M5.Display.startWrite();
    canvas.pushSprite(&M5.Display, 0, 0);
    M5.Display.endWrite();
  }
}

extern "C" void app_main() {
  M5.begin();
  ctx = new Context;
  xTaskCreatePinnedToCore(updateSensorTask, "updateSensor", 8192, nullptr, 10, &ctx->task_update_sensor, 0);
  xTaskCreatePinnedToCore(calculateFftTask, "calculateFft", 8192, nullptr, 10, &ctx->task_calculate_fft, 1);
  xTaskCreatePinnedToCore(drawDisplayTask, "drawDisplayTask", 8192, nullptr, 5, &ctx->task_draw_display, 1);
}
