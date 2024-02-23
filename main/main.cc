// C++
#include <array>
#include <cmath>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include <M5Unified.h>

#include "fft.h"

// FFTサンプル数
constexpr size_t kNumFftSamples = 1000;

struct Context {
  std::array<float, kNumFftSamples> roll, pitch, yaw;
  std::array<float, kNumFftSamples> x, y, z;
  std::array<float, kNumFftSamples> db;

  TaskHandle_t task_update_sensor;
  TaskHandle_t task_calculate_fft;
  TaskHandle_t task_web_server;
};
Context *ctx;

// センサ更新タスク
[[noreturn]] void updateSensorTask(void *) {
  // IMUを設定
  M5.begin();

  // 収集したサンプルの数
  size_t num_samples = 0;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

    // 値を更新
    M5.Imu.update();

    // 値をセット
    M5.Imu.getAccel(&ctx->x[num_samples], &ctx->y[num_samples], &ctx->z[num_samples]);
    M5.Imu.getGyro(&ctx->roll[num_samples], &ctx->pitch[num_samples], &ctx->yaw[num_samples]);

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
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    // センサ取得完了通知が来るまで待つ
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    fft_config_t *real_fft_plain = fft_init(1024, FFT_REAL, FFT_FORWARD, nullptr, nullptr);
    for (auto n = 0; n < real_fft_plain->size; n++) {
      real_fft_plain->input[n] = ctx->x[n];
    }
    fft_execute(real_fft_plain);

    // 0にDC成分, 1にスペクトル中心?
    for (auto n = 1; n < real_fft_plain->size / 2; n++) {
      float real = real_fft_plain->output[2 * n] * real_fft_plain->output[2 * n];
      float imag = real_fft_plain->output[2 * n + 1] * real_fft_plain->output[2 * n + 1];
      float amp = std::sqrt(real + imag);
      printf("%f, ", amp);
    }
    printf("\n");

    fft_destroy(real_fft_plain);

    // 計算完了・再開通知
    xTaskNotifyGive(ctx->task_update_sensor);
  }
}

extern "C" void app_main() {
  ctx = new Context;
  xTaskCreatePinnedToCore(updateSensorTask, "updateSensor", 8192, nullptr, 10, &ctx->task_update_sensor, 0);
  xTaskCreatePinnedToCore(calculateFftTask, "calculateFft", 8192, nullptr, 10, &ctx->task_calculate_fft, 1);
}
