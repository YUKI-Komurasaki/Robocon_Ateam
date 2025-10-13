#include <Arduino.h>
#include <nvs.h>
#include <nvs_flash.h>

void clearNVS() {
  esp_err_t err;

  err = nvs_flash_init();
  Serial.print("nvs_flash_init: ");
  Serial.println(err);

  err = nvs_flash_erase();
  Serial.print("nvs_flash_erase: ");
  Serial.println(err);

  err = nvs_flash_init();
  Serial.print("nvs_flash_re-init: ");
  Serial.println(err);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n==============================");
  Serial.println("NVS（Bluetoothペアリング情報）を初期化しています...");
  Serial.println("==============================");
  clearNVS();
  Serial.println(" -完了- ESP32を再起動または再書き込みしてください。\n");
  Serial.println("ps4コントローラリセットは、SHARE+PS同時長押しです。");
}

void loop() {}