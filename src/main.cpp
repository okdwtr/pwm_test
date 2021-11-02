#include <Arduino.h>
#include "driver/mcpwm.h"

volatile uint16_t duty = 0;

void IRAM_ATTR onButtonUp() {
  if (duty < UINT16_MAX) duty++;
  Serial.println(" -----  Up  ----- ");
  //noInterrupts();
}

void IRAM_ATTR onButtonDown() {
  if (duty > 0) duty--;
  Serial.println(" ----- Down ----- ");
  //noInterrupts();
}

void setup() {
  // put your setup code here, to run once:
  // シリアル
  Serial.begin(115200);

  // 割り込み設定
  Serial.println("interrupt setup");
  pinMode(GPIO_NUM_36, INPUT_PULLDOWN);
  pinMode(GPIO_NUM_39, INPUT_PULLDOWN);
  //attachInterrupt(GPIO_NUM_36, onButtonUp, RISING);
  //attachInterrupt(GPIO_NUM_39, onButtonDown, RISING);
  //noInterrupts();

  // ピン設定
  Serial.println("mcpwm pin setup");
  mcpwm_pin_config_t pinConfig;
  pinConfig.mcpwm0a_out_num = GPIO_NUM_16;
  pinConfig.mcpwm0b_out_num = GPIO_NUM_17;
  pinConfig.mcpwm_fault0_in_num = GPIO_NUM_39;  // limH
  pinConfig.mcpwm_fault1_in_num = GPIO_NUM_36;  // limL
  mcpwm_set_pin(MCPWM_UNIT_0, &pinConfig);
  /*
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_16);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_NUM_16);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, GPIO_NUM_39);  // limH
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_1, GPIO_NUM_36);  // limL
  */

  // PWM設定
  Serial.println("mcpwm param setup");
  mcpwm_config_t pwmConfig;
  pwmConfig.frequency = 20000;  //20KHz
  pwmConfig.cmpr_a = 50;  //50%
  pwmConfig.cmpr_b = 50;  //50%
  pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;
  pwmConfig.counter_mode = MCPWM_UP_COUNTER;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);

  // freq
  uint32_t freq = 20000;  //frequency[Hz] (default: 20[kHz], 12[bit])
  freq = mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0);
  Serial.println("freq: " + String(freq));
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, freq);

  // deadtime
  uint32_t red = 10, fed = 50;  //deadtime[100ns]
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, red, fed);
  //mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("mcpwm test");
  delay(500);
  
  // 1000[ms]動かす
  Serial.println("mcpwm start 1000[ms]");
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  delay(1000);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  delay(1000);

  Serial.println("mcpwm signal test");
  delay(500);

  // 出力固定
  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  delay(1000);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  delay(1000);

  Serial.println("mcpwm duty test");
  delay(500);

  // duty変更
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // duty比呼び戻し
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1); // duty比呼び戻し
  mcpwm_start(MCPWM_UNIT_0,MCPWM_TIMER_0);
  delay(500);
  float duty_percent = 0;
  for (size_t i = 0; i < 1000; i++) {
    duty_percent = i / 10.0;
    esp_err_t e;
    e = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
    if (e != ESP_OK) Serial.println("error: " + String(e));
    e = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0 - duty_percent);
    if (e != ESP_OK) Serial.println("error: " + String(e));
    Serial.println("mcpwm duty[%]: " + String(duty_percent));
    delay(100);
  }
  delay(1000);
  uint32_t duty_us = 0;
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us);
  delay(1000);

/*
  // fault handler（LOW->HIのみ対応）
  mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_LOW); //1回だけ
  mcpwm_fault_set_cyc_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_LOW); //戻ったら再始動
  mcpwm_fault_deinit(MCPWM_UNIT_0, MCPWM_SELECT_F0);  //
*/
/*
  while (1) {
    // ボタンでdutyをいじるやつ
    Serial.println("mcpwm duty control");
    float d = duty / UINT16_MAX;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, d);
    Serial.println("mcpwm duty[%]: " + String(d));
    delay(250);
    //interrupts();
  }
*/
  delay(1000);
}