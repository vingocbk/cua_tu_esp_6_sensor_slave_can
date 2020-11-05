#include "AppDebug.h"
#include <Arduino.h>
#include "Ticker.h"
#include "WebServer.h"
#include "WiFi.h"
#include "EEPROM.h"
#include <ESP32CAN.h>
#include <CAN_config.h>
#include "EspOTA.h"
#include "ArduinoJson.h"

#include "soc/soc.h"  //Brownout detector was triggered
#include "soc/rtc_cntl_reg.h"


#define hallSensor1 14
#define hallSensor2 27  
#define hallSensor3 26
#define hallSensor4 4
#define hallSensor5 16
#define hallSensor6 17
#define ledTestWifi 32
#define inputFG 21
#define PWM 18
#define DIR 19
#define ANALOG_READ_BUTTON 13 //34
#define PIN_CONFIG 0
#define PIN_TOUCH_SENSOR T5
#define PIN_LED_LIGHT_R 22
#define PIN_LED_LIGHT_G 23  //5 with design
#define PIN_LED_LIGHT_B 5   //23 with design
#define LED_CHANNEL_R 1
#define LED_CHANNEL_G 2
#define LED_CHANNEL_B 3
#define MOTOR_CHANNEL 4

#define WL_MAC_ADDR_LENGTH 6
#define SSID_PRE_AP_MODE "AvyInterior-"
#define PASSWORD_AP_MODE "123456789"

// old motor
//#define QUAY_THUAN HIGH
//#define QUAY_NGHICH LOW

  // new motor
 #define QUAY_THUAN LOW
 #define QUAY_NGHICH HIGH

#define MINSPEED 5

#define RESPONSE_LENGTH                     2048     //do dai data nhan ve tu tablet
#define EEPROM_DEVICE_ID                    65
#define EEPROM_SET_MODE_RUN_BEGIN           149
#define EEPROM_DISTANT                      150
#define EEPROM_SET_PERCENT_OUT_LOW_SPEED    151
#define EEPROM_SET_PERCENT_IN_LOW_SPEED     152
#define EEPROM_SET_TIME_RETURN              153
#define EEPROM_WIFI_LED_RED                 154
#define EEPROM_WIFI_LED_GREEN               155 
#define EEPROM_WIFI_LED_BLUE                156
#define EEPROM_WIFI_IS_TOUCH_SENSOR         157
#define EEPROM_DELAY_MQTT_TO_LOW_SPEAKER    158
#define EEPROM_COUNT_TIME_ANALOG            159
#define EEPROM_VALUE_ERROR_ANALOG           160
#define EEPROM_TIME_AUTO_CLOSE              161
#define EEPROM_MIN_STOP_SPEED               162
#define EEPROM_WIFI_MAX_CLEAR               256


#define MOTOR_MODE_SPEED_MAX      255
#define MOTOR_MODE_SPEED_STOP     0
#define MOTOR_MODE_SPEED_1        200
#define MOTOR_MODE_SPEED_2        150
#define MOTOR_MODE_SPEED_3        100
#define MOTOR_MODE_SPEED_4        50


#define MSG_MASTER_ID         0
#define MSG_GET_STATUS        1
#define MSG_CONTROL_OPEN      2
#define MSG_CONTROL_CLOSE     3
#define MSG_CONTROL_STOP      4
#define MSG_CONTROL_LED_VOICE 5
#define MSG_CONTROL_LED_HAND  6
#define MSG_RESET_DISTANT     7
#define MSG_TIME_RETURN       8
#define MSG_MODE_RUN          9
#define MSG_PERCENT_LOW       10
#define MSG_DELAY_ANALOG      11
#define MSG_ERROR_ANALOG      12
#define MSG_AUTO_CLOSE        13
#define MSG_MIN_STOP_SPEED    14

#define MSG_SET_ID       100


#define CONFIG_HOLD_TIME 5000
#define TIME_CHECK_ANALOG 500       //500ms check 1 lan

#define HTTP_PORT 80
WebServer server(HTTP_PORT);

//normal mode
void getStatus();
void motor_init();
void led_init();
void IRAM_ATTR inputSpeed();
void IRAM_ATTR dirhallSensor1();
void IRAM_ATTR dirhallSensor2();
void IRAM_ATTR dirhallSensor3();
void IRAM_ATTR dirhallSensor4();
void IRAM_ATTR dirhallSensor5();
void IRAM_ATTR dirhallSensor6();
void setpwmMotor();
void tickerupdate();
void caculateSpeed();
void setSpeedControl();
void buttonClick();
void Open();
void Close();
void Stop();

void setPwmLedLighton();
void setPwmLedLightoff();
void setPwmLedLightChange();
void checkAnalogReadButton();
void checkAutoClose();
void receiveDataCan();


void setLedApMode();
void checkButtonConfigClick();
void SetupConfigMode();
void StartConfigServer();
void ConfigMode();                  //nhan data tu app
String MacID();

struct motor
{
  /* data */
  int sensor_hall_duplicate;
  int count_pul_FG;
  int count_pul_distant;
  int pul_speed;
  int pre_pul_speed;
  int count_first_run;
  int count_calcu_speed;
  int mode_run;
  int time_return;
  int percent_low_in;
  int percent_low_out;
  int device_id;
  int time_delay_analog;
  int value_error_analog;
  int pre_value_error_analog;
  uint32_t time_check_analog;
  int define_error_analog; 
  int define_time_auto_close;
  uint32_t time_auto_close;   
  int stop_speed;
 
  float speed_velectory;

  bool forward;
  bool daytay;
  bool status_stop;
  bool normal_mode;
  bool is_save_distant;
  bool first_run;
  bool flag_auto_close;
  bool hall_sensor_1;
  bool hall_sensor_2;
  bool hall_sensor_3;
  bool hall_sensor_4;
  bool hall_sensor_5;
  bool hall_sensor_6;

}control_motor;

struct led
{
  /* data */
  int red_after;
  int green_after;
  int blue_after;
  int red_before;
  int green_before;
  int blue_before;
  int count_change_led;
  bool status_led;
}control_led;

struct config
{
  /* data */
  uint32_t time_click_button_config;
}configAPmode;


CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size


Ticker tickerCaculateSpeed(caculateSpeed, 100);   //every 100ms
Ticker tickerSetApMode(setLedApMode, 200, 0);   //every 200ms
Ticker tickerSetPwmLedLightOn(setPwmLedLighton, 20, 255);	//every 20ms
Ticker tickerSetPwmLedLightOff(setPwmLedLightoff, 10, 255);
Ticker tickerSetPwmLedLightChange(setPwmLedLightChange, 10, 255);
