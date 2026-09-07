#include <DJIMotorCtrlESP.hpp>
#include <HXC_TWAI.hpp>

#define MOTOR_TX_PIN D0
#define MOTOR_RX_PIN D1

#define SLAVE_TX_PIN D2
#define SLAVE_RX_PIN D3

#define NORMAL_RECEIVE_ID 0x710
#define REBOOT_RECEIVE_ID 0x012

#define SHINKU_PIN_1 D4
#define SHINKU_PIN_2 D5
#define SHINKU_PIN_3 D6

#define SLAVE1_WHEEL_CONTROL_ID 0x310    // タイヤ
#define SLAVE2_DISHES_ARM_ID 0x710       // お皿
#define SLAVE3_ZEUS_ARM_STS3215_ID 0x410 // 万能手腕
#define SLAVE4_SQUID_ARM_ID 0x110        // いかさん
#define SLAVE5_MARKER_ARM_ID 0x210       // マーカー
#define SLAVEX_BUTSUDAN_LED_ID 0x115     // 仏壇

uint16_t ID = 0;
int8_t data[8] = {0};

TaskHandle_t motor_control_handle = NULL;
twai_handle_t receive_handle = NULL;

HXC_TWAI CAN_BUS(MOTOR_TX_PIN, MOTOR_RX_PIN, CAN_RATE_1MBIT);

M2006_P36 MOTOR1(&CAN_BUS, 1);
M2006_P36 MOTOR2(&CAN_BUS, 2);
M2006_P36 MOTOR3(&CAN_BUS, 3);

void send(uint16_t ID /*ID*/, int8_t data1 /*識別子*/, int8_t data2 /*データ*/, int8_t data3, int8_t data4, int8_t data5, int8_t data6, int8_t data7, int8_t data8)
{

  twai_message_t SendFrame;

  SendFrame.identifier = ID;
  SendFrame.rtr = 0;
  SendFrame.extd = 0;
  SendFrame.data_length_code = 8;
  SendFrame.data[0] = data1;
  SendFrame.data[1] = data2;
  SendFrame.data[2] = data3;
  SendFrame.data[3] = data4;
  SendFrame.data[4] = data5;
  SendFrame.data[5] = data6;
  SendFrame.data[6] = data7;
  SendFrame.data[7] = data8;

  if (twai_transmit_v2(receive_handle, &SendFrame, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    Serial.printf("ID = %x, data1 = %x, data2 = %d, data3 = %d, data4 = %d, data5 = %d, data6 = %d, data7 = %d, data8 = %d\n", ID, data1, data2, data3, data4, data5, data6, data7, data8);
    Serial.println("送信成功");
  }

  return;
}

void receive(void *pvParameters)
{
  for (;;)
  {
    twai_message_t receiveframe;
    if (twai_receive_v2(receive_handle, &receiveframe, pdMS_TO_TICKS(10)) == ESP_OK)
    {
      if (receiveframe.identifier == NORMAL_RECEIVE_ID)
      {

        ID = receiveframe.identifier;
        data[0] = receiveframe.data[0]; // 命令の識別子
        data[1] = receiveframe.data[1]; // 命令の数値
        data[2] = receiveframe.data[2]; // ゴミ
        data[3] = receiveframe.data[3]; // ゴミ
        data[4] = receiveframe.data[4]; // ゴミ
        data[5] = receiveframe.data[5]; // ゴミ
        data[6] = receiveframe.data[6]; // ゴミ
        data[7] = receiveframe.data[7]; // ゴミ
        Serial.printf("ID = %x, data1 = %d, data2 = %d, data3 = %d, data4 = %d, data5 = %d, data6 = %d, data7 = %d, data8 = %d\n", ID, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        xTaskNotifyGive(motor_control_handle);
      }
      else if (receiveframe.identifier == REBOOT_RECEIVE_ID)
      {
        Serial.println("強制再起動を実行します");
        esp_restart();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void motor_control(void *pvParameters)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    switch (data[0])
    {

    case 1:
      MOTOR1.set_speed(50);
      break;

    case 2:
      MOTOR1.set_speed(-50);
      break;

    case 3:
      MOTOR2.set_speed(-30);
      break;

    case 4:
      MOTOR2.set_speed(30);
      break;

    case 6:
      MOTOR3.set_speed(-30);
      break;

    case 7:
      MOTOR3.set_speed(30);
      break;

    case 9:
      digitalWrite(SHINKU_PIN_1, LOW);
      digitalWrite(SHINKU_PIN_2, LOW);
      digitalWrite(SHINKU_PIN_3, LOW);
      break;

    case 10:
      digitalWrite(SHINKU_PIN_1, HIGH);
      digitalWrite(SHINKU_PIN_2, HIGH);
      digitalWrite(SHINKU_PIN_3, HIGH);
      break;

    case 11:
      MOTOR1.set_location(int(8192 * 36 * 3.3));
      MOTOR3.set_location(8192 * 36 * -3);
      MOTOR2.set_location(8192 * 36 * -3);
      while (abs((int(8192 * 36 * 3.3)) - MOTOR1.get_location()) > 5000 || abs((8192 * 36 * -3) - MOTOR2.get_location()) > 5000 || abs((8192 * 36 * -3) - MOTOR3.get_location()) > 5000)
        vTaskDelay(pdMS_TO_TICKS(1));
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(SHINKU_PIN_1, HIGH);
      digitalWrite(SHINKU_PIN_2, HIGH);
      digitalWrite(SHINKU_PIN_3, HIGH);
      break;

    case 12:
      // 例: Kp=1.5, Ki=0.0, Kd=0.1, 死区=0、最大速度を 50 に制限する場合
      MOTOR1.set_location_pid(3.5, 0.0, 0.1, 0.0, 1000.0); // kp, ki, 死区, 最高速度
      MOTOR2.set_location_pid(2.5, 0.0, 0.1, 0.0, 900.0);
      MOTOR3.set_location_pid(2.5, 0.0, 0.1, 0.0, 900.0);

      MOTOR1.set_location(0);
      MOTOR2.set_location(8192 * 36 * -5);
      MOTOR3.set_location(8192 * 36 * -5);
      while (abs(0 - MOTOR1.get_location()) > 5000 || abs((8192 * 36 * -5) - MOTOR2.get_location()) > 5000 || abs((8192 * 36 * -5) - MOTOR3.get_location()) > 5000)
        vTaskDelay(pdMS_TO_TICKS(1));
      vTaskDelay(pdMS_TO_TICKS(500));
      send(SLAVE5_MARKER_ARM_ID, 7, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      vTaskDelay(pdMS_TO_TICKS(100));
      send(SLAVE5_MARKER_ARM_ID, 7, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      break;

    case 13:
      vTaskDelay(pdMS_TO_TICKS(500));
      send(SLAVE5_MARKER_ARM_ID, 6, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      vTaskDelay(pdMS_TO_TICKS(100));
      send(SLAVE5_MARKER_ARM_ID, 6, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      MOTOR1.set_location(8192 * 36 * 2);
      MOTOR2.set_location(0);
      MOTOR3.set_location(0);
      while (abs(8192 * 36 * 2 - MOTOR1.get_location()) > 5000 || abs(0 - MOTOR2.get_location()) > 5000 || abs(0 - MOTOR3.get_location()) > 5000)
        vTaskDelay(pdMS_TO_TICKS(1));
      break;

    default:
      MOTOR1.set_speed(0);
      MOTOR2.set_speed(0);
      MOTOR3.set_speed(0);
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup()
{

  Serial.begin(115200);

  pinMode(SHINKU_PIN_1, OUTPUT);
  pinMode(SHINKU_PIN_2, OUTPUT);
  pinMode(SHINKU_PIN_3, OUTPUT);

  while (CAN_BUS.setup() != ESP_OK)
  {
    Serial.println("モーター用CANバスのセットアップに失敗しました。一秒後に再試行します。");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  Serial.println("モーター用CANセットアップ完了");

  MOTOR1.setup();
  MOTOR2.setup();
  MOTOR3.setup();

  // 例: Kp=1.5, Ki=0.0, Kd=0.1, 死区=0、最大速度を 50 に制限する場合
  MOTOR1.set_location_pid(3.5, 0.0, 0.1, 0.0, 1800.0); // kp, ki, 死区, 最高速度
  MOTOR2.set_location_pid(2.5, 0.0, 0.1, 0.0, 1900.0);
  MOTOR3.set_location_pid(2.5, 0.0, 0.1, 0.0, 1900.0);

  /***********************************CAN関連********************************************/
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(1, (gpio_num_t)SLAVE_TX_PIN, (gpio_num_t)SLAVE_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t ret = twai_driver_install_v2(&g_config, &t_config, &f_config, &receive_handle);
  if (ret == ESP_OK)
    Serial.println("インストール完了");
  else
    Serial.println("インストール失敗");
  ret = twai_start_v2(receive_handle);
  if (ret == ESP_OK)
    Serial.println("スタート完了");
  else
    Serial.println("スタート失敗");
  /**************************************************************************************/

  xTaskCreateUniversal(
      receive,
      "receive",
      8192,
      NULL,
      2,
      NULL,
      PRO_CPU_NUM);

  xTaskCreateUniversal(
      motor_control,
      "motor_control",
      4096,
      NULL,
      1,
      &motor_control_handle,
      PRO_CPU_NUM);
}

void loop()
{
  vTaskDelete(NULL);
}
