#include <bluefruit.h>
#include "pwm_sleeve.h"
#include <Arduino.h>

// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// Function prototypes for packetparser.cpp
uint8_t readPacket(BLEUart *ble_uart);//, uint16_t timeout);
//float parsefloat(uint8_t *buffer);
//void printHex(const uint8_t *data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer[];

bool isConnected = false; // Variable to track connection status

static uint16_t level1[8] = {51};
static uint16_t level2[8] = {102};
static uint16_t level3[8] = {153};
static uint16_t level4[8] = {204};
static uint16_t level5[8] = {255};
static uint16_t silence[8] = {0};
uint16_t pattern[8] = {0};

int currentBuzzer = 0;
const int buzzerChannelMap[8] = {10, 6, 9, 8, 11, 7, 0, 0};

void OnPwmSequenceEnd();
void startAdv();

void setup(void)
{

  nrf_gpio_cfg_output(kLedPinBlue);
  nrf_gpio_cfg_output(kLedPinGreen);

  nrf_gpio_pin_write(kLedPinBlue, 0);
  nrf_gpio_pin_write(kLedPinGreen, 0);
  

  Serial.begin(115200);
  //while (!Serial)
  //  delay(10); // for nrf52840 with native USB

  nrf_gpio_pin_write(kLedPinGreen, 1);

  Serial.println(F("Adafruit Bluefruit52 Controller App Example"));
  Serial.println(F("-------------------------------------------"));

  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Ximira_Phinix_Bracelet");

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println(
      F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(
      F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  audio_tactile::SleeveTactors.OnSequenceEnd(OnPwmSequenceEnd);
  audio_tactile::SleeveTactors.Initialize();
  nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_SEQSTART0);
  nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_SEQSTART0);

  nrf_gpio_pin_write(kLedPinGreen, 1);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(
      BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check if the connection status has changed
  bool newIsConnected = Bluefruit.connected();
  if (newIsConnected != isConnected)
  {
    isConnected = newIsConnected;
    if (isConnected)
    {
      Serial.println("Device connected!");
    }
    else
    {
      Serial.println("Device disconnected!");
    }
  }

  // Wait for new data to arrive
  uint8_t len = readPacket(&bleuart);
  if (len > 0)
  {
    nrf_gpio_pin_write(kLedPinBlue, 0);
    nrf_gpio_pin_write(kLedPinGreen, 0);
    // Got a packet!
    //printHex(packetbuffer, len);

    // Print received UART data
    Serial.write(packetbuffer, len);

    for(int i = 0; i < 8; i ++)
    {
      int buzzerChannel = buzzerChannelMap[i];
      int buzzerStrength = packetbuffer[i];
      buzzerStrength -= 48;
      Serial.println(buzzerStrength);
      //pattern[i] = map(buzzerStrength, 0, 9, 0, 100);
      if(buzzerStrength == 0)
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel,silence);
      }
      if(buzzerStrength == 1)
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel,level1);
      }
      if(buzzerStrength == 2)
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel,level2);
      }
      if(buzzerStrength == 3)
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel,level3);
      }
      if(buzzerStrength == 4)
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel,level4);
      }
      if(buzzerStrength == 5)
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel,level5);
      }
    }

    /*
    if(packetbuffer[0] == '0')
    {
      for (int c = 0; c < 12; c++) {
        audio_tactile::SleeveTactors.UpdateChannel(c,silence);
      }
      
      currentBuzzer = 0;

    }
    if(packetbuffer[0] == '1')
    {
      audio_tactile::SleeveTactors.UpdateChannel(currentBuzzer,silence);
      currentBuzzer ++;
      //Serial.println(currentBuzzer);
      audio_tactile::SleeveTactors.UpdateChannel(currentBuzzer,sin_wave);
      if(currentBuzzer == 12)
      {
        currentBuzzer = 0;
      }
    }
    if(packetbuffer[0] == '2')
    {
      for (int c = 0; c < 12; c++) {
        audio_tactile::SleeveTactors.UpdateChannel(c,sin_wave);
      }
      
      currentBuzzer = 0;

    }
    */


    /*for (int c = 0; c < 12; c++) {
      if(packetbuffer[0] == '1')
      {
        Serial.println("buzzer on");
        audio_tactile::SleeveTactors.UpdateChannel(c,sin_wave);
      }
      if(packetbuffer[0] == '0')
      {
        Serial.println("buzzer off");
        audio_tactile::SleeveTactors.UpdateChannel(c,silence);
      }
    }*/
  }
}

void OnPwmSequenceEnd() {}
