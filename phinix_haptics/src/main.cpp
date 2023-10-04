#include <bluefruit.h>
#include "pwm_sleeve.h"
#include <Arduino.h>


// Function prototypes for packetparser.cpp
uint8_t readPacket(BLEUart *ble_uart);//, uint16_t timeout);

void OnPwmSequenceEnd();

void startAdv(BLEUart *bleuart);
void beginBluetooth(BLEUart *bleuart, BLEDfu *bledfu);
void bluetoothConnectionCheck();

void addBuzzCommand(int channel, unsigned long startDelay, unsigned long duration, int level);
void updateBuzzCommands(float dt);

void directSetBuzzerStrength(uint8_t packetbuffer[]);

// Packet buffer
extern uint8_t packetbuffer[];

// OTA DFU service
BLEDfu bledfu;
BLEUart bleuart;

//delta time
int currentMillis = 0;
int previousMillis = 0;
int dt = 0;

bool testBuzz = false;

void setup(void)
{
  nrf_gpio_cfg_output(kLedPinBlue);
  nrf_gpio_cfg_output(kLedPinGreen);

  nrf_gpio_pin_write(kLedPinBlue, 0);
  nrf_gpio_pin_write(kLedPinGreen, 0);
  
  Serial.begin(115200);

  beginBluetooth(&bleuart, &bledfu);

  // Set up and start advertising
  startAdv(&bleuart);

  Serial.println(
      F("Bluetooth setup and advertising!"));

  audio_tactile::SleeveTactors.OnSequenceEnd(OnPwmSequenceEnd);
  audio_tactile::SleeveTactors.Initialize();
  nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_SEQSTART0);
  nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_SEQSTART0);

  nrf_gpio_pin_write(kLedPinGreen, 1);

  currentMillis = millis();
  previousMillis = currentMillis;
}



void loop(void)
{
  //delta time
  currentMillis = millis();
  dt = currentMillis - previousMillis;
  previousMillis = currentMillis;

  //play a test buzz on start
  Serial.println(currentMillis);
  if(!testBuzz && currentMillis > 5000)
  {
    testBuzz = true;
    addBuzzCommand(0, 0, 1000, 5);
    addBuzzCommand(1, 1000, 2000, 1);
    addBuzzCommand(2, 3000, 3000, 2);
  }

  updateBuzzCommands(dt);
  
  bluetoothConnectionCheck();

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

    directSetBuzzerStrength(packetbuffer);
  }
}

void OnPwmSequenceEnd() {}
