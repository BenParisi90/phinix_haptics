#include <bluefruit.h>
#include "pwm_sleeve.h"
#include <Arduino.h>


// Function prototypes for packetparser.cpp
uint8_t readPacket(BLEUart *ble_uart);//, uint16_t timeout);

void OnPwmSequenceEnd();

void startAdv(BLEUart *bleuart);
void beginBluetooth(BLEUart *bleuart, BLEDfu *bledfu);
void bluetoothConnectionCheck();

void addBuzzCommand(int channel, unsigned long startDelay, int crest, int trough, int wavelength);
void processPendingBuzzCommands(float dt);
void processActiveBuzzCommands();
void clearChannel(int channel);

void directSetBuzzerStrength(uint8_t packetbuffer[]);

void ObjectDetection();
void odDetectBuzzerColumn(uint16_t topIndex, uint16_t middleIndex, uint16_t bottomIndex, uint16_t channel, uint16_t time);
void odPlayBuzzerColumn(bool top, bool middle, bool bottom, uint16_t channel, uint16_t time);

void PathDetection();
void PlayPathDetectionSide(int channel, char urgency);

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
int testBuzzChannel = 0;

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
  /*
  if(testBuzz && currentMillis > 5000)
  {
    testBuzz = false;
    addBuzzCommand(0, 0, 1000, 5);
    addBuzzCommand(1, 1000, 2000, 1);
    addBuzzCommand(2, 3000, 3000, 2);
  }
  */

  processPendingBuzzCommands(dt);

  processActiveBuzzCommands();
  
  bluetoothConnectionCheck();

  // Wait for new data to arrive
  uint8_t len = readPacket(&bleuart);
  if (len > 0)
  {
    //nrf_gpio_pin_write(kLedPinBlue, 0);
    //nrf_gpio_pin_write(kLedPinGreen, 0);
    
    // Got a packet!
    //printHex(packetbuffer, len);

    // Print received UART data
    /*
    o - objectDetection
    p - pathDetection

    c - cardinal direction
    m - memorize person
    e - enable
    d - disable
    b - battery level
    x - emergency
    */
    Serial.write(packetbuffer, len);
    if(packetbuffer[0] == 'o')
    {
      ObjectDetection();
    }
    if(packetbuffer[0] == 'p')
    {
      PathDetection();
    }
    if(packetbuffer[0] == 't')
    {
      uint16_t level[8] = {100};
      uint16_t silence[8] = {0};
      audio_tactile::SleeveTactors.UpdateChannel(testBuzzChannel, silence);
      testBuzzChannel++;
      Serial.println(testBuzzChannel);
      audio_tactile::SleeveTactors.UpdateChannel(testBuzzChannel, level);
    }

    //directSetBuzzerStrength(packetbuffer);
  }
}

void OnPwmSequenceEnd() {}

/* 
An Array of 0 or 1 values that correspond to grid sections
[1] upper left 
[2] middle left
[3] lowerLeft
[4] upper center
[5] middle center
[6] lower center
[7] upper right
[8] middle right
[9] lower right
*/
void ObjectDetection()
{
  //left column
  odDetectBuzzerColumn(1, 2, 3, 2, 2000);
  //middle column
  odDetectBuzzerColumn(4, 5, 6, 3, 2000);
  //right column
  odDetectBuzzerColumn(7, 8, 9, 4, 2000);
}

void odDetectBuzzerColumn(uint16_t topIndex, uint16_t middleIndex, uint16_t bottomIndex, uint16_t channel, uint16_t time)
{
  bool top = packetbuffer[topIndex] == '1';
  bool middle = packetbuffer[middleIndex] == '1';
  bool bottom = packetbuffer[bottomIndex] == '1';
  odPlayBuzzerColumn(top, middle, bottom, channel, time);
}

//buzz the channel for 2 seconds, split between detected high mid and low
void odPlayBuzzerColumn(bool top, bool middle, bool bottom, uint16_t channel, uint16_t time)
{
  uint16_t channelCount = (top) + (middle) + (bottom);
  uint16_t timePerBuzz = time / channelCount;
  uint16_t startTime = 0;

  if(channelCount == 0)
  {
    return;
  }

  clearChannel(channel);
  
  if(top)
  {
    addBuzzCommand(channel, startTime, 255, 255, 0);
    startTime += timePerBuzz;
  }
  if(middle)
  {
    addBuzzCommand(channel, startTime, 153, 153, 0);
    startTime += timePerBuzz;
  }
  if(bottom)
  {
    addBuzzCommand(channel, startTime, 55, 55, 0);
    startTime += timePerBuzz;
  }
  addBuzzCommand(channel, startTime, 0, 0, 0);
}

void PathDetection()
{
  Serial.println("PathDetection");
  Serial.println(packetbuffer[1]);
  Serial.println(packetbuffer[2]);
  PlayPathDetectionSide(1, packetbuffer[1]);
  PlayPathDetectionSide(5, packetbuffer[2]);
}

void PlayPathDetectionSide(int channel, char urgency)
{
  int urgencyInt = urgency - '0';
  if(urgencyInt == 0)
  {
    return;
  }
  int strength = 255 / (4 - urgencyInt);
  addBuzzCommand(channel, 0, strength, 0, 250);
  addBuzzCommand(channel, 1500, 0, 0, 0);
}