#include <bluefruit.h>
#include "pwm_sleeve.h"
#include <Arduino.h>
#include <vector>

// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// Function prototypes for packetparser.cpp
uint8_t readPacket(BLEUart *ble_uart);//, uint16_t timeout);

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
const uint16_t* levels[] = {silence, level1, level2, level3, level4, level5};

int currentBuzzer = 0;
const int buzzerChannelMap[8] = {10, 6, 9, 8, 11, 7, 0, 0};

bool testBuzz = false;

//delta time
int currentMillis = 0;
int previousMillis = 0;
int dt = 0;

void OnPwmSequenceEnd();
void startAdv();

struct BuzzCommand {
  int channel;
  unsigned long startTime;
  unsigned long duration;
  int levelIndex;
  bool active = false;
};

std::vector<BuzzCommand> activeBuzzCommands;

void addBuzzCommand(int channel, unsigned long startDelay, unsigned long duration, int level) {
  BuzzCommand buzzCommand;
  buzzCommand.channel = buzzerChannelMap[channel];
  buzzCommand.startTime = millis() + startDelay;
  buzzCommand.duration = duration;
  buzzCommand.levelIndex = level;
  activeBuzzCommands.push_back(buzzCommand);
}

void updateBuzzCommands(float dt) {
  for (int i = activeBuzzCommands.size() - 1; i >= 0; i--) {
    BuzzCommand& buzzCommand = activeBuzzCommands[i]; // Use a reference to update the original object
    unsigned long currentTime = millis();
    if (!buzzCommand.active && currentTime >= buzzCommand.startTime) {
      audio_tactile::SleeveTactors.UpdateChannel(buzzCommand.channel, levels[buzzCommand.levelIndex]);
      buzzCommand.active = true;
    } else if(buzzCommand.active && currentTime >= buzzCommand.startTime + buzzCommand.duration){
      nrf_gpio_pin_write(kLedPinBlue, 1);
      audio_tactile::SleeveTactors.UpdateChannel(buzzCommand.channel, silence);
      buzzCommand.active = false;
      activeBuzzCommands.erase(activeBuzzCommands.begin() + i); // Remove the command from the vector
    }
  }
}

void setup(void)
{

  nrf_gpio_cfg_output(kLedPinBlue);
  nrf_gpio_cfg_output(kLedPinGreen);

  nrf_gpio_pin_write(kLedPinBlue, 0);
  nrf_gpio_pin_write(kLedPinGreen, 0);
  
  Serial.begin(115200);

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

  currentMillis = millis();
  previousMillis = currentMillis;
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

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
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

    for(int i = 0; i < 8; i++)
    {
      int buzzerChannel = buzzerChannelMap[i];
      int buzzerStrength = packetbuffer[i] - '0';
      Serial.println(buzzerStrength);
      audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel, levels[buzzerStrength]);
    }
  }
}

void OnPwmSequenceEnd() {}
