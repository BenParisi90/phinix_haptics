#include <Arduino.h>
#include <vector>
#include "pwm_sleeve.h"

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

void directSetBuzzerStrength(uint8_t packetbuffer[])
{
    for(int i = 0; i < 8; i++)
    {
      int buzzerChannel = buzzerChannelMap[i];
      int buzzerStrength = packetbuffer[i] - '0';
      Serial.println(buzzerStrength);
      audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel, levels[buzzerStrength]);
    }
}