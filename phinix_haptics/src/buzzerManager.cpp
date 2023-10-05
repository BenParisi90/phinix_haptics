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
  int levelIndex;
  bool active;
};

// Define a vector of vectors to store the buzz commands for each channel
std::vector<std::vector<BuzzCommand>> activeBuzzCommands(8);

void addBuzzCommand(int channel, unsigned long startDelay, int level) {
  Serial.print("Adding buzz command: ");
  Serial.print(channel);
  Serial.print(" ");
  Serial.print(startDelay);
  Serial.print(" ");
  Serial.print(level);
  Serial.println(" ");
  BuzzCommand buzzCommand;
  buzzCommand.channel = buzzerChannelMap[channel];
  buzzCommand.startTime = millis() + startDelay;
  buzzCommand.levelIndex = level;
  buzzCommand.active = false;
  // Add the buzz command to the vector for the specified channel
  activeBuzzCommands[channel].push_back(buzzCommand);
}

void updateBuzzCommands(float dt) {
  for (int i = 0; i < 8; i++) {
    // Iterate over the vector for each channel
    for (int j = activeBuzzCommands[i].size() - 1; j >= 0; j--) {
      BuzzCommand& buzzCommand = activeBuzzCommands[i][j]; // Use a reference to update the original object
      unsigned long currentTime = millis();
      if (!buzzCommand.active && currentTime >= buzzCommand.startTime) {
        audio_tactile::SleeveTactors.UpdateChannel(buzzCommand.channel, levels[buzzCommand.levelIndex]);
        buzzCommand.active = true;
      }
    }
  }
}

void directSetBuzzerStrength(uint8_t packetbuffer[]) {
  for (int i = 0; i < 8; i++) {
    int buzzerChannel = buzzerChannelMap[i];
    int buzzerStrength = packetbuffer[i] - '0';
    Serial.println(buzzerStrength);
    audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel, levels[buzzerStrength]);
  }
}

void clearChannel(int channel) {
  activeBuzzCommands[channel].clear();
}