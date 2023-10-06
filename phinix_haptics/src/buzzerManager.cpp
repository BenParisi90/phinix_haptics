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
};

int doubleMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Define a vector of vectors to store the buzz commands for each channel
std::vector<std::vector<BuzzCommand>> pendingBuzzCommands(8);
std::vector<BuzzCommand> activebuzzCommands(8);

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
  // Add the buzz command to the vector for the specified channel
  pendingBuzzCommands[channel].push_back(buzzCommand);
}

void updateBuzzCommands(float dt) {
  unsigned long currentTime = millis();
  for (int i = 0; i < 8; i++) {
    // Iterate over the vector for each channel
    for (int j = pendingBuzzCommands[i].size() - 1; j >= 0; j--) {
      BuzzCommand& buzzCommand = pendingBuzzCommands[i][j]; // Use a reference to update the original object

      if (currentTime >= buzzCommand.startTime) 
      {
        audio_tactile::SleeveTactors.UpdateChannel(buzzCommand.channel, levels[buzzCommand.levelIndex]);
        activebuzzCommands[buzzCommand.channel] = buzzCommand;
        pendingBuzzCommands[i].erase(pendingBuzzCommands[i].begin() + j);
      }
    }
  }
  double wave = sin(currentTime/1000.0);
  int output = doubleMap(wave, -1.0, 1.0, 0.0, 255.0);
  Serial.println(output);
  //Serial.println(longMap(wave, -1.0, 1.0, 0.0, 255.0));


  uint16_t level[8] = {output};
  audio_tactile::SleeveTactors.UpdateChannel(buzzerChannelMap[3], level);
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
  pendingBuzzCommands[channel].clear();
}