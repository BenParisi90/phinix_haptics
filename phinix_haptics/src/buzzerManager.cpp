#include <Arduino.h>
#include <vector>
#include "pwm_sleeve.h"

uint16_t pattern[8] = {0};

int currentBuzzer = 0;
const int buzzerChannelMap[8] = {10, 6, 9, 8, 11, 7, 0, 0};

struct BuzzCommand {
  int channel;
  unsigned long startTime;
  int crest;
  int trough;
  int wavelength;
};

int doubleMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPattern(uint16_t level)
{
  for(int i = 0; i < 8; i++)
  {
    pattern[i] = level;
  }
}

// Define a vector of vectors to store the buzz commands for each channel
std::vector<std::vector<BuzzCommand>> pendingBuzzCommands(8);
std::vector<BuzzCommand> activebuzzCommands(8);

void addBuzzCommand(int channel, unsigned long startDelay, int crest, int trough, int wavelength) {
  Serial.print("Adding buzz command: ");
  Serial.print(channel);
  Serial.print(" ");
  Serial.print(startDelay);
  Serial.print(" ");
  Serial.print(crest);
  Serial.print(" ");
  Serial.print(trough);
  Serial.print(" ");
  Serial.print(wavelength);
  Serial.println(" ");
  BuzzCommand buzzCommand;
  buzzCommand.channel = buzzerChannelMap[channel];
  buzzCommand.startTime = millis() + startDelay;
  buzzCommand.crest = crest;
  buzzCommand.trough = trough;
  buzzCommand.wavelength = wavelength;
  // Add the buzz command to the vector for the specified channel
  pendingBuzzCommands[channel].push_back(buzzCommand);
}

void processPendingBuzzCommands(float dt) {
  unsigned long currentTime = millis();
  for (int i = 0; i < 8; i++) {
    if(pendingBuzzCommands[i].size() == 0)
    {
      continue;
    }
    // Iterate over the vector for each channel
    BuzzCommand& buzzCommand = pendingBuzzCommands[i][0]; // Use a reference to update the original object

    if (currentTime >= buzzCommand.startTime) 
    {
      activebuzzCommands[buzzCommand.channel] = buzzCommand;
      
      if(buzzCommand.wavelength == 0)
      {
        Serial.println("Execute buzz comand");
        Serial.println(buzzCommand.crest);
        uint16_t level[8] = {buzzCommand.crest};
        audio_tactile::SleeveTactors.UpdateChannel(buzzCommand.channel, level);
      }
      pendingBuzzCommands[i].erase(pendingBuzzCommands[i].begin());
    }
  }
  /*
  double wave = sin(currentTime/1000.0);
  int output = doubleMap(wave, -1.0, 1.0, 0.0, 255.0);
  Serial.println(output);


  uint16_t level[8] = {output};
  audio_tactile::SleeveTactors.UpdateChannel(buzzerChannelMap[3], level);
  */
}

void processActiveBuzzCommands()
{
  unsigned long currentTime = millis();
  for(int i = 0; i < 8; i ++)
  {
    BuzzCommand& buzzCommand = activebuzzCommands[i];
    if(buzzCommand.wavelength != 0)
    {
      double wave = sin((currentTime - buzzCommand.startTime)/.159154 * buzzCommand.wavelength);
      int output = doubleMap(wave, -1.0, 1.0, buzzCommand.trough, buzzCommand.crest);
      Serial.println(output);
      uint16_t level[8] = {output};
      audio_tactile::SleeveTactors.UpdateChannel(buzzCommand.channel, level);
    }
  }
}

void directSetBuzzerStrength(uint8_t packetbuffer[]) {
  for (int i = 0; i < 8; i++) {
    int buzzerChannel = buzzerChannelMap[i];
    int buzzerStrength = packetbuffer[i] - '0';
    Serial.println(buzzerStrength);
    setPattern(255 / 5 * buzzerStrength);
    audio_tactile::SleeveTactors.UpdateChannel(buzzerChannel, pattern);
  }
}

void clearChannel(int channel) {
  pendingBuzzCommands[channel].clear();
}