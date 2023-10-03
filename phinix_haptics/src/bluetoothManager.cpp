#include <bluefruit.h>
#include <Arduino.h>


bool isConnected = false; // Variable to track connection status

void beginBluetooth(BLEUart *bleuart, BLEDfu *bledfu)
{
    Bluefruit.begin();
    Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
    Bluefruit.setName("Ximira_Phinix_Bracelet");

    // To be consistent OTA DFU should be added first if it exists
    bledfu->begin();

    // Configure and start the BLE Uart service
    bleuart->begin();
}

void startAdv(BLEUart *bleuart)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(
      BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(*bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void bluetoothConnectionCheck(void)
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
}