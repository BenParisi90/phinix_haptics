#include <string.h>
#include <Arduino.h>
#include <bluefruit.h>

//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (20)


/* Buffer to hold incoming characters */
uint8_t packetbuffer[20];

/**************************************************************************/
/*!
    @brief  Waits for incoming data and parses it
*/
/**************************************************************************/
uint8_t readPacket(BLEUart *ble_uart)
{
  uint16_t replyidx = 0;

  memset(packetbuffer, 0, READ_BUFSIZE);

  while (ble_uart->available()) {
    char c =  ble_uart->read();
    if (c == '!') {
      replyidx = 0;
    }
    else if(c == ';') {
      
    }
    else
    {
      Serial.print(c);
      packetbuffer[replyidx] = c;
      replyidx++;
    }
  }

  if (!replyidx)  // no data or timeout 
    return 0;
  
  
  // checksum passed!
  return replyidx;
}

