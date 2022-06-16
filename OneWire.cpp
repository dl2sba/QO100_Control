#include <arduino.h>
#include <OneWire.h>

#define ONEWIRE_PORT  19

#define DS18B20_START_CONVERSION  0x44
#define DS18B20_READ_SCRATCHPAD   0xBE
#define DS18B20_WRITE_SCRATCHPAD  0x4E
#define DS18B20_RESOLUTION_9BIT   0x1F
#define DS18B20_RESOLUTION_10BIT  0x3F
#define DS18B20_RESOLUTION_11BIT  0x5F
#define DS18B20_RESOLUTION_12BIT  0x7F

OneWire  oneWirePort(ONEWIRE_PORT);

static byte onewire_data[12];
static int16_t raw;
static uint8_t crc;

////////////////////////////////////////////////////////////////////////////////////////
//  ds18s20 sensors special version
float  readOneTempSensor (  uint8_t adr[8], uint8_t res) {
  float rc = NAN;

  //  reset the bus
  //  one device present?
  if ( oneWirePort.reset() ) {
    //  yes
    //  select the sensor
    oneWirePort.select(adr);
    oneWirePort.write(DS18B20_START_CONVERSION, 1);

    //  wait depending on resolution
    if      ( res == 9 )  delay(110);
    else if ( res == 10 ) delay(210);
    else if ( res == 11 ) delay(400);
    else                  delay(800);

    oneWirePort.reset();
    oneWirePort.select(adr);
    oneWirePort.write(DS18B20_READ_SCRATCHPAD, 1);

    //  read 8 data bytes
    oneWirePort.read_bytes(onewire_data, 9);

    // power down port
    oneWirePort.depower();

    crc = OneWire::crc8(onewire_data, 8) ;
    if ( crc == onewire_data[8] ) {
      raw = (onewire_data[1] << 8) | onewire_data[0];

      //(  do we have a DS18B20 =?
      if (adr[0] != 0x28 ) {
        // no
        raw = raw << 3; // 9 bit resolution default
        if (onewire_data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - onewire_data[6];
        }
      } else {
        byte cfg = (onewire_data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
      rc = ((double) raw) / 16.0;
    }
  }
  return rc;
}
