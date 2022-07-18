
//
//  Board Heltec WiFi Kit 32
//
//  https://github.com/HelTecAutomation/Heltec_ESP32
//  https://www.az-delivery.de/products/acs712-5a
//  https://www.az-delivery.de/products/analog-digitalwandler-ads1115-mit-i2c-interface
//
//  2022-06-21
//      Mode-Switch   PIN_MODE_SWITCH 18
//
//      PA Fan        PA_FAN_PIN      17
//
//
//  uploaded here   https://github.com/dl2sba/QO100_Control
//

#include <Arduino.h>
#include <PeakHold.h>
#include <heltec.h>
#include <Adafruit_ADS1X15.h>
#include "MyFonts.h"

#define BAUDRATE  115200

//===========================================================
#define ADS1115_ADR           0x48
#define ADS1115_RATE          RATE_ADS1115_860SPS
#define ADS1115_GAIN          GAIN_TWO               // 2.048V
#define ADS1115_SCALE_LNA     2850.0f
#define ADS1115_SCALE_LNB     1200.0f                 // 1235.7f
#define ADS1115_SCALE_PA      696.0f
Adafruit_ADS1115 ads1115_1;

//===========================================================
extern float readOneTempSensor( uint8_t adr[8], uint8_t res);
static uint8_t DS18B20_ADDR[8] = {
  0x28, 0xFF, 0x07, 0x22, 0xB2, 0x15, 0x01, 0xD0
};

#define CURRENT_SENSE_ZERO  15440
#define CURRENT_SENSE_SCALE 1141
#define CURRENT_SENSE_MAX   5.0

#define DISP_LINE_HEIGHT   16
#define DISP_LINE(x) ( x * DISP_LINE_HEIGHT )

//  measure voltage every 20ms
#define TICKS_VOLTS     20

// measure temperature every 2s
#define TICKS_TEMP      2000

#define PIN_MODE_SWITCH 18

#define PA_FAN_PIN      17
#define PA_FAN_ON       40.0
#define PA_FAN_OFF      37.0

#define PEAKHOLD_HOLD_TIME    40
#define PEAKHOLD_DECAY_TIME   2
#define PEAKHOLD_DECAY_STEP   200

float paTemp = 0.0;
float lnaVoltage = 0.0;
float lnbVoltage = 0.0;
float paVoltage = 0.0;
float paCurrent = 0;
char  dispBuf[32];



//=======================================
//
//  Show a short hello message on the display including timestamp of compilation
//
void showHello( void ) {
  Heltec.display->clear();
  Heltec.display->drawString(0, DISP_LINE(0), "QO-100 Base");
  Heltec.display->drawString(0, DISP_LINE(1), "Ver " __DATE__);
  Heltec.display->drawString(0, DISP_LINE(2), "    " __TIME__);
  Heltec.display->drawString(0, DISP_LINE(3), "(c) DL2SBA");
  Heltec.display->display();
  delay(2000);
}


//=======================================
//
//  Init the fan control
//
//  turn on the fan for 5s
//
void initFan( void ) {
  pinMode(PA_FAN_PIN, OUTPUT);
  //  move fan for 3s
  Heltec.display->clear();
  Heltec.display->drawString(0, DISP_LINE(0), "QO-100 Base");
  Heltec.display->drawString(0, DISP_LINE(1), "Testing fan");
  sprintf(dispBuf, "Fan on  %4.1f°", PA_FAN_ON );
  Heltec.display->drawString(0, DISP_LINE(2), dispBuf);
  sprintf(dispBuf, "Fan off %4.1f°", PA_FAN_OFF );
  Heltec.display->drawString(0, DISP_LINE(3), dispBuf);
  Heltec.display->display();
  digitalWrite(PA_FAN_PIN, HIGH);
  delay(5000);
  digitalWrite(PA_FAN_PIN, LOW);
}

//=======================================
//
//  setup the device
//
//  set serial port
//
//  init display
//
//  init ADC
void setup(void) {
  Serial.begin(BAUDRATE);
  while (!Serial);

  pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);

  Heltec.begin(true, false, true);
  Heltec.display->setFont(Nimbus_Mono_L_Regular_14);

  Wire.begin(SDA_OLED, SCL_OLED);

  ads1115_1.begin(ADS1115_ADR);
  ads1115_1.setDataRate(ADS1115_RATE);
  ads1115_1.setGain(ADS1115_GAIN);

  showHello();

  initFan();


}

//=======================================
//
//   Convert the 16-bit unsigned ADC value in a float value
//
//  raw  ADC value
//  scale
//  offset
//
float scaleRaw(uint16_t raw, float scale, uint16_t offset = 0) {
  raw -= offset;
  if ( raw > 0xff00) {
    raw = 0;
  }
  return raw / scale;
}


//=======================================
//
//  read all the relevant ADC channels
//
//  Voltage channels are read every TICKS_VOLTS ms
// 
void processADChannels(uint32_t currTicks) {
  static uint32_t ticksVolts = millis();
  static MaximumHold paCurrentmaxHold(PEAKHOLD_HOLD_TIME, PEAKHOLD_DECAY_TIME, PEAKHOLD_DECAY_STEP);
  static uint16_t currentRaw;

  //  ticks exceeded for next measurement?
  if ( currTicks > ticksVolts + TICKS_VOLTS) {
    //  yes
    //  remind timestamp
    ticksVolts = currTicks;

    currentRaw = ads1115_1.readADC_SingleEnded(0);
    paCurrentmaxHold.consume(currentRaw);

    lnaVoltage = scaleRaw(ads1115_1.readADC_SingleEnded(3), ADS1115_SCALE_LNA);
    lnbVoltage = scaleRaw(ads1115_1.readADC_SingleEnded(2), ADS1115_SCALE_LNB);
    paVoltage = scaleRaw(ads1115_1.readADC_SingleEnded(1), ADS1115_SCALE_PA);
    paCurrent = scaleRaw(paCurrentmaxHold.getPeak(), CURRENT_SENSE_SCALE, CURRENT_SENSE_ZERO);
  }


}

//=======================================
//
//  Temp channel is read every TICKS_TEMP ms
// 
void processTempSensor(uint32_t currTicks) {
  static uint32_t ticksTemp = millis();
  if ( currTicks > ticksTemp + TICKS_TEMP ) {
    ticksTemp = currTicks;
    paTemp = readOneTempSensor(DS18B20_ADDR, 9);
  }
}


//=======================================
//
//  main loop
//
//    read the ADC channels
//    read the temperature
//    control the fan based on the temperature
//      on if temp > PA_FAN_ON
//      off if temp < PA_FAN_OFF
//      manula override by switch on front panel
//  show the data on the display
//
void loop( void) {
  uint32_t currTicks = millis();

  Heltec.display->clear();

  processTempSensor(currTicks);
  processADChannels(currTicks);

  if ((!digitalRead(PIN_MODE_SWITCH)) || ( paTemp > PA_FAN_ON )) {
    digitalWrite(PA_FAN_PIN, HIGH);
  } else if (paTemp < PA_FAN_OFF ) {
    digitalWrite(PA_FAN_PIN, LOW);
  }

  sprintf(dispBuf, "Input.....%4.1fV", lnbVoltage);
  Heltec.display->drawString(0, DISP_LINE(0), dispBuf);

  sprintf(dispBuf, "%4.2fA", paCurrent);
  Heltec.display->drawString(88, DISP_LINE(1), dispBuf);

  Heltec.display->drawProgressBar(10, DISP_LINE(1) + (DISP_LINE_HEIGHT / 2) - 2, 70, 10, (paCurrent / 5.0 ) * 100);

  sprintf(dispBuf, "%3.1f°", paTemp);
  Heltec.display->drawString(88, DISP_LINE(2), dispBuf);

  sprintf(dispBuf, "PA...%4.1fV", paVoltage);
  Heltec.display->drawString(0, DISP_LINE(2), dispBuf);

  sprintf(dispBuf, "Ctrl+LNA..%4.1fV", lnaVoltage);
  Heltec.display->drawString(0, DISP_LINE(3), dispBuf);

  Heltec.display->display();
}
