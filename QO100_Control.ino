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
#include <Arduino.h>
#include <heltec.h>
#include <Adafruit_ADS1X15.h>
#include "MyFonts.h"

#define BAUDRATE  115200

//===========================================================
#define ADS1115_ADR           0x48
#define ADS1115_RATE          RATE_ADS1115_860SPS
#define ADS1115_GAIN          GAIN_TWO               // 2.048V
#define ADS1115_SCALE_LNA     2850.0f
#define ADS1115_SCALE_LNB     1235.7f
#define ADS1115_SCALE_PA      696.0f
Adafruit_ADS1115 ads1115_1;

extern float readOneTempSensor( uint8_t adr[8], uint8_t res);

static uint8_t DS18B20_ADDR[8] = {
  0x28, 0xFF, 0x07, 0x22, 0xB2, 0x15, 0x01, 0xD0
};

#define CURRENT_SENSE_ZERO  15440
#define CURRENT_SENSE_SCALE 1141
#define CURRENT_SENSE_MAX   5.0

#define LIFESTATE_X  120
#define LIFESTATE_Y  48
#define DISP_LINE_HEIGHT   16
#define DISP_LINE(x) ( x * DISP_LINE_HEIGHT )

#define TICKS_VOLTS     20
#define TICKS_TEMP      2000

#define PIN_MODE_SWITCH 18

#define PA_FAN_PIN      17
#define PA_FAN_ON       40.0
#define PA_FAN_OFF      37.0


/**
   https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Exponential%20Moving%20Average/C++Implementation.html#arduino-example
*/
template <uint8_t K, class uint_t = uint16_t>
class EMA {
  public:
    /// Update the filter with the given input and return the filtered output.
    uint_t operator()(uint_t input) {
      state += input;
      uint_t output = (state + half) >> K;
      state -= output;
      return output;
    }

    static_assert(
      uint_t(0) < uint_t(-1),  // Check that `uint_t` is an unsigned type
      "The `uint_t` type should be an unsigned integer, otherwise, "
      "the division using bit shifts is invalid.");

    /// Fixed point representation of one half, used for rounding.
    constexpr static uint_t half = 1 << (K - 1);

  private:
    uint_t state = 0;
};


/**
    https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Simple%20Moving%20Average/C++Implementation.html
*/
template <uint8_t N, class input_t = uint16_t, class sum_t = uint32_t>
class SMA {
  public:
    input_t operator()(input_t input) {
      sum -= previousInputs[index];
      sum += input;
      previousInputs[index] = input;
      if (++index == N)
        index = 0;
      return (sum + (N / 2)) / N;
    }

    static_assert(
      sum_t(0) < sum_t(-1),  // Check that `sum_t` is an unsigned type
      "Error: sum data type should be an unsigned integer, otherwise, "
      "the rounding operation in the return statement is invalid.");

  private:
    uint8_t index             = 0;
    input_t previousInputs[N] = {};
    sum_t sum                 = 0;
};


/**

*/
void showHello( void ) {
  Heltec.display->clear();
  Heltec.display->drawString(0, DISP_LINE(0), "QO-100 Base");
  Heltec.display->drawString(0, DISP_LINE(1), "Ver " __DATE__);
  Heltec.display->drawString(0, DISP_LINE(2), "    " __TIME__);
  Heltec.display->drawString(0, DISP_LINE(3), "(c) DL2SBA");
  Heltec.display->display();
  delay(2000);
}

/**

*/
void setup(void) {
  Serial.begin(BAUDRATE);
  while (!Serial);

  pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);
  pinMode(PA_FAN_PIN, OUTPUT);

  Heltec.begin(true, false, true);
  Heltec.display->setFont(Nimbus_Mono_L_Regular_14);

  Wire.begin(SDA_OLED, SCL_OLED);

  ads1115_1.begin(ADS1115_ADR);
  ads1115_1.setDataRate(ADS1115_RATE);
  ads1115_1.setGain(ADS1115_GAIN);

  showHello();
}

/**

*/
float scaleRaw(uint16_t raw, float scale, uint16_t offset = 0) {
  raw -= offset;
  if ( raw > 0xff00) {
    raw = 0;
  }
  return raw / scale;
}

/**

*/
void showLifesign(void) {
  static uint8_t lifeState = 0;
  Heltec.display->drawString(LIFESTATE_X, LIFESTATE_Y, " ");
  switch (lifeState) {
    case 0:
      Heltec.display->drawString(LIFESTATE_X, LIFESTATE_Y, "|");
      lifeState = 1;
      break;

    case 1:
      Heltec.display->drawString(LIFESTATE_X, LIFESTATE_Y, "/");
      lifeState = 2;
      break;

    case 2:
      Heltec.display->drawString(LIFESTATE_X, LIFESTATE_Y, "-");
      lifeState = 3;
      break;

    default:
      Heltec.display->drawString(LIFESTATE_X, LIFESTATE_Y, "\\");
      lifeState = 0;
      break;
  }
}

void loop( void ) {
  if ( digitalRead(PIN_MODE_SWITCH) ) {
    loop1(TICKS_TEMP, TICKS_VOLTS);
  } else {
    loopFast(TICKS_TEMP, 1);
  }
}

/*

*/
void loop1( uint32_t pTicksTemp, uint32_t pTicksVolt) {
  static char dispBuf[32];
  static SMA<20> currentSMA;
  static uint32_t ticksTemp = millis();
  static uint32_t ticksVolts = millis();
  uint32_t currTicks = millis();
  static uint16_t currentRaw;
  static float paTemp = 0.0;
  static float lna = 0.0;
  static float lnb = 0.0;
  static float pa = 0.0;
  static float current = 0;

  Heltec.display->clear();

  if ( currTicks > ticksTemp + pTicksTemp ) {
    ticksTemp = currTicks;
    paTemp = readOneTempSensor(DS18B20_ADDR, 9);
  }

  if ( currTicks > ticksVolts + pTicksVolt) {
    ticksVolts = currTicks;
    lna = scaleRaw(ads1115_1.readADC_SingleEnded(3), ADS1115_SCALE_LNA);
    lnb = scaleRaw(ads1115_1.readADC_SingleEnded(2), ADS1115_SCALE_LNB);
    pa = scaleRaw(ads1115_1.readADC_SingleEnded(1), ADS1115_SCALE_PA);
    currentRaw = currentSMA(ads1115_1.readADC_SingleEnded(0));
    current = scaleRaw(currentRaw, CURRENT_SENSE_SCALE, CURRENT_SENSE_ZERO);
  }

  sprintf(dispBuf, "Input.....%4.1fV", lnb);
  Heltec.display->drawString(0, DISP_LINE(0), dispBuf);

  sprintf(dispBuf, "%4.2fA", current);
  Heltec.display->drawString(88, DISP_LINE(1), dispBuf);

  //  sprintf(dispBuf, "%04x", currentRaw);
  //  Heltec.display->drawString(0, DISP_LINE(1), dispBuf);

  Heltec.display->drawProgressBar(10, DISP_LINE(1) + (DISP_LINE_HEIGHT / 2) - 2, 70, 10, (current / 5.0 ) * 100);

  sprintf(dispBuf, "%3.1fC", paTemp);
  Heltec.display->drawString(88, DISP_LINE(2), dispBuf);

  sprintf(dispBuf, "PA...%4.1fV", pa);
  Heltec.display->drawString(0, DISP_LINE(2), dispBuf);

  sprintf(dispBuf, "Ctrl+LNA..%4.1fV", lna);
  Heltec.display->drawString(0, DISP_LINE(3), dispBuf);

  showLifesign();

  Heltec.display->display();
}

/*

*/
void loopFast( uint32_t pTicksTemp, uint32_t pTicksVolt) {
  static char dispBuf[32];
  static SMA<10> currentSMA;
  static uint32_t ticksTemp = millis();
  static uint32_t ticksVolts = millis();
  uint32_t currTicks = millis();
  static uint16_t currentRaw;
  static float paTemp = 0.0;
  static float current = 0;
  uint8_t updateDisplay = false;


  if ( currTicks > ticksTemp + pTicksTemp ) {
    ticksTemp = currTicks;
    paTemp = readOneTempSensor(DS18B20_ADDR, 9);
    updateDisplay = true;
  }

  if ( paTemp > PA_FAN_ON ) {
    digitalWrite(PA_FAN_PIN, HIGH);
  } else if (paTemp < PA_FAN_OFF ) {
    digitalWrite(PA_FAN_PIN, LOW);
  }

  if ( currTicks > ticksVolts + pTicksVolt) {
    ticksVolts = currTicks;
    currentRaw = currentSMA(ads1115_1.readADC_SingleEnded(0));
    current = scaleRaw(currentRaw, CURRENT_SENSE_SCALE, CURRENT_SENSE_ZERO);
    updateDisplay = true;
  }

  if ( updateDisplay) {
    Heltec.display->clear();
    sprintf(dispBuf, "Current... %4.2fA", current);
    Heltec.display->drawString(0, DISP_LINE(0), dispBuf);

    sprintf(dispBuf, "PA Temp... %3.1fC", paTemp);
    Heltec.display->drawString(0, DISP_LINE(1), dispBuf);

    Heltec.display->drawProgressBar(0, DISP_LINE(2) + (DISP_LINE_HEIGHT / 2) - 2, 126, 20, (current / 5.0 ) * 100);

    Heltec.display->display();
  }
}
