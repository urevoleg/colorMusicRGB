/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
#include <GyverRGB.h>
GRGB strip(3, 5, 6);  // куда подключены цвета (R, G, B)

#define COLOR_WHEEL_PERIOD 5000000
#define COLOR_WHEEL_UPDATE_TIME int(COLOR_WHEEL_PERIOD / 1530)

unsigned long last_update_color_wheel = 0;
int color_wheel_value = 0;

#define MAX_VALUE_UPDATE_TIME 10
unsigned long last_update_max_value = 0;

#define AUDIO_PIN A1

#define SAMPLES 250

#define USE_ENCODER 0 // управление при помощи энкодера
#define USE_BTN 0     // управление при помощи кнопки

#ifdef USE_ENCODER
#define BTN_PIN 10
#define CLK 4
#define DT 9
#include "GyverEncoder.h"
Encoder enc1(CLK, DT, BTN_PIN);
#endif

#ifdef USE_BTN
#define BTN_PIN 10
#endif

//----- OLED ------
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

SSD1306AsciiWire oled;


//############### ПАРАМЕТРЫ РЕЖИМА audioRGB #######################
/*
   Параметры сглаживания средней громкости
*/
#define SMOOTH_AVG_LEVEL 0.01
float avgLevel = 1;

/*
   Параметра сглаживания глобальных максимумов
*/
#define SMOOTH_GLOB_MAX 0.01
float avgLevelMax = 1;

/*
   Параметры сглаживания масштабированного уровня громкости
*/
#define SMOOTH_SCALE_LEVEL 0.15
float avgScaleLevel = 1;

#define LOW_BRIGHTNESS 10 // LOW_BRIGTHNESS применять к avgLevel
#define LOW_LEVEL 80     // нижний порог входного сигнала

int globMax = 1;
int thisMax = 1;
unsigned long globMaxTimer;
//###################################################################
//############### ПАРАМЕТРЫ РЕЖИМА colorWheel #######################
int valueColorWheelMode = 0;
unsigned long timeColorWheelMode = 0;
float periodColorWheel = 10.0;                                                // период всего цветового круга * 10^6 мкс - 1 сек по дефолту
unsigned long speedColorWheelMode = long(1000000 * periodColorWheel / 1530);  // длительность смены цвета для достижения нужного периода
unsigned int minSpeedColorWheelMode = 650;                                    // примерно 1сек период
unsigned int maxSpeedColorWheelMode = 20000;                                  // около 30 сек
int thisBrightness = 255;                                            // яркость по-умолчанию
//###################################################################
//############## РЕЖИМ FIRE #########################################
unsigned long prevTimeFire = 0;
unsigned long updateTimeFire = 100;
float SMOOTH_FIRE = 0.01;
int MIN_BRIGHTNESS_FIRE = 55;
int MAX_BRIGHTNESS_FIRE = 75;
int AVG_BRIGHTNESS_FIRE = 125;
float val = 10;
int thisVal = 0;
//###################################################################

#define LOG_OUTPUT 0  // включить вывод лога в Serial port
//######### Переменные #######
int modeNum = 1;        // номер режима
int modeNumCounts = 3;  // кол-во режимов

float tmp_smooth = 1500;

unsigned long lastRiseLevel = 0;
float period_smooth = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

#ifdef USE_ENCODER
  enc1.setType(TYPE1);
  enc1.setFastTimeout(25);
#endif
  strip.setDirection(REVERSE);
  strip.setBrightness(thisBrightness);

  // 1 уставка внутреннего опорного
  analogReference(INTERNAL);  // внутреннее опорное напряжение 1.1B

  // ----- OLED --------
  Wire.begin();
  Wire.setClock(400000L);

  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.displayRemap(true);
  oled.setFont(lcd5x7);

  drawInfo();

  // Изменяем частоту АЦП --> 250кГц
  bitSet(ADCSRA, ADPS2);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);

  // подождем для устаканивания переходных процессов
  delay(2000);
}


void loop() {

  control();  // опрос элементов управления и изменение настроек
  effects();  // отображение выбранного режима

}

void control() {
  // обязательная функция отработки энкодера
#ifdef USE_ENCODER
  enc1.tick();

  if (enc1.isDouble()) {
    modeNum++;
    if (modeNum > modeNumCounts) {
      modeNum = 1;
    }
    drawInfo();
    Serial.print("modeNum: "); Serial.println(modeNum);
  }

  // если поворачивается энкодер
  if (enc1.isTurn()) {

    drawInfo();

    switch (modeNum) {
      case 1: // аудио режим, пока настроек здесь нет
        if (enc1.isRight()) thisBrightness += 5;    // если было удержание + поворот направо, увеличиваем на 5
        if (enc1.isLeft()) thisBrightness -= 5;     // если было удержание + поворот налево, уменьшаем на 5

        if (thisBrightness > 255) {
          thisBrightness = 255;
        }

        if (thisBrightness < 0) {
          thisBrightness = 0;
        }
        Serial.print("brightness: "); Serial.println(thisBrightness);
        break;

      case 2: // режим цветового круга, настройка скорости

        if (enc1.isRight()) periodColorWheel += 0.1;      // если был поворот направо, увеличиваем на 1
        if (enc1.isLeft()) periodColorWheel -= 0.1;       // если был поворот налево, уменьшаем на 1

        if (enc1.isFastR()) periodColorWheel += 0.1;    // если был быстрый поворот направо, увеличиваем на 10
        if (enc1.isFastL()) periodColorWheel -= 0.1;    // если был быстрый поворот налево, уменьшаем на 10

        speedColorWheelMode = long(1000000 * periodColorWheel / 1530);

        if (speedColorWheelMode > maxSpeedColorWheelMode) {
          speedColorWheelMode = maxSpeedColorWheelMode;
        }

        if (speedColorWheelMode < minSpeedColorWheelMode) {
          speedColorWheelMode = minSpeedColorWheelMode;
        }

        if (enc1.isRightH()) thisBrightness += 5;    // если было удержание + поворот направо, увеличиваем на 5
        if (enc1.isLeftH()) thisBrightness -= 5;     // если было удержание + поворот налево, уменьшаем на 5

        if (thisBrightness > 255) {
          thisBrightness = 255;
        }

        if (thisBrightness < 0) {
          thisBrightness = 0;
        }
        Serial.print("brightness: "); Serial.println(thisBrightness);
        Serial.print("periodColorWheel: "); Serial.println(periodColorWheel);
        break;

      case 3: // режим огня
        if (enc1.isRight()) MAX_BRIGHTNESS_FIRE += 5;    // если было удержание + поворот направо, увеличиваем на 5
        if (enc1.isLeft()) MAX_BRIGHTNESS_FIRE -= 5;     // если было удержание + поворот налево, уменьшаем на 5
        if (MAX_BRIGHTNESS_FIRE < MIN_BRIGHTNESS_FIRE + 5) {
          MAX_BRIGHTNESS_FIRE = MIN_BRIGHTNESS_FIRE + 5;
        }
        if (MAX_BRIGHTNESS_FIRE > 255) {
          MAX_BRIGHTNESS_FIRE = 255;
        }
        Serial.print("MAX_BRIGHTNESS_FIRE: "); Serial.println(MAX_BRIGHTNESS_FIRE);

        break;
    }
  }


#endif
}

void drawInfo() {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("<M>");
  oled.setCursor(20, 0);
  oled.print(modeNum);

  oled.setCursor(0, 2);
  oled.print("<B>");
  oled.setCursor(20, 2);
  oled.print(thisBrightness);
}

void effects() {
  switch (modeNum) {
    case 1:
      audioMode();
      break;

    case 2:
      colorWheelMode();
      break;

    case 3:
      fireTick();
      break;
  }
}

void fireTick() {
  int rndVal;
  if (millis() - prevTimeFire > updateTimeFire) {
    rndVal = random(3, 7) * 10;
    prevTimeFire = millis();
  }
  val = val * (1 - SMOOTH_FIRE) + rndVal * SMOOTH_FIRE;
  strip.colorWheel(val);
  thisBrightness = map(val, 30, 70, MIN_BRIGHTNESS_FIRE, MAX_BRIGHTNESS_FIRE);
  strip.setBrightness(thisBrightness);
}

void colorWheelMode() {
  // режим цветового круга
  if (micros() - timeColorWheelMode > long(1000000 * periodColorWheel / 1530)) {
    valueColorWheelMode++;
    if (valueColorWheelMode > 1530) {
      valueColorWheelMode = 0;
    }
    strip.setBrightness(thisBrightness);
    strip.colorWheel(valueColorWheelMode);
    timeColorWheelMode = micros();
  }
}

void audioMode() {

  if (millis() - last_update_max_value > MAX_VALUE_UPDATE_TIME) {
    thisMax = 0;
    for (int i = 0; i < SAMPLES; i++) {
      int adc_value = analogRead(AUDIO_PIN);
      // поиск максимума
      if (adc_value > thisMax) {
        thisMax = adc_value;
      }
    }

    // глобальный максимум максимумов
    if (thisMax > globMax) {
      globMax = thisMax;
    }

    // фильтр для сглаживания максимума
    avgLevelMax +=  (globMax - avgLevelMax) * SMOOTH_GLOB_MAX;

    // медленный фильтр для определения средней громкости
    avgLevel +=  (thisMax - avgLevel) * SMOOTH_AVG_LEVEL;

    // изменяем яркость
    // масштабируем мгновенный максимум от общей средней громкости и общего среднего максимума к
    // диапазону ШИМ LOW_BRIGHTNESS-thisBrightness
    int scaleLevel = constrain(map(thisMax, avgLevel, avgLevelMax, LOW_BRIGHTNESS, thisBrightness), LOW_BRIGHTNESS, thisBrightness);

    // сгладим этот полученный уровень
    avgScaleLevel +=  (scaleLevel - avgScaleLevel) * SMOOTH_SCALE_LEVEL;

    // если текущий уровень аудио ниже порога, то установить LOW_BRIGHTNESS
    if (avgLevel < LOW_LEVEL) {
      avgScaleLevel = LOW_BRIGHTNESS;

      // возвращаем период цветового круга по дефолту
      period_smooth = COLOR_WHEEL_PERIOD / 1000;
    }

    strip.setBrightness(avgScaleLevel);

    // вычисляем средний уровень ШИМ
    tmp_smooth +=  (avgScaleLevel - tmp_smooth) * 0.005;

    // смотрим превыщение текущего уровня над средним и запоминаем момент если превысили
    if (avgScaleLevel > tmp_smooth ) {
      lastRiseLevel = millis();
    }

    // вычисляем период таких превыщений
    unsigned long period = (millis() -  lastRiseLevel) * 10;
    if (period < 1000) {
      period = 1000;
    }
    if (period > 7500) {
      period = 7500;
    }

    // сглаживаем период, его будем использовать для цветового круга
    period_smooth +=  (period - period_smooth) * 0.025;

    if (0) {
      Serial.print(globMax); Serial.print(" ");
      Serial.print(avgLevelMax); Serial.print(" ");
      Serial.print(avgLevel); Serial.print(" ");
      Serial.println(avgScaleLevel);
    }

    last_update_max_value = millis();
  }

  // радуга поверх всей ленты
  if (micros() - last_update_color_wheel > int(1000.0 * period_smooth / 1530)) {
    color_wheel_value++;
    if (color_wheel_value > 1530) color_wheel_value = 0;
    strip.colorWheel(color_wheel_value);
    last_update_color_wheel = micros();
  }

  // сброс глобального максимума
  if (millis() - globMaxTimer > 1500) {      // каждые 1500 мс
    // обновляем глобальный максимум общим средним уровнем громкости
    globMax = avgLevel;
    globMaxTimer = millis();
  }
}
