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
#define LOW_LEVEL 100     // нижний порог входного сигнала

int globMax;
int thisMax;
unsigned long globMaxTimer;
//###################################################################
//############### ПАРАМЕТРЫ РЕЖИМА colorWheel #######################
int valueColorWheelMode = 0;
unsigned long timeColorWheelMode = 0;
long speedColorWheelMode = 0;
unsigned int minSpeedColorWheelMode = 100;
unsigned int brightnessRGB = 128;
//###################################################################

#define LOG_OUTPUT 0  // включить вывод лога в Serial port
//######### Переменные #######
int modeNum = 1;        // номер режима
int modeNumCounts = 2;  // кол-во режимов

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

#ifdef USE_ENCODER
  enc1.setType(TYPE1);
  enc1.setFastTimeout(40);
#endif
  strip.setDirection(REVERSE);
  strip.setBrightness(0);

  // 1 уставка внутреннего опорного
  analogReference(INTERNAL);  // внутреннее опорное напряжение 1.1B

  // Изменяем частоту АЦП --> 250кГц
  bitSet(ADCSRA, ADPS2);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);
}


void loop() {

  control();  // опрос элементов управления и изменение настроек
  effects();  // отображение выбранного режима

}

void control() {
  // обязательная функция отработки энкодера
#ifdef USE_ENCODER
  enc1.tick();

  if (enc1.isClick()) {
    modeNum++;
    if (modeNum > modeNumCounts) {
      modeNum = 1;
    }
    Serial.print("modeNum: "); Serial.println(modeNum);
  }

  // если поворачивается энкодер
  if (enc1.isTurn()) {
    switch (modeNum) {
      case 1: // аудио режим, пока настроек здесь нет
        if (enc1.isRightH()) brightnessRGB += 5;    // если было удержание + поворот направо, увеличиваем на 5
        if (enc1.isLeftH()) brightnessRGB -= 5;     // если было удержание + поворот налево, уменьшаем на 5

        if (brightnessRGB > 255) {
          brightnessRGB = 255;
        }

        if (brightnessRGB < 0) {
          brightnessRGB = 0;
        }
        break;

      case 2: // режим цветового круга, настройка скорости

        if (enc1.isRight()) speedColorWheelMode++;        // если был поворот направо, увеличиваем на 1
        if (enc1.isLeft()) speedColorWheelMode--;         // если был поворот налево, уменьшаем на 1

        if (enc1.isFastR()) speedColorWheelMode += 10;    // если был быстрый поворот направо, увеличиваем на 10
        if (enc1.isFastL()) speedColorWheelMode -= 10;    // если был быстрый поворот налево, уменьшаем на 10

        if (speedColorWheelMode > 5000) {
          speedColorWheelMode = 5000;
        }

        if (speedColorWheelMode < minSpeedColorWheelMode) {
          speedColorWheelMode = minSpeedColorWheelMode;
        }

        if (enc1.isRightH()) brightnessRGB += 5;    // если было удержание + поворот направо, увеличиваем на 5
        if (enc1.isLeftH()) brightnessRGB -= 5;     // если было удержание + поворот налево, уменьшаем на 5

        if (brightnessRGB > 255) {
          brightnessRGB = 255;
        }

        if (brightnessRGB < 0) {
          brightnessRGB = 0;
        }
        Serial.print("brightnessColorWheelMode: "); Serial.println(brightnessRGB);
        Serial.print("speedColorWheelMode: "); Serial.println(speedColorWheelMode);
        break;
    }
  }


#endif
}

void effects() {
  switch (modeNum) {
    case 1:
      audioMode();
      break;

    case 2:
      colorWheelMode();
      break;
  }
}

void colorWheelMode() {
  // режим цветового круга
  if (micros() - timeColorWheelMode > speedColorWheelMode) {
    valueColorWheelMode++;
    if (valueColorWheelMode > 1530) {
      valueColorWheelMode = 0;
    }
    strip.setBrightness(brightnessRGB);
    strip.colorWheel(valueColorWheelMode);
    timeColorWheelMode = micros();
  }
}

void audioMode() {

  if (millis() - last_update_max_value > MAX_VALUE_UPDATE_TIME) {
    thisMax = 0;
    for (int i = 0; i < SAMPLES; i++) {
      int adc_value = analogRead(A0);
      // поиск максимума
      if (adc_value > thisMax) {
        thisMax = adc_value;
      }
    }

    // глобальный максимум максимумов
    if (thisMax > globMax) globMax = thisMax;

    // фильтр для сглаживания максимума
    avgLevelMax +=  (globMax - avgLevelMax) * SMOOTH_GLOB_MAX;

    // медленный фильтр для определения средней громкости
    avgLevel +=  (thisMax - avgLevel) * SMOOTH_AVG_LEVEL;

    // изменяем яркость
    // масштабируем мгновенный максимум от общей средней громкости и общего среднего максимума к
    // диапазону ШИМ 0-brightnessRGB
    int scaleLevel = constrain(map(thisMax, avgLevel, avgLevelMax, 0, brightnessRGB), 0, brightnessRGB);

    // сгладим этот полученный уровень
    avgScaleLevel +=  (scaleLevel - avgScaleLevel) * SMOOTH_SCALE_LEVEL;

    // если текущий уровень аудио ниже порога, то установить LOW_BRIGHTNESS
    if (avgLevel < LOW_LEVEL) {
      avgScaleLevel = LOW_BRIGHTNESS;
    }

    strip.setBrightness(avgScaleLevel);

    if (LOG_OUTPUT) {
      Serial.print(globMax); Serial.print(" ");
      Serial.print(avgLevelMax); Serial.print(" ");
      Serial.print(avgLevel); Serial.print(" ");
      Serial.println(avgScaleLevel);
    }

    last_update_max_value = millis();
  }

  // радуга поверх всей ленты
  if (micros() - last_update_color_wheel > COLOR_WHEEL_UPDATE_TIME) {
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
