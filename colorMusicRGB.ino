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

#define LOW_BRIGHTNESS 10

int globMax;
int thisMax;
unsigned long globMaxTimer;

#define LOG_OUTPUT 0  // включить вывод лога в Serial port


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  strip.setDirection(REVERSE);
  strip.setBrightness(0);

  // 1 уставка внутреннего опорного
  analogReference(INTERNAL);  // внутреннее опорное напряжение 1.1B

  // Изменяем частоту АЦП --> 250кГц
  bitSet(ADCSRA, ADPS2);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);
}

// the loop routine runs over and over again forever:
void loop() {

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
    // диапазону ШИМ 0-255
    int scaleLevel = constrain(map(thisMax, avgLevel, avgLevelMax, 0, 255), 0, 255);

    // сгладим этот полученный уровень
    avgScaleLevel +=  (scaleLevel - avgScaleLevel) * SMOOTH_SCALE_LEVEL;

    // если текущий уровень якрости выше нижнего порога
    if (avgScaleLevel > LOW_BRIGHTNESS) strip.setBrightness(avgScaleLevel);
    // если ниже, установить яркость нижнего порога
    else strip.setBrightness(LOW_BRIGHTNESS);


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
