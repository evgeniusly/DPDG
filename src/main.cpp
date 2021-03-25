#include <SPI.h>
//Add the SdFat Libraries
#include <SdFat.h>
#include <SdFatUtil.h>
//and the MP3 Shield Library
#include <SFEMP3Shield.h>

// ============================================================
// SETTINGS
// ============================================================

#define MICROLED 1

const uint8_t PIN_LED = 13;
const uint8_t PIN_VIBRO_LEFT = 11;
const uint8_t PIN_VIBRO_RIGHT = 12;

const uint8_t SENSOR_0 = A0;
const uint8_t SENSOR_1 = A1;
const uint8_t SENSOR_2 = A2;
const uint8_t SENSOR_3 = A3;
const uint8_t SENSOR_4 = A4;

const int LEDS_COUNT = 144;
const int LEDS_TAIL_MIN = 0;
const int LEDS_TAIL_MAX = LEDS_COUNT / 3;

const double ITERATION_PER_MINUTE_MIN = 1;
const double ITERATION_PER_MINUTE_MAX = 80;

const double SOUND_VOLUME_MIN = 60; // input values are -1/2dB. e.g. 40 results in -20dB.
const double SOUND_VOLUME_MAX = 0;  // input values are -1/2dB. e.g. 40 results in -20dB.

const double VIBRO_PWR_MIN = 0;
const double VIBRO_PWR_MAX = 200;

// const int FRAME_RATE = 300;

const int SENSOR_MAX_VAL = 1023;

// ============================================================
// VARIABLES
// ============================================================

// CRGB leds[LEDS_COUNT];
int led_id_current = 0;
int led_id_previous = -1;
int led_color_hue = 0;
int led_brightness = 100;
int led_tail = 200;
unsigned long time_from_start = 0;
unsigned long time_from_start_prev = 0;
int time_from_last_tick = 0;
double iteration_per_minute = 30;
double iteration_progress_delta = 0;
double iteration_progress = 0;
double space_position = 0;
double sound_volume_coef = 1;
float foo = 0;

double dot_whide = 0.02;

SdFat sd;
SFEMP3Shield MP3player;
uint8_t result;

#if defined(MICROLED)
// ===== ЦВЕТОВАЯ ГЛУБИНА =====
// 1, 2, 3 (байт на цвет)
// на меньшем цветовом разрешении скетч будет занимать в разы меньше места,
// но уменьшится и количество оттенков и уровней яркости!
// дефайн делается ДО ПОДКЛЮЧЕНИЯ БИБЛИОТЕКИ
// без него будет 3 байта по умолчанию
#define COLOR_DEBTH 3
#include <microLED.h> // подключаем библу
microLED<LEDS_COUNT, PIN_LED, -1, LED_WS2812, ORDER_GRB, CLI_AVER> strip;

#else
#include "FastLED.h"
CRGB leds[LEDS_COUNT];

#endif

// ============================================================
// HELPERS
// ============================================================

// convert double `x` in range from [`in_min`, `in_max`] to double [`out_min`, `out_max`]
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============================================================
// FUNCS
// ============================================================

void updateControledParams()
{
  int sensor_0 = analogRead(SENSOR_0);
  int sensor_1 = analogRead(SENSOR_1);
  int sensor_2 = analogRead(SENSOR_2);
  int sensor_3 = analogRead(SENSOR_3);
  int sensor_4 = analogRead(SENSOR_4);

  iteration_per_minute = mapDouble(sensor_0, 0, SENSOR_MAX_VAL, ITERATION_PER_MINUTE_MAX, ITERATION_PER_MINUTE_MIN);
  led_color_hue = map(sensor_1, 0, SENSOR_MAX_VAL, 255, 0);
  led_brightness = map(sensor_2, 0, SENSOR_MAX_VAL, 255, 0);
  led_tail = map(sensor_3, 0, SENSOR_MAX_VAL, LEDS_TAIL_MAX, LEDS_TAIL_MIN);
  sound_volume_coef = mapDouble(sensor_4, 0, SENSOR_MAX_VAL, 0, 100);
  // Serial.println(sound_volume_coef);
}

// update current iteration progress
// based on time from last tick
// set double `iteration_progress` => [0, 1)
void updateIterationProgress()
{
  time_from_start = millis();
  time_from_last_tick = time_from_start - time_from_start_prev; // [0, ...] ms
  time_from_start_prev = time_from_start;

  iteration_progress_delta = (double)time_from_last_tick / (60000 / iteration_per_minute);
  // if (iteration_progress_delta > 1) {
  //   // iteration_progress_delta -= floor(iteration_progress_delta); // [0, 1]
  //   iteration_progress_delta = modff(iteration_progress_delta); // [0, 1]
  // }

  iteration_progress += iteration_progress_delta;
  // iteration_progress -= floor(iteration_progress); // [0, 1]
  iteration_progress = modff(iteration_progress, &foo); // [0, 1)
}

// update current space position
// based on `iteration_progress`
// set double `space_position` => [0, 1]
void updateSpacePosition()
{
  space_position = (cos(2 * PI * iteration_progress) + 1) / 2; // [0, 1]
  // Serial.println(String(iteration_progress) + " " + String(space_position));
}

#if defined(MICROLED)
// LED display update magic
void proceedLEDmicroLED()
{
  led_id_current = int(round((LEDS_COUNT - 1) * space_position));

  if (led_id_current != led_id_previous)
  {
    strip.clear();
    strip.setBrightness(led_brightness);

    // tail part
    int direction = led_id_current - led_id_previous;
    direction = (direction > 0) - (direction < 0); // -1 => rtl, 1 => ltr
    for (int i = 0; i < led_tail; i++)
    {
      int led_id = led_id_current - direction * (led_tail - i);
      if (led_id < 0)
      {
        led_id *= -1;
      }
      if (led_id > (LEDS_COUNT - 1))
      {
        led_id = (LEDS_COUNT - 1) - (led_id - (LEDS_COUNT - 1));
      }
      int pow = (double)255 * (i + 1) / (led_tail + 1);
      // strip.leds[led_id] = mWheel8(led_color_hue, pow);
      strip.leds[led_id] = mHSV(led_color_hue, 255, pow);
    }

    // lead dot
    // strip.leds[led_id_current] = mWheel8(led_color_hue, 255);
    strip.leds[led_id_current] = mHSV(led_color_hue, 255, 255);

    strip.show();
    led_id_previous = led_id_current;
  }
}

#else
// LED display update magic
void proceedLEDFastLED()
{
  led_id_current = int(round((LEDS_COUNT - 1) * space_position));

  if (led_id_current != led_id_previous)
  {
    FastLED.clear();
    FastLED.setBrightness(led_brightness);

    // tail part
    int direction = led_id_current - led_id_previous;
    direction = (direction > 0) - (direction < 0); // -1 => rtl, 1 => ltr
    for (int i = 0; i < led_tail; i++)
    {
      int led_id = led_id_current - direction * (led_tail - i);
      if (led_id < 0)
      {
        led_id *= -1;
      }
      if (led_id > (LEDS_COUNT - 1))
      {
        led_id = (LEDS_COUNT - 1) - (led_id - (LEDS_COUNT - 1));
      }
      int pow = (double)255 * (i + 1) / (led_tail + 1);
      leds[led_id] = CHSV(led_color_hue, 255, pow);
    }

    // lead dot
    leds[led_id_current] = CHSV(led_color_hue, 255, 255);

    FastLED.show();
    led_id_previous = led_id_current;
  }
}
#endif

// void proceedLEDByPosition()
// {
//   FastLED.setBrightness(led_brightness);
//   FastLED.clear();
//   for (int i = 0; i < LEDS_COUNT; i++)
//   {
//     double led_pos = double(i) / (LEDS_COUNT - 1);              // [0, 1]
//     double led_offset_from_dot = abs(space_position - led_pos); // [0, 1]
//     double offset_coef = min(led_offset_from_dot / dot_whide, 1);
//     double power = double(255) * (double(1) - offset_coef);
//     leds[i] = CHSV(led_color_hue, 255, max(power, 0));
//     //Serial.println(power);
//   }
//   FastLED.show();
// }

void proceedVibro()
{
  // double vibro_left_pwr = ((1 - space_position) * (255 - VIBRO_PWR_MIN)) + VIBRO_PWR_MIN;
  // double vibro_right_pwr = (space_position * (255 - VIBRO_PWR_MIN)) + VIBRO_PWR_MIN;
  double vibro_left_pwr = mapDouble(1 - space_position, 0, 1, VIBRO_PWR_MIN, VIBRO_PWR_MAX);
  double vibro_right_pwr = mapDouble(space_position, 0, 1, VIBRO_PWR_MIN, VIBRO_PWR_MAX);

  analogWrite(PIN_VIBRO_LEFT, vibro_left_pwr);
  analogWrite(PIN_VIBRO_RIGHT, vibro_right_pwr);

  // Serial.println(vibro_left_pwr);
}

void proceedSound()
{
  double sound_left_pwr = mapDouble(1 - space_position, 0, 1, SOUND_VOLUME_MIN, SOUND_VOLUME_MAX);
  double sound_right_pwr = mapDouble(space_position, 0, 1, SOUND_VOLUME_MIN, SOUND_VOLUME_MAX);
  sound_left_pwr += sound_volume_coef;
  sound_right_pwr += sound_volume_coef;

  // Serial.println(sound_left_pwr);
  MP3player.setVolume(sound_left_pwr, sound_right_pwr); // commit new volume

  if (MP3player.isPlaying() == 0)
  {
    result = MP3player.playTrack(2);
    if (result != 0)
    {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to play track"));
    }
  };
}

// ============================================================
// ARDUINO
// ============================================================

void setup()
{
  // This is the list of timers in Arduino Mega 2560
  // timer 0 (controls pin 13, 4);
  // timer 1 (controls pin 12, 11);
  // timer 2 (controls pin 10, 9);
  // timer 3 (controls pin 5, 3, 2);
  // timer 4 (controls pin 8, 7, 6);

  // Pins D9, D10 to 31.4 кГц (Arduino Mega 2560)
  // use this
  // TCCR2A = 0b00000001;  // 8bit
  // TCCR2B = 0b00000001;  // x1 phase correct

  // or use this
  // int myEraser = 7;             // this is 111 in binary and is used as an eraser
  // TCCR2B &= ~myEraser;          // this operation (AND plus NOT),  set the three bits in TCCR2B to 0

  // Those prescaler values are good for all timers (TCCR1B, TCCR2B, TCCR3B, TCCR4B) except for timer 0 (TCCR0B).
  // prescaler = 1 ---> PWM frequency is 31000 Hz
  // prescaler = 2 ---> PWM frequency is 4000 Hz
  // prescaler = 3 ---> PWM frequency is 490 Hz (default value)
  // prescaler = 4 ---> PWM frequency is 120 Hz
  // prescaler = 5 ---> PWM frequency is 30 Hz
  // prescaler = 6 ---> PWM frequency is <20 Hz
  // int myPrescaler = 1;          // this could be a number in [1 , 6].
  // TCCR2B |= myPrescaler;

  Serial.begin(9600); // set serial output speed

  // set pin as output
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_VIBRO_LEFT, OUTPUT);
  pinMode(PIN_VIBRO_RIGHT, OUTPUT);

  // LED setup
#if defined(MICROLED)
  strip.setMaxCurrent(2000);
#else
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, LEDS_COUNT);
  // FastLED.setCorrection( TypicalLEDStrip );
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);
#endif

  //Initialize the SdCard.
  if (!sd.begin(SD_SEL, SPI_FULL_SPEED))
    sd.initErrorHalt();
  // depending upon your SdCard environment, SPI_HAVE_SPEED may work better.
  if (!sd.chdir("/"))
    sd.errorHalt("sd.chdir");

  //Initialize the MP3 Player Shield
  result = MP3player.begin();
  if (result != 0)
  {
    Serial.print(F("Error code: "));
    Serial.print(result);
    Serial.println(F(" when trying to start MP3 player"));
  }
}

void loop()
{
  updateControledParams();
  updateIterationProgress();
  updateSpacePosition();

#if defined(MICROLED)
  proceedLEDmicroLED();
#else
  proceedLEDFastLED();
#endif

  // proceedLEDByPosition();

  proceedVibro();
  proceedSound();

  // delay(1000.0 / FRAME_RATE); // forced frame rate emulation
}
