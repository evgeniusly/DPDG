// #include <Arduino.h>
#include "FastLED.h"

// ============================================================
// SETTINGS
// ============================================================

const uint8_t PIN_LED = 13;
const uint8_t PIN_VIBRO_LEFT = 9;
const uint8_t PIN_VIBRO_RIGHT = 10;

const uint8_t SENSOR_0 = A0;
const uint8_t SENSOR_1 = A1;
const uint8_t SENSOR_2 = A2;
const uint8_t SENSOR_3 = A3;

const int LEDS_COUNT = 144;

const double ITERATION_PER_MINUTE_MIN = 1;
const double ITERATION_PER_MINUTE_MAX = 120;

const double VIBRO_PWR_MIN = 0;
const double VIBRO_PWR_MAX = 255;

// const int FRAME_RATE = 300;

const int SENSOR_MAX_VAL = 1023;

// ============================================================
// VARIABLES
// ============================================================

CRGB leds[LEDS_COUNT];
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
float foo = 0;

// double dot_whide = 0.02;

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

  iteration_per_minute = mapDouble(sensor_0, 0, SENSOR_MAX_VAL, ITERATION_PER_MINUTE_MAX, ITERATION_PER_MINUTE_MIN);
  led_color_hue = map(sensor_1, 0, SENSOR_MAX_VAL, 255, 0);
  led_brightness = map(sensor_2, 0, SENSOR_MAX_VAL, 255, 0);
  led_tail = map(sensor_3, 0, SENSOR_MAX_VAL, 20, 255);
  //dot_whide = sensor_3 / SENSOR_MAX_VAL;

  //Serial.println(led_tail);
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
}

// LED display update magic
void proceedLED()
{
  led_id_current = int(LEDS_COUNT * space_position);

  if (led_id_current != led_id_previous)
  {
    //FastLED.clear();
    FastLED.setBrightness(led_brightness);
    fadeToBlackBy(leds, LEDS_COUNT, led_tail);
    leds[led_id_current] = CHSV(led_color_hue, 255, 255);
    FastLED.show();
    led_id_previous = led_id_current;
  }
}

void proceedLEDByPosition()
{
  // FastLED.setBrightness(led_brightness);
  // FastLED.clear();
  // for (int i = 0; i < LEDS_COUNT; i++)
  // {
  //   double led_pos = double(i) / LEDS_COUNT;                  // [0, 1]
  //   double led_offset_from_dot = abs(space_position - led_pos); // [0, 1]
  //   double ojfset_coef = min(led_offset_from_dot / dot_whide, 1);
  //   double power = 255 * (1 - ojfset_coef);
  //   leds[i] = CHSV(led_color_hue, 255, max(power, 0));
  //   //Serial.println(power);
  // }
  // FastLED.show();
}

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
  // double sound_left_pwr = mapDouble(1 - space_position, 0, 1, VIBRO_PWR_MIN, VIBRO_PWR_MAX);
  // double sound_right_pwr = mapDouble(space_position, 0, 1, VIBRO_PWR_MIN, VIBRO_PWR_MAX);

  // Serial.println(sound_left_pwr);
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
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, LEDS_COUNT);
  // FastLED.setCorrection( TypicalLEDStrip );
  // limit my draw to 1A at 5v of power draw
  // FastLED.setMaxPowerInVoltsAndMilliamps(5,1000);
}

void loop()
{
  updateControledParams();
  updateIterationProgress();
  updateSpacePosition();

  proceedLED();
  // proceedVibro();
  // proceedSound();

  // delay(1000.0 / FRAME_RATE); // forced frame rate emulation
}
