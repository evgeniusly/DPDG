#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <SFEMP3Shield.h>

// ============================================================
// SETTINGS
// ============================================================

#define MICROLED 1

const uint8_t PIN_LED = 13;
const uint8_t PIN_VIBRO_LEFT = 11;
const uint8_t PIN_VIBRO_RIGHT = 12;

const uint8_t PIN_BTN_PREV = 3;
const uint8_t PIN_BTN_NEXT = 4;

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

const int SENSOR_MAX_VAL = 1023;

// ============================================================
// VARIABLES
// ============================================================

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

bool btn_is_active = false;

double dot_whide = 0.02;

int files_count = 0;
int audio_index = 1;

SdFat sd;
SdFile file;
SFEMP3Shield MP3player;

// ============================================================
// LED
// ============================================================

#if defined(MICROLED)
// ===== ЦВЕТОВАЯ ГЛУБИНА =====
// 1, 2, 3 (байт на цвет)
// на меньшем цветовом разрешении скетч будет занимать в разы меньше места,
// но уменьшится и количество оттенков и уровней яркости!
// дефайн делается ДО ПОДКЛЮЧЕНИЯ БИБЛИОТЕКИ
// без него будет 3 байта по умолчанию
#define COLOR_DEBTH 3
#include <microLED.h> // подключаем библу
// CLI_OFF CLI_LOW CLI_AVER CLI_HIGH
microLED<LEDS_COUNT, PIN_LED, -1, LED_WS2812, ORDER_GRB, CLI_HIGH> strip;

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

void playNext()
{
  audio_index++;
  if (audio_index > files_count)
    audio_index = 1;
  MP3player.stopTrack();
  MP3player.playTrack(audio_index);
}
void playPrev()
{
  audio_index--;
  if (audio_index < 1)
    audio_index = files_count;
  MP3player.stopTrack();
  MP3player.playTrack(audio_index);
}

void checkButtons()
{
  int btn_prev_state = digitalRead(PIN_BTN_PREV);
  int btn_next_state = digitalRead(PIN_BTN_NEXT);

  if (btn_prev_state == HIGH && !btn_is_active)
  {
    btn_is_active = true;
    playPrev();
  }

  if (btn_next_state == HIGH && !btn_is_active)
  {
    btn_is_active = true;
    playNext();
  }

  if (btn_prev_state == LOW && btn_next_state == LOW && btn_is_active)
  {
    btn_is_active = false;
  }
}

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

  iteration_progress += iteration_progress_delta;
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

  MP3player.setVolume(sound_left_pwr, sound_right_pwr);

  if (MP3player.isPlaying() == 0)
  {
    MP3player.playTrack(audio_index);
  };
}

// ============================================================
// ARDUINO
// ============================================================

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_VIBRO_LEFT, OUTPUT);
  pinMode(PIN_VIBRO_RIGHT, OUTPUT);

  pinMode(PIN_BTN_PREV, INPUT);
  pinMode(PIN_BTN_NEXT, INPUT);

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

  sd.vwd()->rewind();
  while (file.openNext(sd.vwd(), O_READ))
  {
    if (file.isFile())
    {
      files_count++;
    }
    file.close();
  }

  // Initialize Audio
  MP3player.begin();
}

void loop()
{
  checkButtons();

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
}
