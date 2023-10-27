
#include <ESP32AnalogRead.h>
#include <APA102.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"

#include <math.h>

//#define TEST
//#define DEBUG

#define MAX_GREEN 3
#define MAX_YELLOW 6
#define MAX_BRIGHTNESS 31

#define VCC_VOLTS 3.3f
#define PSU_VOLTS 3.3f

// ADCs are on pins 1-17
#define OIL_PRESS_PIN 2    // green
#define OIL_TEMP_PIN 3     // black?
#define WATER_PRESS_PIN 4  // orange
#define WATER_TEMP_PIN 5   // blue
#define FUEL_READ_EN_PIN 10
#define FUEL_PIN 6
#define VOLTMETER_PIN 9

ESP32AnalogRead oil_press_adc, oil_temp_adc, water_press_adc, water_temp_adc, fuel_adc, volts_adc;

#define OIL_PRESS_RESISTOR_OHMS 99.9f
#define WATER_PRESS_RESISTOR_OHMS 129.9f
#define OIL_TEMP_RESISTOR_OHMS 997.0f
#define WATER_TEMP_RESISTOR_OHMS 618.0f
#define FUEL_RESISTOR_OHMS 996.0f
#define VOLTMETER_PRIMARY_RESISTOR_OHMS 3306.0f
#define VOLTMETER_SECONDARY_RESISTOR_OHMS 998.0f

// coefficients for the Steinhart-Hart thermistor model
#define WATER_TEMP_SENSOR_A 1.6859e-03
#define WATER_TEMP_SENSOR_B 2.622578527e-04
#define WATER_TEMP_SENSOR_C 4.79495763e-08
#define OIL_TEMP_SENSOR_A 1.801187287e-03
#define OIL_TEMP_SENSOR_B 2.18652583e-04
#define OIL_TEMP_SENSOR_C 2.719311439e-07

#ifdef TEST
#include <cstdlib>
#endif

const double PULSES_PER_REV = 2.0;
const uint16_t MIN_RPM = 4300;
const uint16_t MAX_RPM = 5300;
const uint16_t FLASH_RPM = 5500;
const uint16_t NUM_LIGHTS = 8;
const uint16_t RPM_INCREMENT = (MAX_RPM - MIN_RPM) / NUM_LIGHTS;
const uint16_t RPM_HYSTERESIS = RPM_INCREMENT / 5;
uint16_t rpm;
// when this is num_lights + 1, set all red; when it is num_lights + 2, flash cyan.
uint8_t num_lights_illuminated;

float oil_press = 0.0f;
float oil_temp = 0.0f;
float water_press = 0.0f;
float water_temp = 0.0f;
float fuel = 0.0f;
float volts = 0.0f;

const uint8_t RPM_PIN = 1;
const uint8_t RPM_PERIODS = 4;
unsigned long rpm_pulse_times[RPM_PERIODS + 1];  // rising edge to rising edge is two samples but one period
uint8_t current_time_index = 0;
uint8_t earliest_time_index = 0;

// I want oil pressure to be pretty responsive
const uint8_t OIL_PRESS_SAMPLE_COUNT = 8;
float oil_press_samples[OIL_PRESS_SAMPLE_COUNT];
uint8_t current_oil_press_sample_index = 0;
// 64 samples will be about two seconds worth of data.
const uint8_t WATER_TEMP_SAMPLE_COUNT = 64;
float water_temp_samples[WATER_TEMP_SAMPLE_COUNT];
uint8_t current_water_temp_sample_index = 0;
// fuel does not have to be very responsive, but will be very noisy.
const uint8_t FUEL_SAMPLE_COUNT = 255;
float fuel_samples[FUEL_SAMPLE_COUNT];
uint8_t current_fuel_sample_index = 0;

// enable pins are for the logic level shifter; not needed for 5V boards
const uint8_t CLOCK_EN_PIN = 33;
const uint8_t CLOCK_PIN = 41;
const uint8_t DATA_EN_PIN = 20;
const uint8_t DATA_PIN = 42;
const uint16_t LED_COUNT = NUM_LIGHTS * 2;  // should be symmetrical, meeting in the middle
APA102<DATA_PIN, CLOCK_PIN> led_strip;
rgb_color colors[LED_COUNT];
uint8_t brightness;

#define LEDS_COUNT 1
#define LEDS_PIN 18
#define CHANNEL 0
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL);

#ifdef DEBUG
bool pulse_detected = false;
unsigned long last_print_millis = 0;
unsigned long print_interval = 100;
#endif

uint16_t get_rpm() {
#ifdef TEST
  if (rpm < 30) {
    delay(100);
    rpm++;
  } else if (rpm < 4450) {
    rpm += 19;
  } else if (rpm < 6500) {
    rpm += 7;
  } else {
    rpm = 0;
  }
  return rpm;
#else
  // If the engine is off, pulses won't be added to rpm_pulse_times and the difference between the
  // first and last recorded pulse will remain whatever it was when the last pulse was detected.
  // More than 100ms between now and the last pulse means < 300RPM, so assume the engine is off.
  if (micros() - rpm_pulse_times[current_time_index] > 100000) {
    return 0;
  }
  unsigned long time_diff = rpm_pulse_times[current_time_index] - rpm_pulse_times[earliest_time_index];
  double pulses_per_min = (double)RPM_PERIODS / (double)time_diff * 1000000.0 * 60.0;
  return (uint16_t)(pulses_per_min / PULSES_PER_REV);
#endif
}

float get_sensor_ohms(ESP32AnalogRead adc, float power_supply_volts, float primary_resistor_ohms) {
  float sensor_volts = adc.readVoltage();
  return sensor_volts * primary_resistor_ohms / (power_supply_volts - sensor_volts);
}

float get_steinhart_hart_thermistor_tempF(float ohms, float a, float b, float c) {
  float tempK = 1.0f / (a + b * log(ohms) + c * pow(log(ohms), 3.0f));
  float tempC = tempK - 273.15f;
  float tempF = 5.0f / 9.0f * tempC + 32.0f;
  return tempF;
}

float get_oil_temp() {
#ifdef TEST
  if (rand() % 10 == 0) oil_temp -= 1.0f;
  if (rand() % 10 == 9) oil_temp += 1.0f;
#else
  float sensor_ohms = get_sensor_ohms(OIL_TEMP_PIN, PSU_VOLTS, OIL_TEMP_RESISTOR_OHMS);
  oil_temp = get_steinhart_hart_thermistor_tempF(sensor_ohms, OIL_TEMP_SENSOR_A, OIL_TEMP_SENSOR_B, OIL_TEMP_SENSOR_C);
#endif
  return oil_temp;
}

float get_oil_press() {
#ifdef TEST
  oil_press = rpm / 100.0f;
  return oil_press;
#else
  float sensor_ohms = get_sensor_ohms(OIL_PRESS_PIN, PSU_VOLTS, OIL_PRESS_RESISTOR_OHMS);
  float current_oil_press_sample;
  // This formula was derived experimentally.
  // For the range within which I am most concerned, this will return the correct pressure
  // (assuming I can trust the gauge I used to calibrate it) within 1.5psi
  if (sensor_ohms < 2.0f) {
    current_oil_press_sample = 0.0f;
  } else {
    current_oil_press_sample = sensor_ohms * 0.77f + 6.5f;
  }
  oil_press_samples[current_oil_press_sample_index] = current_oil_press_sample;
  if (++current_oil_press_sample_index == OIL_PRESS_SAMPLE_COUNT) {
    current_oil_press_sample_index = 0;
  }
  float oil_press_average = 0.0f;
  for (float oil_press_sample : oil_press_samples) {
    oil_press_average += oil_press_sample;
  }
  oil_press = oil_press_average / (float)OIL_PRESS_SAMPLE_COUNT;
#endif
  return oil_press;
}

float get_water_temp() {
#ifdef TEST
  if (rand() % 10 == 0) water_temp -= 1.0f;
  if (rand() % 10 == 9) water_temp += 1.0f;
#else
  float sensor_ohms = get_sensor_ohms(WATER_TEMP_PIN, PSU_VOLTS, WATER_TEMP_RESISTOR_OHMS);
  float current_water_temp_sample = get_steinhart_hart_thermistor_tempF(sensor_ohms, WATER_TEMP_SENSOR_A, WATER_TEMP_SENSOR_B, WATER_TEMP_SENSOR_C);
  water_temp_samples[current_water_temp_sample_index] = current_water_temp_sample;
  if (++current_water_temp_sample_index > WATER_TEMP_SAMPLE_COUNT) {
    current_water_temp_sample_index = 0;
  }
  float water_temp_average = 0.0f;
  for (float water_temp_sample : water_temp_samples) {
    water_temp_average += water_temp_sample;
  }
  water_temp = water_temp_average / (float)WATER_TEMP_SAMPLE_COUNT;
#endif
  return water_temp;
}

float get_water_press() {
#ifdef TEST
  if (rand() % 10 == 0) water_press -= 0.1f;
  if (rand() % 10 == 9) water_press += 0.1f;
#else
  float sensor_ohms = get_sensor_ohms(WATER_PRESS_PIN, PSU_VOLTS, WATER_PRESS_RESISTOR_OHMS);
  // This formula was determined experimentally. See the comment in get_oil_press.
  // This should return the correct pressure within 0.5psi for the normal range (5-20psi)
  if (sensor_ohms > 230.0f) {
    water_press = 0.0f;
  } else {
    water_press = (-32.0f * sensor_ohms + 8400.0f) / sensor_ohms;
  }
#endif
  return water_press;
}

float get_fuel() {
#ifdef TEST
  fuel -= 0.01f;
  if (fuel < 0.0f) {
    fuel = 12.0f;
  }
#else
  digitalWrite(FUEL_READ_EN_PIN, HIGH);
  float sensor_ohms = get_sensor_ohms(FUEL_PIN, PSU_VOLTS, FUEL_RESISTOR_OHMS);
  float current_fuel_sample = 11.75f - 0.2f * sensor_ohms;
  digitalWrite(FUEL_READ_EN_PIN, LOW);
  fuel_samples[current_fuel_sample_index] = current_fuel_sample;
  if (++current_fuel_sample_index > FUEL_SAMPLE_COUNT) {
    current_fuel_sample_index = 0;
  }
  float fuel_average = 0.0f;
  for (float fuel_sample : fuel_samples) {
    fuel_average += fuel_sample;
  }
  fuel = fuel_average / (float)FUEL_SAMPLE_COUNT;
#endif
  return fuel;
}

float get_battery_volts() {
#ifdef TEST
  if (rand() % 10 == 0) volts -= 0.01f;
  if (rand() % 10 == 9) volts += 0.01f;
  return volts;
#else
  float sensor_volts = volts_adc.readVoltage();
  return sensor_volts * (VOLTMETER_PRIMARY_RESISTOR_OHMS + VOLTMETER_SECONDARY_RESISTOR_OHMS) / VOLTMETER_SECONDARY_RESISTOR_OHMS;
#endif
}

uint8_t get_on_brightness() {
  return 1;  //24
}

uint8_t get_flash_brightness() {
  uint8_t brightness = get_on_brightness() + 1;
  if (brightness > MAX_BRIGHTNESS) {
    brightness = MAX_BRIGHTNESS;
  }
  return brightness;
}

uint8_t get_num_lights_to_illuminate(uint16_t rpm) {
  // one light should be on at min_rpm
  uint8_t new_num_lights = (rpm - MIN_RPM) / RPM_INCREMENT + 1;
#ifdef DEBUG
  Serial.print("RPM: ");
  Serial.println(rpm);
  Serial.print("RPM increment: ");
  Serial.println(rpm_increment);
  Serial.print("num_lights_illuminated: ");
  Serial.println(num_lights_illuminated);
  Serial.print("new_num_lights: ");
  Serial.println(new_num_lights);
#endif
  // hysteresis
  if (new_num_lights == num_lights_illuminated + 1 && rpm < MIN_RPM + RPM_INCREMENT * num_lights_illuminated + RPM_HYSTERESIS) {
    new_num_lights = num_lights_illuminated;
  } else if (new_num_lights == num_lights_illuminated - 1 && rpm > MIN_RPM + RPM_INCREMENT * new_num_lights - RPM_HYSTERESIS) {
    new_num_lights = num_lights_illuminated;
  }
  return new_num_lights;
}

void light_on(uint8_t light, uint8_t red, uint8_t green, uint8_t blue) {
  colors[light].red = red;
  colors[light].green = green;
  colors[light].blue = blue;
  colors[LED_COUNT - light - 1].red = red;
  colors[LED_COUNT - light - 1].green = green;
  colors[LED_COUNT - light - 1].blue = blue;
}

void light_off(uint8_t light) {
  colors[light].red = 0;
  colors[light].green = 0;
  colors[light].blue = 0;
  // see comment in light_on()
  colors[LED_COUNT - light - 1].red = 0;
  colors[LED_COUNT - light - 1].green = 0;
  colors[LED_COUNT - light - 1].blue = 0;
}

void reset_rpm_time_array() {
  for (uint8_t c = 0; c <= RPM_PERIODS; c++) {
    rpm_pulse_times[c] = 0;
  }
  current_time_index = 0;
}

void IRAM_ATTR record_rpm_pulse_time() {
  unsigned long time = micros();
  if (time - rpm_pulse_times[current_time_index] < 3000) {
    // There is a bit of noise in the signal, and because it's used to control ignition I don't want to filter it.
    // If we're here, pulse length is less than 2.5ms. This indicates 10000 RPM. Either it's noise or the engine is already toast. Either way, we can safely ignore it.
    return;
  }
  current_time_index = earliest_time_index;
  if (++earliest_time_index > RPM_PERIODS) {
    earliest_time_index = 0;
  }
  rpm_pulse_times[current_time_index] = time;
#ifdef DEBUG
  pulse_detected = true;
#endif
}

void update_values() {
  rpm = get_rpm();
  oil_press = get_oil_press();
  oil_temp = get_oil_temp();
  water_press = get_water_press();
  water_temp = get_water_temp();
  fuel = get_fuel();
  volts = get_battery_volts();
}

void serial_output_values() {
  // TODO truncate values to one or two decimals
  Serial.print("RPM:");
  // would be better to use binary values and Serial.write(), but that becomes more complicated as only 8 bits fit in a char
  Serial.print(rpm);
  Serial.print('\n');

  Serial.print("OP:");
  Serial.print(oil_press);
  Serial.print('\n');

  Serial.print("OT:");
  Serial.print(oil_temp);
  Serial.print('\n');

  Serial.print("WP:");
  Serial.print(water_press);
  Serial.print('\n');

  Serial.print("WT:");
  Serial.print(water_temp);
  Serial.print('\n');

  Serial.print("FUEL:");
  Serial.print(fuel);
  Serial.print('\n');

  Serial.print("VOLTS:");
  Serial.print(volts);
  Serial.print('\n');
}

void setup() {
  pinMode(RPM_PIN, INPUT_PULLUP);
  pinMode(FUEL_READ_EN_PIN, OUTPUT);
  digitalWrite(FUEL_READ_EN_PIN, LOW);

  oil_press_adc.attach(OIL_PRESS_PIN);
  oil_temp_adc.attach(OIL_TEMP_PIN);
  water_press_adc.attach(WATER_PRESS_PIN);
  water_temp_adc.attach(WATER_TEMP_PIN);
  fuel_adc.attach(FUEL_PIN);
  volts_adc.attach(VOLTMETER_PIN);

#ifdef TEST
  oil_press = 30.0f;
  oil_temp = 190.0f;
  water_press = 13.0f;
  water_temp = 180.0f;
  fuel = 12.0f;
  volts = 14.0f;
#endif
  strip.begin();
  strip.setBrightness(15);

  // hardcode a few values for initialization
  rpm = FLASH_RPM;
  num_lights_illuminated = NUM_LIGHTS + 2;
  brightness = get_on_brightness();
  attachInterrupt(RPM_PIN, record_rpm_pulse_time, RISING);
  reset_rpm_time_array();
  pinMode(CLOCK_EN_PIN, OUTPUT);
  pinMode(DATA_EN_PIN, OUTPUT);
  digitalWrite(CLOCK_EN_PIN, LOW);
  digitalWrite(DATA_EN_PIN, LOW);
  Serial.begin(921600);
  delay(100);
}

void loop() {
  update_values();

  serial_output_values();

  if ((rpm > 6000 || oil_press < 10.0f || oil_temp > 230.0f || water_press < 6.5f || water_temp > 220.0f) && millis() % 128 > 63) {
    strip.setLedColor(0, 255, 0, 0);
  } else {
    strip.setLedColor(0, 0, 0, 0);
  }

  if (rpm < 300) {
    if (millis() % 1024 > 511) {
      brightness = get_on_brightness();
    } else {
      brightness = 0;
    }
    for (uint16_t c = 0; c < NUM_LIGHTS; c++) {
      light_on(c, 255, 0, 0);
    }
  } else if (rpm < MIN_RPM) {
    brightness = 0;
  } else if (rpm < MAX_RPM) {
    brightness = get_on_brightness();
    num_lights_illuminated = get_num_lights_to_illuminate(rpm);
    for (uint8_t c = 0; c < MAX_GREEN && c < NUM_LIGHTS && c < num_lights_illuminated; c++) {
      light_on(c, 0, 180, 0);
    }
    for (uint8_t c = MAX_GREEN; c < MAX_YELLOW && c < NUM_LIGHTS && c < num_lights_illuminated; c++) {
      light_on(c, 255, 80, 0);
    }
    for (uint8_t c = MAX_YELLOW; c < NUM_LIGHTS && c < num_lights_illuminated; c++) {
      light_on(c, 255, 0, 0);
    }
    for (uint8_t c = num_lights_illuminated; c < NUM_LIGHTS; c++) {
      light_off(c);
    }
  } else if (rpm < FLASH_RPM) {
    brightness = get_on_brightness();
    for (uint16_t c = 0; c < NUM_LIGHTS; c++) {
      light_on(c, 255, 0, 0);
    }
  } else {
    // flash
    if (millis() % 128 > 63) {
      brightness = get_flash_brightness();
    } else {
      brightness = 0;
    }
    for (uint16_t c = 0; c < NUM_LIGHTS; c++) {
      light_on(c, 0, 255, 255);
    }
  }
  led_strip.write(colors, LED_COUNT, brightness);
  delay(30);  // this makes the RPi happy, and we don't have to update every microsecond anyway.
#ifdef DEBUG

  noInterrupts();
  if (pulse_detected) {
    Serial.print("Pulse us: ");
    Serial.println(rpm_pulse_times[current_time_index]);
    pulse_detected = false;
  }
  if (millis() - print_interval > last_print_millis) {
    last_print_millis = millis();
    for (uint8_t c = 0; c <= rpm_periods; c++) {
      Serial.print("Pulse time: ");
      Serial.println(rpm_pulse_times[c]);
    }
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
  interrupts();
#endif
}
