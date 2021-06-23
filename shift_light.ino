#include <APA102.h>

#define MAX_GREEN 3
#define MAX_YELLOW 6
#define MAX_BRIGHTNESS 31
//#define TEST
//#define DEBUG

const double pulses_per_rev = 2.0;
const uint16_t min_rpm = 4000;
const uint16_t max_rpm = 4800;
const uint16_t flash_rpm = 5000;
const uint16_t num_lights = 8;
const uint16_t rpm_increment = (max_rpm - min_rpm) / num_lights;
const uint16_t rpm_hysteresis = rpm_increment / 5;
uint16_t rpm;
// when this is num_lights + 1, set all red; when it is num_lights + 2, flash cyan.
uint8_t num_lights_illuminated;

const uint8_t rpm_pin = 7;
const uint8_t rpm_periods = 4;
unsigned long rpm_pulse_times[rpm_periods+1]; // rising edge to rising edge is two samples but one period
uint8_t current_time_index = 0;
uint8_t earliest_time_index = 0;

// enable pins are for the logic level shifter; not needed for 5V boards
const uint8_t clock_en_pin = 33;
const uint8_t clock_pin = 34;
const uint8_t data_en_pin = 20;
const uint8_t data_pin = 21;
const uint16_t led_count = num_lights * 2 - 1; // should be symmetrical, meeting in the middle; -1 because I goofed
APA102<data_pin, clock_pin> led_strip;
rgb_color colors[led_count];
uint8_t brightness;

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
  } else if (rpm < 3950) {
    rpm = 3950;
  } else if (rpm < 5100) {
    rpm += 2;
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
  double pulses_per_min = (double)rpm_periods / (double)time_diff * 1000000.0 * 60.0;
  return (uint16_t)(pulses_per_min / pulses_per_rev);
#endif
}

uint8_t get_on_brightness() {
  return 24;
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
  uint8_t new_num_lights = (rpm - min_rpm) / rpm_increment + 1;
#ifdef DEBUG
  Serial.print("RPM: "); Serial.println(rpm);
  Serial.print("RPM increment: "); Serial.println(rpm_increment);
  Serial.print("num_lights_illuminated: "); Serial.println(num_lights_illuminated);
  Serial.print("new_num_lights: "); Serial.println(new_num_lights);
#endif
  // hysteresis
  if (new_num_lights == num_lights_illuminated + 1 && rpm < min_rpm + rpm_increment * num_lights_illuminated + rpm_hysteresis) {
    new_num_lights = num_lights_illuminated;
  } else if (new_num_lights == num_lights_illuminated - 1 && rpm > min_rpm + rpm_increment * new_num_lights - rpm_hysteresis) {
    new_num_lights = num_lights_illuminated;
  }
  return new_num_lights;
}

void light_on(uint8_t light, uint8_t red, uint8_t green, uint8_t blue) {
  colors[light].red = red;
  colors[light].green = green;
  colors[light].blue = blue;
  colors[led_count - light - 1].red = red;
  colors[led_count - light - 1].green = green;
  colors[led_count - light - 1].blue = blue;
}

void light_off(uint8_t light) {
  colors[light].red = 0;
  colors[light].green = 0;
  colors[light].blue = 0;
  // see comment in light_on()
  colors[led_count - light - 1].red = 0;
  colors[led_count - light - 1].green = 0;
  colors[led_count - light - 1].blue = 0;
}

void reset_rpm_time_array() {
  for (uint8_t c = 0; c <= rpm_periods; c++) {
    rpm_pulse_times[c] = 0;
  }
  current_time_index = 0;
}

void IRAM_ATTR record_rpm_pulse_time() {
  current_time_index = earliest_time_index;
  if (++earliest_time_index > rpm_periods) {
    earliest_time_index = 0;
  }
  rpm_pulse_times[current_time_index] = micros();
#ifdef DEBUG
  pulse_detected = true;
#endif
}

void setup() {
  // hardcode a few values for initialization
  rpm = flash_rpm;
  num_lights_illuminated = num_lights + 2;
  brightness = get_on_brightness();
  pinMode(rpm_pin, INPUT_PULLUP);
  attachInterrupt(rpm_pin, record_rpm_pulse_time, RISING);
  reset_rpm_time_array();
  pinMode(clock_en_pin, OUTPUT);
  pinMode(data_en_pin, OUTPUT);
  digitalWrite(clock_en_pin, LOW);
  digitalWrite(data_en_pin, LOW);
#ifdef DEBUG
  Serial.begin(115200);
#endif
}

void loop() {
  rpm = get_rpm();
  if (rpm < 300) {
    if (millis() % 1024 > 511) {
      brightness = get_on_brightness();
    } else {
      brightness = 0;
    }
    for (uint16_t c = 0; c < num_lights; c++) {
      light_on(c, 255, 0, 0);
    }
  } else if (rpm < min_rpm) {
    brightness = 0;
  } else if (rpm < max_rpm) {
    brightness = get_on_brightness();
    num_lights_illuminated = get_num_lights_to_illuminate(rpm);
    for (uint8_t c = 0; c < MAX_GREEN && c < num_lights && c < num_lights_illuminated; c++) {
      light_on(c, 0, 180, 0);
    }
    for (uint8_t c = MAX_GREEN; c < MAX_YELLOW && c < num_lights && c < num_lights_illuminated; c++) {
      light_on(c, 255, 80, 0);
    }
    for (uint8_t c = MAX_YELLOW; c < num_lights && c < num_lights_illuminated; c++) {
      light_on(c, 255, 0, 0);
    }
    for (uint8_t c = num_lights_illuminated; c < num_lights; c++) {
      light_off(c);
    }
  } else if (rpm < flash_rpm) {
    brightness = get_on_brightness();
    for (uint16_t c = 0; c < num_lights; c++) {
      light_on(c, 255, 0, 0);
    }
  } else {
    // flash
    if (millis() % 128 > 63) {
      brightness = get_flash_brightness();
    } else {
      brightness = 0;
    }
    for (uint16_t c = 0; c < num_lights; c++) {
      light_on(c, 0, 255, 255);
    }
  }
  led_strip.write(colors, led_count, brightness);
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
      //Serial.print("Pulse time: ");
      //Serial.println(rpm_pulse_times[c]);
    }
    Serial.print("RPM: "); Serial.println(rpm);
  }
  interrupts();
#endif
}
