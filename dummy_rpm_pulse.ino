long delay_us = 0;
// give it 1s to initialize
long next_pulse_micros = 1000000;
long last_pulse_micros = next_pulse_micros;
uint16_t pulses_per_rev = 2;

uint8_t pulse_pin = 6;
uint8_t extra_ground_pin = 7;
uint8_t extra_extra_ground_pin = 0;

uint16_t rpm =  250;


uint16_t get_rpm() {
  if (rpm < 300) {
    rpm += 2;
  } else if (rpm < 3950) {
    rpm = 3950;
  } else if (rpm < 5050) {
    rpm += 1;
  } else {
    rpm = 250;
  }
  return rpm;
}

long get_next_pulse_micros(long last_pulse_micros, uint16_t rpm) {
  double revs_per_sec = ((double) rpm) / 60.0;
  long micros_per_rev = (long) (1000000.0 / revs_per_sec);
  return last_pulse_micros + micros_per_rev / pulses_per_rev;
}

void setup() {
  pinMode(pulse_pin, OUTPUT);
  pinMode(extra_ground_pin, OUTPUT);
  pinMode(extra_extra_ground_pin, OUTPUT);
  digitalWrite(pulse_pin, LOW);
  digitalWrite(extra_ground_pin, LOW);
  digitalWrite(extra_extra_ground_pin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay_us = next_pulse_micros - micros();
  if (delay_us < 10000) {
    delayMicroseconds(delay_us);
  } else {
    delay(delay_us / 1000);
  }
  digitalWrite(pulse_pin, HIGH);
  delay(1);
  digitalWrite(pulse_pin, LOW);
  last_pulse_micros = next_pulse_micros;
  next_pulse_micros = get_next_pulse_micros(last_pulse_micros, get_rpm());
  if (next_pulse_micros - last_pulse_micros > 5 * 1000000) {
    next_pulse_micros = last_pulse_micros + 5 * 1000000;
  }
}

