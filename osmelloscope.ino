#define MEASURE_PIN 7
#define NUM_SAMPLES 16384

uint16_t samples[NUM_SAMPLES];
long sample_times[NUM_SAMPLES];

void clear_samples() {
  for (int c = 0; c < NUM_SAMPLES; c++) {
    samples[c] = 0;
    sample_times[c] = 0l;
  }
}

void setup() {
  pinMode(MEASURE_PIN, INPUT);
  Serial.begin(2000000);
  clear_samples();
}

void loop() {
  while (!Serial.available()) {
    delay(10);
  }
  for (int c = 0; c < NUM_SAMPLES; c++) {
    sample_times[c] = micros();
    samples[c] = analogRead(MEASURE_PIN);
  }
  for (int c = 0; c < NUM_SAMPLES; c++) {
    Serial.print(sample_times[c]);
    Serial.print(" ");
    Serial.println((float)samples[c]/8191.0f*3.3f);
  }
  // clear the serial buffer
  while(Serial.available()) {
    Serial.read();
  }
}
