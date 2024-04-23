#include <SimpleKalmanFilter.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read a reference value from A0 and map it from 0 to 100
  float real_value = analogRead(A0) / 4095.0 * 100.0; // Assuming 12-bit ADC resolution for Jetson NANO

  // Add noise to the reference value and use it as the measured value
  float measured_value = real_value + random(-100, 100) / 100.0;

  // Calculate the estimated value with Kalman Filter
  float estimated_value = simpleKalmanFilter.updateEstimate(measured_value);

  // Send to Serial output every 100ms
  if (millis() > refresh_time) {
    Serial.print(real_value, 4);
    Serial.print(",");
    Serial.print(measured_value, 4);
    Serial.print(",");
    Serial.print(estimated_value, 4);
    Serial.println();

    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }
}
