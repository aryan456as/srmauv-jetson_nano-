#include <SimpleKalmanFilter.h>
#include <SFE_BMP180.h>

SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SFE_BMP180 pressure;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

float baseline; // baseline pressure

double getPressure() {
  char status;
  double T, P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0) {
          return (P);
        }
      }
    }
  }
  return 0; // Return 0 if pressure reading fails
}

void setup() {
  Serial.begin(115200);

  // BMP180 Pressure sensor start
  if (!pressure.begin()) {
    Serial.println("BMP180 Pressure Sensor Error!");
    while (1)
      ; // Pause forever.
  }
  baseline = getPressure();
}

void loop() {
  float p = getPressure();
  float altitude = pressure.altitude(p, baseline);
  float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);

  if (millis() > refresh_time) {
    Serial.print(altitude, 6);
    Serial.print(",");
    Serial.print(estimated_altitude, 6);
    Serial.println();
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }
}
