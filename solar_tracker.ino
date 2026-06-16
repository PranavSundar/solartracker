#include <Servo.h>  

Servo horizontal;
int servohori = 180;  
int servohoriLimitHigh = 175;
int servohoriLimitLow = 5;

Servo vertical;
int servovert = 45;  
int servovertLimitHigh = 100;
int servovertLimitLow = 1;

int ldrlt = A0;  // Top Left
int ldrrt = A1;  // Top Right
int ldrld = A2;  // Bottom Left
int ldrrd = A3;  // Bottom Right

float slt, srt, sld, srd;
const float alpha = 0.2;  // Faster response than 0.05, still smoothed

void setup() {
  Serial.begin(9600);
  horizontal.attach(3);
  vertical.attach(9);
  horizontal.write(servohori);
  vertical.write(servovert);
  delay(2500);

  // Seed EMA properly
  slt = analogRead(ldrlt);
  srt = analogRead(ldrrt);
  sld = analogRead(ldrld);
  srd = analogRead(ldrrd);
}

void loop() {
  // Single reads — no extra delay, fast loop
  int lt = analogRead(ldrlt);
  int rt = analogRead(ldrrt);
  int ld = analogRead(ldrld);
  int rd = analogRead(ldrrd);

  // EMA smoothing
  slt = alpha * lt + (1 - alpha) * slt;
  srt = alpha * rt + (1 - alpha) * srt;
  sld = alpha * ld + (1 - alpha) * sld;
  srd = alpha * rd + (1 - alpha) * srd;

  const int tol = 12;   // Dead-band — ignore tiny fluctuations
  const int dtime = 20; // ms per step — sweet spot between speed and smoothness

  float avt = (slt + srt) / 2.0;
  float avd = (sld + srd) / 2.0;
  float avl = (slt + sld) / 2.0;
  float avr = (srt + srd) / 2.0;

  float dvert  = avt - avd;
  float dhoriz = avl - avr;

  // Proportional step size — bigger error = bigger jump, faster tracking
  int vstep = (abs(dvert)  > 80) ? 3 : (abs(dvert)  > 35) ? 2 : 1;
  int hstep = (abs(dhoriz) > 80) ? 3 : (abs(dhoriz) > 35) ? 2 : 1;

  if (abs(dvert) > tol) {  
    if (avt > avd) {
      servovert -= vstep;
      if (servovert < servovertLimitLow) servovert = servovertLimitLow;
    } else {
      servovert += vstep;
      if (servovert > servovertLimitHigh) servovert = servovertLimitHigh;
    }
    vertical.write(servovert);
  }

  if (abs(dhoriz) > tol) {  
    if (avl > avr) {
      servohori -= hstep;
      if (servohori < servohoriLimitLow) servohori = servohoriLimitLow;
    } else {
      servohori += hstep;
      if (servohori > servohoriLimitHigh) servohori = servohoriLimitHigh;
    }
    horizontal.write(servohori);
  }

  Serial.print("dV:"); Serial.print(dvert);
  Serial.print("  dH:"); Serial.println(dhoriz);

  delay(dtime);
}