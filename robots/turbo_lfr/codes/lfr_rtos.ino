#include <QTRSensors.h>
#include <Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ===================== HARDWARE ===================== */

// Motors
#define motorL1 4
#define motorL2 3
#define motorL_PWM 5

#define motorR1 7
#define motorR2 8
#define motorR_PWM 6

// EDF / ESC
#define ESC_PIN 9

// Sensors
#define SENSOR_COUNT 8
uint16_t sensorValues[SENSOR_COUNT];

/* ===================== OBJECTS ===================== */

QTRSensors qtr;
Servo Fan;

/* ===================== SHARED STATE ===================== */
// Volatile = safe between RTOS tasks

volatile int lineError = 0;
volatile int motorLeftCmd = 0;
volatile int motorRightCmd = 0;
volatile int fanTarget = 0;

/* ===================== PID PARAMETERS ===================== */

float Kp = 0.40;
float Ki = 0.0;        // Disabled on purpose
float Kd = 2.5;

volatile int lastError = 0;

/* ===================== SPEED LIMITS ===================== */

int BaseSpeed = 95;
int MaxSpeed  = 135;
int MinSpeed  = -80;

/* ===================== RTOS TASKS ===================== */

/* ---------- SENSOR TASK (1 kHz, highest priority) ---------- */
void SensorTask(void *pv) {
  while (true) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    lineError = position - 3500;   // center for 8 sensors
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ---------- PID TASK (1 kHz) ---------- */
void PIDTask(void *pv) {
  while (true) {
    int error = lineError;

    int P = error;
    int D = error - lastError;
    lastError = error;

    int correction = (int)(P * Kp + D * Kd);

    int left  = BaseSpeed + correction;
    int right = BaseSpeed - correction;

    motorLeftCmd  = constrain(left,  MinSpeed, MaxSpeed);
    motorRightCmd = constrain(right, MinSpeed, MaxSpeed);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ---------- MOTOR TASK (1 kHz) ---------- */
void MotorTask(void *pv) {
  while (true) {
    int L = motorLeftCmd;
    int R = motorRightCmd;

    // LEFT
    if (L < 0) {
      digitalWrite(motorL1, HIGH);
      digitalWrite(motorL2, LOW);
      L = -L;
    } else {
      digitalWrite(motorL1, LOW);
      digitalWrite(motorL2, HIGH);
    }

    // RIGHT
    if (R < 0) {
      digitalWrite(motorR1, LOW);
      digitalWrite(motorR2, HIGH);
      R = -R;
    } else {
      digitalWrite(motorR1, HIGH);
      digitalWrite(motorR2, LOW);
    }

    analogWrite(motorL_PWM, L);
    analogWrite(motorR_PWM, R);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ---------- FAN TASK (200 Hz, isolated) ---------- */
void FanTask(void *pv) {
  int currentPWM = 0;

  while (true) {
    if (currentPWM < fanTarget) currentPWM++;
    if (currentPWM > fanTarget) currentPWM--;

    Fan.write(currentPWM);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ---------- SERIAL TASK (debug / tuning) ---------- */
void SerialTask(void *pv) {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == 'u') fanTarget += 5;
      if (c == 'd') fanTarget -= 5;

      fanTarget = constrain(fanTarget, 0, 180);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ===================== SETUP ===================== */

void setup() {
  Serial.begin(115200);

  // Sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SENSOR_COUNT);
  qtr.setEmitterPin(13);

  delay(300);
  for (int i = 0; i < 100; i++) qtr.calibrate();

  // Motor pins
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorR_PWM, OUTPUT);

  // Fan
  Fan.attach(ESC_PIN);
  fanTarget = 60;   // safe starting suction

  /* ===== RTOS TASK CREATION ===== */
  xTaskCreatePinnedToCore(SensorTask, "Sensor", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(PIDTask,    "PID",    4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(MotorTask,  "Motor",  4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(FanTask,    "Fan",    2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialTask, "Serial", 2048, NULL, 0, NULL, 1);
}

/* ===================== LOOP ===================== */

void loop() {
  vTaskDelay(portMAX_DELAY); // loop is intentionally unused
}
