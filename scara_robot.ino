 
#include <AccelStepper.h>
#include <math.h>
#include <EEPROM.h>

// Motor pins
#define MOTOR1_STEP 2
#define MOTOR1_DIR 3
#define MOTOR2_STEP 4
#define MOTOR2_DIR 5
#define MOTOR3_STEP 6
#define MOTOR3_DIR 7

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP, MOTOR3_DIR);

const float stepsPerRevolution = 1600.0;
const float mmPerStep_Z = 0.01;
const float stepsPerDegree = stepsPerRevolution / 360.0;

const float l0 = 250;
const float l1 = 150;
const float l2 = 150;
const float l3 = 100;

String mode = "";
bool emergencyStop = false;
long last_report_time = 0;

void setup() {
    Serial.begin(9600);
    delay(200);

    // Load positions from EEPROM
    long theta1_steps, theta2_steps, d3_steps;

    EEPROM.get(0, theta1_steps);
    EEPROM.get(10, theta2_steps);
    EEPROM.get(20, d3_steps);

    stepper1.setCurrentPosition(theta1_steps);
    stepper2.setCurrentPosition(theta2_steps);
    stepper3.setCurrentPosition(d3_steps);

    setDefaultMotorSettings();
    Serial.println("READY. Type FK or IK:");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "STOP") {
            emergencyStop = true;
            stepper1.stop();
            stepper2.stop();
            stepper3.stop();

            Serial.println("EMERGENCY STOP");
            return;
        }

        if (input == "RESET") {
            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);
            stepper3.setCurrentPosition(0);

            savePositionToEEPROM();
            Serial.println("Position Reset to (0,0,0)");
            return;
        }

        if (input == "FK") {
            mode = "FK";
            Serial.println("FK MODE SELECTED.");
            return;
        }

        if (input == "IK") {
            mode = "IK";
            Serial.println("IK MODE SELECTED.");
            return;
        }

        if (emergencyStop) {
            Serial.println("STOPPED. RESET NEEDED.");
            return;
        }

        if (mode == "FK") {
            float dtheta1 = readNextFloat(input);
            float dtheta2 = readNextFloat(input);
            float dd3 = input.toFloat();

            long t1_steps = stepper1.currentPosition()
                + (long)(dtheta1 * stepsPerDegree);

            long t2_steps = stepper2.currentPosition()
                + (long)(dtheta2 * stepsPerDegree);

            long d3_steps = stepper3.currentPosition()
                + (long)(dd3 / mmPerStep_Z);

            stepper1.moveTo(t1_steps);
            stepper2.moveTo(t2_steps);
            stepper3.moveTo(d3_steps);

            Serial.println("SYNCHRONIZED MOTION START");

            while (stepper1.distanceToGo() != 0 ||
                   stepper2.distanceToGo() != 0 ||
                   stepper3.distanceToGo() != 0) {

                stepper1.run();
                stepper2.run();
                stepper3.run();

                reportLivePosition();
            }

            savePositionToEEPROM();
        }
        else if (mode == "IK") {
            float x = readNextFloat(input);
            float y = readNextFloat(input);
            float z = input.toFloat();

            float r2 = x * x + y * y;

            float cos_theta2 =
                (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);

            if (abs(cos_theta2) > 1) {
                Serial.println("Unreachable Point.");
                return;
            }

            float sin_theta2 =
                sqrt(1 - cos_theta2 * cos_theta2);

            float theta2 = atan2(sin_theta2, cos_theta2);

            float k1 = l1 + l2 * cos(theta2);
            float k2 = l2 * sin(theta2);

            float theta1 =
                atan2(y, x) - atan2(k2, k1);

            float d3 = l0 - z - l3;

            long t1_steps =
                (long)(degrees(theta1) * stepsPerDegree);

            long t2_steps =
                (long)(degrees(theta2) * stepsPerDegree);

            long d3_steps = (long)(d3 / mmPerStep_Z);

            stepper1.moveTo(t1_steps);
            stepper2.moveTo(t2_steps);
            stepper3.moveTo(d3_steps);

            Serial.println("SYNCHRONIZED MOTION START");

            while (stepper1.distanceToGo() != 0 ||
                   stepper2.distanceToGo() != 0 ||
                   stepper3.distanceToGo() != 0) {

                stepper1.run();
                stepper2.run();
                stepper3.run();

                reportLivePosition();
            }

            savePositionToEEPROM();
        }
    }

    stepper1.run();
    stepper2.run();
    stepper3.run();

    reportLivePosition();
}

void reportLivePosition() {
    unsigned long now = millis();

    if (now - last_report_time < 100) return;

    last_report_time = now;

    float theta1 =
        stepper1.currentPosition() / stepsPerDegree;

    float theta2 =
        stepper2.currentPosition() / stepsPerDegree;

    float d3 =
        stepper3.currentPosition() * mmPerStep_Z;

    if (mode == "FK") {
        float x = l1 * cos(radians(theta1))
                + l2 * cos(radians(theta1 + theta2));

        float y = l1 * sin(radians(theta1))
                + l2 * sin(radians(theta1 + theta2));

        float z = l0 - d3 - l3;

        Serial.print("X=");
        Serial.print(x);

        Serial.print(" Y=");
        Serial.print(y);

        Serial.print(" Z=");
        Serial.println(z);
    }
    else if (mode == "IK") {
        Serial.print("θ1=");
        Serial.print(theta1);

        Serial.print(" θ2=");
        Serial.print(theta2);

        Serial.print(" d3=");
        Serial.println(d3);
    }
}

float readNextFloat(String &s) {
    float val = s.substring(0, s.indexOf(' ')).toFloat();
    s = s.substring(s.indexOf(' ') + 1);
    return val;
}

void setDefaultMotorSettings() {
    stepper1.setMaxSpeed(500);
    stepper2.setMaxSpeed(500);
    stepper3.setMaxSpeed(500);

    stepper1.setAcceleration(300);
    stepper2.setAcceleration(300);
    stepper3.setAcceleration(200);
}

void savePositionToEEPROM() {
    EEPROM.put(0, stepper1.currentPosition());
    EEPROM.put(10, stepper2.currentPosition());
    EEPROM.put(20, stepper3.currentPosition());
}
