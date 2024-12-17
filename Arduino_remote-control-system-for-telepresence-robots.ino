#include "GY_85.h"
#include <Kalman.h>
#include <Wire.h>

// Motor pin definitions
#define ENA_m1 11  // Back Left
#define ENB_m1 5   // Front Left
#define ENA_m2 12  // Front Right
#define ENB_m2 13  // Back Right
#define IN_11  10  // Back Left control
#define IN_12  9   // Back Left control
#define IN_13  7   // Front Left control
#define IN_14  6   // Front Left control
#define IN_21  23  // Front Right control
#define IN_22  22  // Front Right control
#define IN_23  24  // Back Right control
#define IN_24  4   // Back Right control

// Ultrasonic sensor pins
#define TRIG_PIN_CENTER 3
#define ECHO_PIN_CENTER 2
#define TRIG_PIN_FRONT_LEFT 16
#define ECHO_PIN_FRONT_LEFT 17
#define TRIG_PIN_FRONT_RIGHT 15
#define ECHO_PIN_FRONT_RIGHT 14
#define TRIG_PIN_LEFT 26
#define ECHO_PIN_LEFT 27
#define TRIG_PIN_RIGHT 18
#define ECHO_PIN_RIGHT 19

// Motor speed constants
#define ROTATION_SPEED 100
#define MOVEMENT_SPEED 200
#define ANGLE_TOLERANCE 2.0

// Command queue
#define QUEUE_SIZE 5
char commandQueue[QUEUE_SIZE];
int queueFront = 0;
int queueRear = 0;

// Create module objects
GY_85 GY85;
Kalman kalmanX;
Kalman kalmanY;

// State variables
float targetAngle = 0;
bool isRotating = false;
String currentMovement = "S";
String lastRotation = "T";

// IMU variables
float roll = 0;
float pitch = 0;
float yaw = 0;
unsigned long lastTime;

// Calibration offsets
int ax_offset = 0;
int ay_offset = 0;
int az_offset = 0;

void setup() {
    // Initialize motor pins
    pinMode(ENA_m1, OUTPUT);
    pinMode(ENB_m1, OUTPUT);
    pinMode(ENA_m2, OUTPUT);
    pinMode(ENB_m2, OUTPUT);
    pinMode(IN_11, OUTPUT);
    pinMode(IN_12, OUTPUT);
    pinMode(IN_13, OUTPUT);
    pinMode(IN_14, OUTPUT);
    pinMode(IN_21, OUTPUT);
    pinMode(IN_22, OUTPUT);
    pinMode(IN_23, OUTPUT);
    pinMode(IN_24, OUTPUT);

    // Initialize ultrasonic pins
    pinMode(TRIG_PIN_CENTER, OUTPUT);
    pinMode(ECHO_PIN_CENTER, INPUT);
    pinMode(TRIG_PIN_FRONT_LEFT, OUTPUT);
    pinMode(ECHO_PIN_FRONT_LEFT, INPUT);
    pinMode(TRIG_PIN_FRONT_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_FRONT_RIGHT, INPUT);
    pinMode(TRIG_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIG_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);

    Serial.begin(230400);
    Wire.begin();
    
    // Initialize GY-085
    GY85.init();
    calibrateAccelerometer();
    lastTime = millis();
}

void calibrateAccelerometer() {
    int numReadings = 100;
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    
    for (int i = 0; i < numReadings; i++) {
        int* readings = GY85.readFromAccelerometer();
        ax_sum += GY85.accelerometer_x(readings);
        ay_sum += GY85.accelerometer_y(readings);
        az_sum += GY85.accelerometer_z(readings);
        delay(10);
    }
    
    ax_offset = ax_sum / numReadings;
    ay_offset = ay_sum / numReadings;
    az_offset = az_sum / numReadings;
}

void updateSensorData() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    
    // Read accelerometer data
    int* accelerometerReadings = GY85.readFromAccelerometer();
    int ax = GY85.accelerometer_x(accelerometerReadings) - ax_offset;
    int ay = GY85.accelerometer_y(accelerometerReadings) - ay_offset;
    int az = GY85.accelerometer_z(accelerometerReadings) - az_offset;
    
    // Read gyroscope data
    float* gyroReadings = GY85.readGyro();
    float gx = GY85.gyro_x(gyroReadings);
    float gy = GY85.gyro_y(gyroReadings);
    float gz = GY85.gyro_z(gyroReadings);
    
    // Convert to physical units
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    float gx_rad = radians(gx);
    float gy_rad = radians(gy);
    float gz_rad = radians(gz);
    
    // Calculate angles
    float accelPitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * RAD_TO_DEG;
    float accelRoll = atan2(-ax_g, az_g) * RAD_TO_DEG;
    
    // Apply Kalman filter
    pitch = kalmanX.getAngle(accelPitch, gy_rad, dt);
    roll = kalmanY.getAngle(accelRoll, gx_rad, dt);
    yaw -= gz_rad * dt;
}

float readUltrasonicSensor(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    unsigned long timeout = 23529;  // Maximum timeout for distance measurement
    unsigned long duration = pulseIn(echoPin, HIGH, timeout);
    
    if (duration == 0) {
        return 1.0;  // Return maximum normalized distance if no echo
    }
    
    float distance = (duration / 2.0) / 29.1;  // Convert to cm
    return distance > 25.0 ? 1.0 : distance / 25.0;  // Normalize to 0-1 range
}

void sendUltrasonicData() {
    float distances[5];
    bool shouldSendData = false;
    
    // Read all sensors
    distances[0] = readUltrasonicSensor(TRIG_PIN_CENTER, ECHO_PIN_CENTER);
    distances[1] = readUltrasonicSensor(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT);
    distances[2] = readUltrasonicSensor(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT);
    distances[3] = readUltrasonicSensor(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    distances[4] = readUltrasonicSensor(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    
    // Check if any reading is different from 1.00
    for(int i = 0; i < 5; i++) {
        if(abs(distances[i] - 1.00) > 0.001) {
            shouldSendData = true;
            break;
        }
    }
    
    // Only send data if obstacles detected
    if(shouldSendData) {
        for(int i = 0; i < 5; i++) {
            Serial.print(distances[i], 3);
            if(i < 4) Serial.print(',');
        }
        Serial.println();
    }
}

void processCommand(char command) {
    // Handle rotation commands
    if (command == 'T' || command == 'U' || command == 'V' || command == 'W' || 
        command == 'X' || command == 'Y' || command == 'J' || command == 'Z') {
        
        String newRotation(command);
        if (newRotation != lastRotation) {
            lastRotation = newRotation;
            updateTargetAngle(command);
            isRotating = true;
        }
    }
    // Handle movement commands
    else if (command == 'F' || command == 'D' || command == 'B' || command == 'R' || command == 'L' || command == 'P' || command == 'K' ||
             command == 'S' || command == 'C' || command == 'G' || command == 'I' || 
             command == 'H') {
        currentMovement = String(command);
    }
}

void updateTargetAngle(char rotationCommand) {
    switch (rotationCommand) {
        case 'T': targetAngle = 0; break;    // Forward
        case 'U': targetAngle = 180; break;  // Backward
        case 'V': targetAngle = 90; break;   // Right
        case 'W': targetAngle = 270; break;  // Left
        case 'X': targetAngle = 45; break;   // Top-right
        case 'Y': targetAngle = 135; break;  // Bottom-right
        case 'J': targetAngle = 225; break;  // Bottom-left
        case 'Z': targetAngle = 315; break;  // Top-left
    }
}

float getYawDegrees() {
    float yaw_degrees = yaw * RAD_TO_DEG;
    if (yaw_degrees < 0) {
        yaw_degrees = -fmod(-yaw_degrees, 360);
    } else {
        yaw_degrees = fmod(yaw_degrees, 360);
    }
    if (yaw_degrees < 0) yaw_degrees += 360;
    return yaw_degrees;
}

bool handleRotation() {
    float currentYaw = getYawDegrees();
    float angleDiff = targetAngle - currentYaw;
    
    // Normalize angle difference to -180 to 180
    if (angleDiff > 180) angleDiff -= 360;
    if (angleDiff < -180) angleDiff += 360;
    
    if (abs(angleDiff) <= ANGLE_TOLERANCE) {
        stopRobot();
        return true;  // Rotation complete
    }
    
    // Adjust rotation speed based on angle difference
    int rotSpeed = ROTATION_SPEED;
    if (abs(angleDiff) < 30) {
        rotSpeed = ROTATION_SPEED / 2;  // Slower rotation when close to target
    }
    
    if (angleDiff > 0) {
        rotateRight(rotSpeed);
    } else {
        rotateLeft(rotSpeed);
    }
    return false;  // Still rotating
}

void executeMovement(String movement) {
    if (movement == "F") moveForward();
    else if (movement == "D") moveForwardd();
    else if (movement == "B") moveBackward();
    else if (movement == "R") moveRightt();
    else if (movement == "P") moveLeft();
    else if (movement == "K") moveRight();
    else if (movement == "L") moveLeftt();
    else if (movement == "H") moveTopRight();
    else if (movement == "I") moveBottomRight();
    else if (movement == "G") moveTopLeft();
    else if (movement == "C") moveBottomLeft();
    else if (movement == "S") stopRobot();
}

// Movement functions
void moveForward() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveForwardd() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}
void moveBackward() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveRight() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}
void moveRightt() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveLeft() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}
void moveLeftt() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveTopRight() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveBottomRight() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveTopLeft() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void moveBottomLeft() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    
    analogWrite(ENA_m1, MOVEMENT_SPEED);
    analogWrite(ENB_m1, MOVEMENT_SPEED);
    analogWrite(ENA_m2, MOVEMENT_SPEED);
    analogWrite(ENB_m2, MOVEMENT_SPEED);
}

void rotateRight(int speed) {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    
    analogWrite(ENA_m1, speed);
    analogWrite(ENB_m1, speed);
    analogWrite(ENA_m2, speed);
    analogWrite(ENB_m2, speed);
}

void rotateLeft(int speed) {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, speed);
    analogWrite(ENB_m1, speed);
    analogWrite(ENA_m2, speed);
    analogWrite(ENB_m2, speed);
}

void stopRobot() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, LOW);
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, LOW);
    
    analogWrite(ENA_m1, 0);
    analogWrite(ENB_m1, 0);
    analogWrite(ENA_m2, 0);
    analogWrite(ENB_m2, 0);
}

void loop() {
    static unsigned long lastUltrasonicUpdate = 0;
    static unsigned long lastCommandCheck = 0;
    static unsigned long lastIMUUpdate = 0;
    
    const unsigned long ULTRASONIC_INTERVAL = 20;  // 20ms for sensor readings
    const unsigned long COMMAND_CHECK_INTERVAL = 10;  // 10ms for command checking
    const unsigned long IMU_UPDATE_INTERVAL = 10;  // 10ms for IMU updates
    
    unsigned long currentMillis = millis();
    
    // Update IMU data
    if (currentMillis - lastIMUUpdate >= IMU_UPDATE_INTERVAL) {
        updateSensorData();
        lastIMUUpdate = currentMillis;
    }
    
    // Read ultrasonic sensors
    if (currentMillis - lastUltrasonicUpdate >= ULTRASONIC_INTERVAL) {
        sendUltrasonicData();
        lastUltrasonicUpdate = currentMillis;
    }
    
    // Check for commands
    if (currentMillis - lastCommandCheck >= COMMAND_CHECK_INTERVAL) {
        if (Serial.available() > 0) {
            char command = Serial.read();
            processCommand(command);
        }
        lastCommandCheck = currentMillis;
    }
    
    // Handle movement and rotation
    if (isRotating) {
        if (handleRotation()) {
            isRotating = false;
            executeMovement(currentMovement);
        }
    } else {
        executeMovement(currentMovement);
    }
}

// Command queue functions
void addToQueue(char command) {
    int newRear = (queueRear + 1) % QUEUE_SIZE;
    if (newRear != queueFront) {
        commandQueue[queueRear] = command;
        queueRear = newRear;
    }
}

char getNextCommand() {
    if (queueFront == queueRear) {
        return 'S';  // Empty queue, return stop
    }
    char command = commandQueue[queueFront];
    queueFront = (queueFront + 1) % QUEUE_SIZE;
    return command;
}

// Emergency stop function
void emergencyStop() {
    stopRobot();
    isRotating = false;
    currentMovement = "S";
    
    // Clear command queue
    queueFront = queueRear = 0;
}
