// Pin definitions for ultrasonic sensors
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

#define MAX_DISTANCE 25      // Maximum reading distance in cm
#define TOTAL_READINGS 60    // Total number of readings to take
#define READING_INTERVAL 1000 // 1 second interval

int readingCount = 0;
unsigned long lastReadingTime = 0;

void setup() {
    Serial.begin(230400);
    
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
    
    Serial.println("Starting 60 seconds of readings...");
    Serial.println("Time(s), Center(cm), FR(cm), FL(cm), L(cm), R(cm)");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if we should take a new reading
    if ((currentTime - lastReadingTime >= READING_INTERVAL) && (readingCount < TOTAL_READINGS)) {
        float distances[5];
        
        // Read all sensors
        distances[0] = readUltrasonicSensor(TRIG_PIN_CENTER, ECHO_PIN_CENTER);
        distances[1] = readUltrasonicSensor(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT);
        distances[2] = readUltrasonicSensor(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT);
        distances[3] = readUltrasonicSensor(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
        distances[4] = readUltrasonicSensor(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
        
        // Print reading number and all sensor values
        Serial.print(readingCount + 1);
        Serial.print("s, ");
        
        for (int i = 0; i < 5; i++) {
            Serial.print(distances[i], 1);
            if (i < 4) {
                Serial.print(", ");
            }
        }
        Serial.println();
        
        readingCount++;
        lastReadingTime = currentTime;
    }
    
    // Print completion message when done
    if (readingCount == TOTAL_READINGS) {
        Serial.println("Reading complete.");
        readingCount++; // Increment to prevent multiple messages
    }
}

float readUltrasonicSensor(int trigPin, int echoPin) {
    // Send ultrasonic pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Wait for echo with timeout
    unsigned long duration = pulseIn(echoPin, HIGH, 23529);  // Timeout for max distance
    
    if (duration == 0) {
        return MAX_DISTANCE;  // Return maximum distance if timeout
    }
    
    // Convert time to distance (in cm)
    float distance = (duration / 2.0) / 29.1;
    
    // Limit to maximum distance
    return min(distance, MAX_DISTANCE);
}