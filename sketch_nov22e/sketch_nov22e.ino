// Motor Pins
#define enA 9     // Right Motor Enable Pin
#define in1 5     // Right Motor in1
#define in2 6     // Right Motor in2
#define enB 3     // Left Motor Enable Pin
#define in3 10    // Left Motor in3
#define in4 11    // Left Motor in4

// Sensor Pins (connected to digital pins)
#define ir1 2     // Leftmost Sensor
#define ir2 7     // Left Sensor
#define ir3 8     // Middle Sensor
#define ir4 12    // Right Sensor
#define ir5 13    // Rightmost Sensor

// Speed and PID variables
int baseSpeed = 100;  // Base speed for motors
int maxSpeed = 140;   // Maximum motor speed

float Kp = 10.0;   // Increased Proportional constant for better correction
float Ki = 0.9;    // Slightly lower Integral constant to avoid drifting
float Kd = 5.0;    // Lower Derivative constant to reduce oscillation

int previousError = 0;
float integral = 0;
int lastKnownError = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  Serial.begin(9600);
}

void loop() {
  int s1 = digitalRead(ir1) == LOW ? 1 : 0;
  int s2 = digitalRead(ir2) == LOW ? 1 : 0;
  int s3 = digitalRead(ir3) == LOW ? 1 : 0;
  int s4 = digitalRead(ir4) == LOW ? 1 : 0;
  int s5 = digitalRead(ir5) == LOW ? 1 : 0;

  // Calculate error
  int weights[5] = { -2, -1, 0, 1, 2 };
  int error = 0, totalActiveSensors = 0;
  int sensors[5] = { s1, s2, s3, s4, s5 };

  for (int i = 0; i < 5; i++) {
    error += sensors[i] * weights[i];
    totalActiveSensors += sensors[i];
  }
  // If no sensor detects the line, use last known error for turning
  if (totalActiveSensors == 5) {
    // Stop the motors when all sensors detect black
    setMotors(0, 0);
    return;  // Exit the loop to avoid further processing
  }
  if (totalActiveSensors == 0) {
    if (lastKnownError < 0) {
      // Strong left turn (more aggressive)
      setMotors(-maxSpeed, maxSpeed);
    } 
    else if (lastKnownError > 0) {
      // Strong right turn (more aggressive)
      setMotors(maxSpeed, -maxSpeed);
    } 
    else {
      // If no direction is known, slightly turn right
      setMotors(-baseSpeed, baseSpeed);
    }
    return;
  }

  error /= totalActiveSensors;
  lastKnownError = error;

  // PID Control
  float proportional = error * Kp;
  integral = constrain(integral + error, -50, 50);
  float integralTerm = integral * Ki;
  float derivative = (error - previousError) * Kd;
  int pid = proportional + integralTerm + derivative;

  previousError = error;

  // Adjust speed dynamically based on error
  int dynamicBaseSpeed = map(abs(error), 0, 2, baseSpeed, baseSpeed / 2);
  int leftSpeed = constrain(dynamicBaseSpeed + pid, 0, maxSpeed);
  int rightSpeed = constrain(dynamicBaseSpeed - pid, 0, maxSpeed);

  setMotors(leftSpeed, rightSpeed);

  // Debugging Output
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print(" | PID: ");
  // Serial.print(pid);
  // Serial.print(" | Left: ");
  // Serial.print(leftSpeed);
  // Serial.print(" | Right: ");
  // Serial.println(rightSpeed);
}

void setMotors(int leftSpeed, int rightSpeed) {
  analogWrite(enA, abs(rightSpeed));
  digitalWrite(in1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(in2, rightSpeed > 0 ? LOW : HIGH);

  analogWrite(enB, abs(leftSpeed));
  digitalWrite(in3, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(in4, leftSpeed > 0 ? LOW : HIGH);
}