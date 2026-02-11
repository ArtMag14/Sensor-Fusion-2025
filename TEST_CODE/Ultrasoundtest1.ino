/****************************************************
 * USER PARAMETERS
 ****************************************************/
#define COUNTS_PER_REV        15
#define WHEEL_DIAMETER_CM     6.5
#define MAX_PWM              200
#define MIN_MOVING_PWM        60

#define SAFE_DISTANCE        30.0
#define WARNING_DISTANCE     60.0
#define SIDE_WARNING         25.0
#define BACK_SAFE            25.0

#define UPDATE_INTERVAL_MS   100

/****************************************************
 * MOTOR PINS (UNCHANGED)
 ****************************************************/
#define LEFT_PWM   4
#define LEFT_IN1   17
#define LEFT_IN2   16

#define RIGHT_PWM  5
#define RIGHT_IN1  18
#define RIGHT_IN2  7

/****************************************************
 * ENCODER PINS (UNCHANGED)
 ****************************************************/
#define LEFT_ENC_A   12
#define RIGHT_ENC_A  48

/****************************************************
 * ULTRASONIC PINS (UNCHANGED)
 ****************************************************/
struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
};

UltrasonicSensor sensorFront = {14, 15};
UltrasonicSensor sensorLeft  = {6, 3};
UltrasonicSensor sensorRight = {38, 39};
UltrasonicSensor sensorBack  = {8, 9};
//front right
//40,41
//front left
//8,9

/****************************************************
 * MOVEMENT STATES
 ****************************************************/
enum MovementState {
  FORWARD,
  PIVOT_LEFT,
  PIVOT_RIGHT,
  BACKWARD,
  STOPPED
};

MovementState currentState = STOPPED;

/****************************************************
 * OBSTACLE FLAGS
 ****************************************************/
int flagFront = 0;
int flagLeft  = 0;
int flagRight = 0;
int flagBack  = 0;

/****************************************************
 * DIRECTION TRACKING
 ****************************************************/
int leftDirection  = 0;  // -1 reverse, 0 stop, 1 forward
int rightDirection = 0;

/****************************************************
 * GLOBALS
 ****************************************************/
volatile long leftCount  = 0;
volatile long rightCount = 0;

long lastLeftCount  = 0;
long lastRightCount = 0;

float distanceMeters = 0.0;
float speedMps       = 0.0;

unsigned long lastUpdate = 0;

/****************************************************
 * ENCODER ISRs
 ****************************************************/
void IRAM_ATTR leftEncoderISR() { leftCount++; }
void IRAM_ATTR rightEncoderISR() { rightCount++; }

/****************************************************
 * MOTOR CONTROL
 ****************************************************/
void setMotor(int in1, int in2, int pwmPin, int pwm, bool isLeft) {

  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    if (isLeft) leftDirection = 1;
    else rightDirection = 1;
  }
  else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    if (isLeft) leftDirection = -1;
    else rightDirection = -1;
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    if (isLeft) leftDirection = 0;
    else rightDirection = 0;
  }

  analogWrite(pwmPin, abs(pwm));
}

void stopMotors() {
  setMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM, 0, true);
  setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, 0, false);
}

/****************************************************
 * ULTRASONIC
 ****************************************************/
float measureDistanceCM(UltrasonicSensor s) {
  digitalWrite(s.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trigPin, LOW);

  long duration = pulseIn(s.echoPin, HIGH, 30000);
  if (duration == 0) return 400.0;
  return (duration * 0.0343) / 2.0;
}

/****************************************************
 * UPDATE FLAGS
 ****************************************************/
void updateFlags(float front, float left, float right, float back) {

  flagFront = (front <= SAFE_DISTANCE) ? 2 :
              (front <= WARNING_DISTANCE) ? 1 : 0;

  flagLeft  = (left <= SIDE_WARNING) ? 2 :
              (left <= WARNING_DISTANCE) ? 1 : 0;

  flagRight = (right <= SIDE_WARNING) ? 2 :
              (right <= WARNING_DISTANCE) ? 1 : 0;

  flagBack  = (back <= BACK_SAFE) ? 2 :
              (back <= WARNING_DISTANCE) ? 1 : 0;
}

/****************************************************
 * ENCODER METRICS
 ****************************************************/
void updateEncoderMetrics() {

  long dL = abs(leftCount - lastLeftCount);
  long dR = abs(rightCount - lastRightCount);

  lastLeftCount  = leftCount;
  lastRightCount = rightCount;

  float avgCounts = (dL + dR) / 2.0;
  float wheelCircumference = PI * (WHEEL_DIAMETER_CM / 100.0);
  float distanceDelta = (avgCounts / COUNTS_PER_REV) * wheelCircumference;

  distanceMeters += distanceDelta;
  speedMps = distanceDelta / (UPDATE_INTERVAL_MS / 1000.0);
}

/****************************************************
 * FORWARD PWM SCALE
 ****************************************************/
int computeForwardPWM(float frontDist) {

  if (frontDist <= SAFE_DISTANCE) return 0;
  if (frontDist >= WARNING_DISTANCE) return MAX_PWM;

  float ratio = (frontDist - SAFE_DISTANCE) /
                (WARNING_DISTANCE - SAFE_DISTANCE);

  int pwm = MIN_MOVING_PWM +
            ratio * (MAX_PWM - MIN_MOVING_PWM);

  return constrain(pwm, MIN_MOVING_PWM, MAX_PWM);
}

/****************************************************
 * REACTIVE CONTROLLER
 ****************************************************/
void movementController(float front, float left, float right, float back) {

  switch (currentState) {

    case FORWARD:

      if (flagFront == 2) {
        currentState = (left > right) ? PIVOT_LEFT : PIVOT_RIGHT;
        return;
      }

      {
        int basePWM = computeForwardPWM(front);
        int leftPWM  = basePWM;
        int rightPWM = basePWM;

        if (flagLeft == 2) {
          leftPWM -= 40;
          rightPWM += 40;
        }
        else if (flagRight == 2) {
          leftPWM += 40;
          rightPWM -= 40;
        }

        setMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM, leftPWM, true);
        setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, rightPWM, false);
      }
      break;

    case PIVOT_LEFT:

      if (flagFront == 0) {
        currentState = FORWARD;
        return;
      }

      setMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM, -120, true);
      setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, 120, false);
      break;

    case PIVOT_RIGHT:

      if (flagFront == 0) {
        currentState = FORWARD;
        return;
      }

      setMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM, 120, true);
      setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, -120, false);
      break;

    case BACKWARD:

      if (flagBack == 2) {
        currentState = STOPPED;
        return;
      }

      setMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM, -100, true);
      setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, -100, false);
      break;

    case STOPPED:
      stopMotors();
      break;
  }
}

/****************************************************
 * PRINT STATE
 ****************************************************/
void printState() {

  const char* stateStr;

  switch (currentState) {
    case FORWARD:     stateStr = "FORWARD"; break;
    case PIVOT_LEFT:  stateStr = "PIVOT_LEFT"; break;
    case PIVOT_RIGHT: stateStr = "PIVOT_RIGHT"; break;
    case BACKWARD:    stateStr = "BACKWARD"; break;
    default:          stateStr = "STOPPED"; break;
  }

  const char* leftDirStr  = (leftDirection == 1) ? "FWD" :
                            (leftDirection == -1) ? "REV" : "STOP";

  const char* rightDirStr = (rightDirection == 1) ? "FWD" :
                            (rightDirection == -1) ? "REV" : "STOP";

  Serial.printf(
    "State:%s | L:%s R:%s | Flags F:%d L:%d R:%d B:%d | Dist:%.2f m | Speed:%.2f m/s\n",
    stateStr,
    leftDirStr,
    rightDirStr,
    flagFront,
    flagLeft,
    flagRight,
    flagBack,
    distanceMeters,
    speedMps
  );
}

/****************************************************
 * SETUP
 ****************************************************/
void setup() {

  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),
                  leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),
                  rightEncoderISR, RISING);

  pinMode(sensorFront.trigPin, OUTPUT);
  pinMode(sensorFront.echoPin, INPUT);
  pinMode(sensorLeft.trigPin, OUTPUT);
  pinMode(sensorLeft.echoPin, INPUT);
  pinMode(sensorRight.trigPin, OUTPUT);
  pinMode(sensorRight.echoPin, INPUT);
  pinMode(sensorBack.trigPin, OUTPUT);
  pinMode(sensorBack.echoPin, INPUT);

  currentState = FORWARD;
}

/****************************************************
 * LOOP
 ****************************************************/
void loop() {

  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) return;
  lastUpdate = millis();

  float dFront = measureDistanceCM(sensorFront);
  float dLeft  = measureDistanceCM(sensorLeft);
  float dRight = measureDistanceCM(sensorRight);
  float dBack  = measureDistanceCM(sensorBack);

  updateFlags(dFront, dLeft, dRight, dBack);
  updateEncoderMetrics();
  movementController(dFront, dLeft, dRight, dBack);
  printState();
}
