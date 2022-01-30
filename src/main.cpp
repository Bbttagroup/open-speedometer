#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// wheel circumference in mm
#define WHEEL_CIRCUMFERENCE 1590

// servo angle in degrees
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_ANGLE 1
#define SERVO_PIN 9

// calculation cycle time in miliseconds
#define CALC_CYCLE_TIME 800L

// calculation cycle detections count
#define CALC_CYCLE_COUNT 10

#define MAGNET_COUNT 4

// time constants
#define MINUTE 60000L
#define SECONDS_IN_MINUTE 60
#define RPM_MINUTE_CONVERSION MINUTE / MAGNET_COUNT

// length constants
#define MM_IN_METER 1000L
#define KM_CONVERSION 3.6

#define MAX_RPM 900
#define SPEED_LIMIT 80

// speedometer servo instance
Servo servo;

// magnet decection dounter per cycle
volatile byte detections;

unsigned int speed = 0;
bool speed_updated = false;

unsigned long speed_timeold = 0;
unsigned long cycle_timeold = 0;

// used to increment variable for rotations per cycle
void magnet_detect();
void calculate_speed_rpm();
void calculate_speed(double rpm);
void show_speed();

void setup()
{
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  servo.write(90);
  delay(200);
  servo.write(SERVO_MAX_ANGLE);
  delay(200);
  Serial.println("Hello");

  attachInterrupt(digitalPinToInterrupt(2), magnet_detect, RISING); // Initialize the intterrupt pin (Arduino digital pin 2)

  detections = 0;
  speed_timeold = 0;
}

void loop()
{
  calculate_speed_rpm();
  show_speed();
}

void magnet_detect() // This function is called whenever a magnet/interrupt is detected by the arduino
{
  detections++;
  // Serial.println("Detect");
}

void calculate_speed_rpm()
{
  unsigned long current_time = millis();
  if (detections >= CALC_CYCLE_COUNT || (current_time-speed_timeold) >= CALC_CYCLE_TIME)
  {
    noInterrupts();
    double speed_rpm = detections * long(RPM_MINUTE_CONVERSION) / (current_time-cycle_timeold);
    detections = 0;
    interrupts();

    Serial.print("RPM\t");
    Serial.println(speed_rpm);

    speed_timeold = current_time;
    cycle_timeold = current_time;

    calculate_speed(speed_rpm);
  }
}

void calculate_speed(double rpm)
{
  float rps = rpm / SECONDS_IN_MINUTE;
  float speed_meters_per_second = rps * WHEEL_CIRCUMFERENCE / MM_IN_METER;
  speed = ceil(speed_meters_per_second * KM_CONVERSION);
  Serial.print("Speed\t");
  Serial.print(speed_meters_per_second * KM_CONVERSION);
  Serial.print("\t");
  Serial.println(speed);
  speed_updated = true;
}

void show_speed()
{
  if (speed_updated)
  {
    int servo_val = map(speed, 0, SPEED_LIMIT, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE);
    Serial.println(servo_val);
    servo.write(servo_val);
    speed_updated = false;
  }
}
