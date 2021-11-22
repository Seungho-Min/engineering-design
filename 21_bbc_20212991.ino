#include <Servo.h>
// Arduino pin assignment

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _DUTY_MIN 1698
#define _DUTY_NEU 1476
#define _DUTY_MAX 1210
#define _SERVO_SPEED 150 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

unsigned long last_sampling_time;
float duty_chg_per_interval;
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
int a, b; // unit: mm
int duty_target, duty_curr;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
 
// initialize serial port
  Serial.begin(57600);

  a = 70;
  b = 270;
  
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
  
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  
  if(dist_cali > 255) {
    duty_curr += duty_chg_per_interval;
  }else{
    duty_curr -= duty_chg_per_interval;
  }
  myservo.writeMicroseconds(duty_curr);
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  delay(20);

  if(toggle_interval_cnt >= toggle_interval) {
    toggle_interval_cnt = 0;
    if(duty_target == _POS_START) duty_target = _POS_END;
    else duty_target = _POS_START;
  }
  else {
    toggle_interval_cnt++;
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}
