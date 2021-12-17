#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

#define interval 3 // milliseconds
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10 // [1352] 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0 // [1352] IR센서를 아두이노 A0 핀에 연결


// Framework setting
#define _DIST_TARGET 255  // [1352]목표값을 탁구공 중심 위치까지 거리 255mm로 Fix
#define _DIST_MIN 100 // [1352] 최소 측정 거리를 100mm로 설정
#define _DIST_MAX 410 // [1352] 측정 거리의 최댓값을 410mm로 설정

// Distance sensor
#define _DIST_ALPHA 0.65// [2979] 알파값 설정

// Servo range
#define _DUTY_MIN 1210    // //[2998] Servo Minimum microseconds
#define _DUTY_NEU 1490   //[2983] Servo 중간값 설정
#define _DUTY_MAX 1698       // [2979] Servo Max값 설정


// Servo speed control
#define _SERVO_ANGLE 40 // [2992] 서보 각 설정
#define _SERVO_SPEED 750 // [2976] 서보 스피드 설정
#define _R_TIME 30

// Event periods
#define _INTERVAL_DIST 10   // [2987] 거리측정 INTERVAL값 설정
#define _INTERVAL_SERVO 20  // [2980] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100  //[2989] 시리얼 출력 INTERVAL 설정
#define a 30
#define b 330

// PID parameters
float Kp = 1.7;       
float Kd = 75; 
float Ki = 0.015;

//////////////////////
// global variables //
//////////////////////
static long apt = 0;
unsigned long oldmil;
double Time = 0.3;
int fc = 3;
float dt = interval / 1000.0;
float lambda = 2 * PI * fc * dt;
float calidist = 0.0, filter = 0.0, prev = 0.0;

// Servo instance
Servo myservo;  //[2991] create servo object to control a servo
double controls = 0;
// Distance sensor
float dist_target; // location to send the ball [2976] 공 위치
float dist_raw, dist_ema, dist_cali; // [2981] 거리 변수 설정(현재, ema 필터 적용, 직전값)
float alpha;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial; //[2989] 논리형 변수 설정

double last_error;
// Servo speed control
int duty_chg_per_interval, duty_per_max ; // current speed (unit: us/interval)
int duty_adjust; //[1352] 주기동안 duty 변화량 변수
int duty_target, duty_curr;//[1352] 서보모터의 목표위치, 서보에 실제 입력할 위치
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);

  // move servo to neutral position
  myservo.attach(PIN_SERVO); //[2998] Servo Attaching
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);//[2998] Servo Set to neutral position


  // initialize global variables
  duty_curr = _DUTY_MIN; //[2999] dist_min 값 적용
  dist_target = _DIST_TARGET;
  alpha = _DIST_ALPHA; //[2999] alpha 선언 및 값 적용
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;


  // initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval.
  duty_per_max = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_DIST / 1000); //[2985] duty_chg_per_interval 설정
  duty_adjust = (float) duty_per_max * _INTERVAL_DIST / _R_TIME;
  duty_chg_per_interval = 0;
}


void loop() {
  /////////////////////
  // Event generator //
  /////////////////////

  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    event_dist = false;
    dist_ema = ir_distance_filtered();

    error_curr = dist_target - dist_ema;
    pterm = Kp * error_curr ;
    dterm = Kd * (error_curr - last_error) / Time;
    iterm += Ki * (error_curr) * Time;
    control = dterm + pterm + iterm;
    duty_target = _DUTY_NEU + control;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; 
    last_error = error_curr;
  }

  if (event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target < duty_curr) {
      if (duty_chg_per_interval > -duty_per_max) {
        duty_chg_per_interval -= duty_chg_adjust;
        if (duty_chg_per_interval < -duty_per_max) duty_chg_per_interval = -duty_per_max;
      }
      duty_curr += duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }
    else if (duty_target > duty_curr) {
      if (duty_chg_per_interval < duty_per_max) {
        duty_chg_per_interval += duty_chg_adjust;
        if (duty_chg_per_interval > duty_per_max) duty_chg_per_interval = duty_per_max;
      }
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }

    
    else {
      duty_chg_per_interval = 0;
    }
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);

  }
  if (event_serial) {
    event_serial = false; 
    
    Serial.print("IR:"); 
    
    Serial.print(dist_ema);
    
    Serial.print(",T:"); 
    
    Serial.print(dist_target);
    
    Serial.print(",P:");
    
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    
    Serial.print(",D:");
    
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    
    Serial.print(",I:"); 
    
    Serial.print(map(iterm, -1000, 1000, 510, 610));
    
    Serial.print(",DTT:");
    
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
     
    Serial.print(",DTC:"); 
    
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
     
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void) { // return value unit: mm
  float dist = ir_distance(); //[2998] calling distance from ir_distance()
  unsigned long dmil = 0; //[2998] return previous distance
  unsigned long mil = millis();  //[2998] get current time

  if (mil != oldmil) { //[2998] set time difference on dmil for checking previous time
    dmil = mil - oldmil;
    oldmil = mil;
  }

  apt -= dmil; //[2998] set difference for dmil and apt time

  if (apt <= 0) {  //[2998] check time before calculating filtered time
    apt += interval;
    filter = lambda / (1 + lambda) * dist + 1 / (1 + lambda) * prev;
    prev = filter; //[2998] Sensor filter previous  value update
  }
  float calidist = 100.0 + 300.0 / (b - a) * (filter - a);
  return calidist;
}
