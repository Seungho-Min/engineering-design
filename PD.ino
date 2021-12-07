#include <Servo.h>


/////////////////////////////

// Configurable parameters //

/////////////////////////////



// Arduino pin assignment

#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결

#define PIN_SERVO 10 // [1352] 서보모터를 아두이노의 10번 핀에 연결

#define PIN_IR A0 // [1352] IR센서를 아두이노 A0 핀에 연결



// Framework setting

#define _DIST_TARGET 255 // [1352]목표값을 탁구공 중심 위치까지 거리 255mm로 Fix

#define _DIST_MIN 100 // [1352] 최소 측정 거리를 100mm로 설정

#define _DIST_MAX 410 // [1352] 측정 거리의 최댓값을 410mm로 설정



// Distance sensor

#define _DIST_ALPHA 0.9 // [2979] 알파값 설정




// Servo range

#define _DUTY_MIN 1550 //[2998] Servo Minimum microseconds

#define _DUTY_NEU 1490 //[2983] Servo 중간값 설정

#define _DUTY_MAX 1100 // [2979] Servo Max값 설정



// Servo speed control

#define _SERVO_ANGLE 30 // [2992] 서보 각 설정

#define _SERVO_SPEED 20 // [2976] 서보 스피드 설정



// Event periods

#define _INTERVAL_DIST 10 

#define _INTERVAL_SERVO 5 // [2980] 서보 INTERVAL값 설정

#define _INTERVAL_SERIAL 100 //[2989] 시리얼 출력 INTERVAL 설정



// PID parameters

#define _KP 1.5 // [3000] K🇵값 초기화

#define _KI  // [3000] K¡값 초기화

#define _KD 1.5 // [3000] Kd값 초기화


//////////////////////

// global variables //

//////////////////////



// Servo instance

Servo myservo; //[2991] create servo object to control a servo


// Distance sensor

float dist_target; // location to send the ball [2976] 공 위치

float dist_raw, dist_ema, dist_prev, dist_min, dist_max;// [2981] 거리 변수 설정(현재, ema 필터 적용, 직전값)



// Event periods

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial, time_curr; 

bool event_dist, event_servo, event_serial; //[2989] 논리형 변수 설정


// Servo speed control

int duty_chg_per_interval; //[1352] 주기동안 duty 변화량 변수 

int duty_target, duty_curr;//[1352] 서보모터의 목표위치, 서보에 실제 입력할 위치



// PID variables

float error_curr, error_prev, control, pterm, dterm, iterm, error;



//[2998] initialize variables for Low Pass Filter

unsigned long oldmil; //[2998] old milliseconds var

static long apt = 0;  //[2998] filter sampling time saver

float filter = 0.0, prev = 0.0; //[2998] setup for filter and prev var’s


int a, b;

void setup() {

   // initialize GPIO pins for LED and attach servo

   pinMode(PIN_LED, OUTPUT); //[1352] LED 핀 설정

   // myservo.attach(PIN_SERVO); //[1352] Servo 핀 설정 

   // [2998] servo attach duplicated

   // move servo to neutral position

   myservo.attach(PIN_SERVO); //[2998] Servo Attaching 

   myservo.writeMicroseconds(_DUTY_NEU); //[2998] Servo Set to neutral position


   // initialize global variables

   dist_min = _DIST_MIN; //[2999] dist_min 값 적용

   dist_max = _DIST_MAX; //[2999] dist_max 값 적용

   dist_ema = 0.0; //[2999] dist_ema 값 초기화

   float alpha = _DIST_ALPHA; //[2999] alpha 선언 및 값 적용

   // convert angle speed into duty change per interval.
   duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_DIST / 1000); //[2985] duty_chg_per_interval 설정

   // initialize serial port

   Serial.begin(57600); //[2999] serial port 57600

   a = 50;
   b = 240;
}

 
void loop() {

/////////////////////

// Event generator //

/////////////////////
    time_curr = millis();

    if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST){

      last_sampling_time_dist += _INTERVAL_DIST;

      event_dist = true;

    }

    if (time_curr >= last_sampling_time_dist + _INTERVAL_SERVO){

      last_sampling_time_servo += _INTERVAL_SERVO;

      event_servo = true;

    }

    if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){

      last_sampling_time_serial += _INTERVAL_SERIAL;

      event_serial = true;

    }

////////////////////

// Event handlers //

////////////////////


   if(event_dist) {
    
      event_dist = false; 
      // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered();

  // PID control logic

      error_curr= _DIST_TARGET - dist_raw;
      pterm = _KP*error_curr; 
      dterm = _KD * (error_curr - error_prev);
      control = dterm + pterm;
//     control = dterm;
      duty_target = _DUTY_NEU + control; 
   
      if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
      if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
//      else if(duty_target >= _DUTY_MIN && duty_target < _DUTY_NEU) duty_target += duty_chg_per_interval;
//      else if(duty_target >= _DUTY_NEU && duty_target < _DUTY_MAX) duty_target += duty_chg_per_interval;
//      else if (duty_target > _DUTY_MAX) { //[1352] 조건일때 duty_target값을 _DUTY_MAX으로
//         duty_target = _DUTY_MAX - duty_chg_per_interval;
     error_prev = error_curr;
      
   }

   if(event_servo) {

      event_servo = false; //[1352]server Event handler가 false
      // adjust duty_curr toward duty_target by duty_chg_per_interval
      duty_curr = 1485 + error_curr*0.75;
      if(duty_target > _DUTY_NEU) duty_curr -= duty_chg_per_interval;
      if(duty_target < _DUTY_NEU) duty_curr += duty_chg_per_interval;
      myservo.writeMicroseconds(duty_curr); 
    
  }


  if(event_serial) {

     event_serial = false;

     Serial.print("dist_ir:");

     Serial.print(dist_raw);
     
     Serial.print(",pterm:"); 
     
     Serial.print(map(pterm,-1000,1000,510,610));

     Serial.print(",dterm:"); 

     Serial.print(map(dterm,-1000,1000,510,610));

     Serial.print(",duty_target:"); 

     Serial.print(map(duty_target,1000,2000,410,510));

     Serial.print(",duty_curr:"); 
     
     Serial.print(map(duty_curr,1000,2000,410,510));

     Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    }
}


float ir_distance(void){ // return value unit: mm

    float val; //[2998] Announce val 

    float volt = float(analogRead(PIN_IR)); //[2998] Read analog data from PIN_IR

    val = ((6762.0/(volt-9.0))-4.0) * 10.0; 
    
    return val;
}


float ir_distance_filtered(void){ // return value unit: mm
   float raw_dist = ir_distance();
   float dist_cali = 100.0 + 300.0 / (b - a) * (raw_dist - a);
   float a = (_DIST_ALPHA*dist_cali)+((1-_DIST_ALPHA)*dist_ema);
   if((_DIST_ALPHA*dist_cali)+((1-_DIST_ALPHA)*dist_ema) > 410) a = 410;
   if((_DIST_ALPHA*dist_cali)+((1-_DIST_ALPHA)*dist_ema) < 100) a = 100;
   return a;
}
