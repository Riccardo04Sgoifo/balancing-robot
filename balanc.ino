
#include<Wire.h>

#include "WiFi.h"
#include "AsyncUDP.h"
#include "pid.h"

#include <PS4Controller.h>


#define FOR_PWM_PIN_1 5 // forward pin
#define REV_PWM_PIN_1 18 // reverse pin

#define FOR_PWM_PIN_2 4 // forward pin
#define REV_PWM_PIN_2 0 // reverse pin

#define IBT2_ENABLE_PIN 2 // enable


#define FOR_PWM_CHN_1 0 // forward ledc channel
#define REV_PWM_CHN_1 1 // reverse ledc channel

#define FOR_PWM_CHN_2 2 // forward ledc channel
#define REV_PWM_CHN_2 3 // reverse ledc channel


#define IBT2_PWM_FREQ 5000 // PWM frequency Hz
#define IBT2_PWM_RES 8 // PWM resolution bits


#define ENC_A_PIN_1 25 // encoder A pin
#define ENC_B_PIN_1 26 // encoder B pin

#define ENC_A_PIN_2 32 // encoder A pin
#define ENC_B_PIN_2 33 // encoder B pin

#define MPU6050_ADDRESS 0x68


// WiFi

const char * ssid = "sgf_net00";
const char * password = "Teteio#1";

bool is_WiFi = true; // should put a switch

AsyncUDP udp;

// mpu6050

int16_t Acx, Acy, Acz, Tmp, Gyx, Gyy, Gyz;
float Gyxf, Gyyf, Gyzf, Acxf, Acyf, Aczf;
double Acy_angle, Acx_angle, x_angle, y_angle;

int read_mpu6050_timer = 10000;
int read_mpu6050_time_interval = 10000;

// encoder

bool enc_1_A_state;
bool prev_enc_1_A_state;

bool enc_2_A_state;
bool prev_enc_2_A_state;


int counter_1 = 0;
int prev_counter_1 = 0;
float speed_1;
float smoothed_counter_1 = 0;

int counter_2 = 0;
int prev_counter_2 = 0;
float speed_2;
float smoothed_counter_2 = 0;

// time

long current_time;
long prev_time;
long prev_speed_time;
int dt; // delta time in us

// PID

//motor

float p_gain = 800.0f;
float i_gain = 0.0f;
float d_gain = 45.0f;

PID motor_pid_1(p_gain, i_gain, d_gain);
PID motor_pid_2(p_gain, i_gain, d_gain);

float set_point = -0.06f;
float error_1 = 0.0f;
float error_2 = 0.0f;

float output_1 = 0.0f;
float output_2 = 0.0f;

bool is_dead = false;


// position

float p_gain_pos = 0.5f;
float i_gain_pos = 0.0f;
float d_gain_pos = 0.6f;


PID position_pid_1(p_gain_pos, i_gain_pos, d_gain_pos);
PID position_pid_2(p_gain_pos, i_gain_pos, d_gain_pos);

float desidered_pos_1 = 0.0f;
float desidered_pos_2 = 0.0f;

float smoothed_desidered_pos_1 = 0.0f;
float smoothed_desidered_pos_2 = 0.0f;

float position_smoothing_factor = 0.95f;

float max_position_set_point_offset = 0.0f; // should scale down position pid contribute if angle gets too high aka near death mode

float set_point_offset_1 = 0.0f;
float set_point_offset_2 = 0.0f;

// ps4 dualshock

float controller_speed_y = 5.0f;
float controller_speed_x = 3.0f;

float controller_input_x = 0.0f;
float controller_input_y = 0.0f;


// linearization and scale

float lin_off_1 = 14.0f;
float lin_off_2 = 7.0f;

float scale_power_1 = 1.0f;
float scale_power_2 = 1.0f;


// test

int current_pwm = 0;
int change_speed_rate = 1;
int change_speed_timer = 100000;
int change_speed_time_interval = 100000;

int serial_send_timer = 100000;

int turn_on_timer = 0;


// tasks do not work with the PS4 library





void setup() {

// PWM setup

  pinMode(IBT2_ENABLE_PIN, OUTPUT);

  digitalWrite(IBT2_ENABLE_PIN, LOW);

  ledcSetup(FOR_PWM_CHN_1, IBT2_PWM_FREQ, IBT2_PWM_RES);
  ledcSetup(REV_PWM_CHN_1, IBT2_PWM_FREQ, IBT2_PWM_RES);

  ledcSetup(FOR_PWM_CHN_2, IBT2_PWM_FREQ, IBT2_PWM_RES);
  ledcSetup(REV_PWM_CHN_2, IBT2_PWM_FREQ, IBT2_PWM_RES);


  ledcAttachPin(FOR_PWM_PIN_1, FOR_PWM_CHN_1);
  ledcAttachPin(REV_PWM_PIN_1, REV_PWM_CHN_1);

  ledcAttachPin(FOR_PWM_PIN_2, FOR_PWM_CHN_2);
  ledcAttachPin(REV_PWM_PIN_2, REV_PWM_CHN_2);

  ledcWrite(FOR_PWM_CHN_1, 0);
  ledcWrite(REV_PWM_CHN_2, 0);
  ledcWrite(FOR_PWM_CHN_1, 0);
  ledcWrite(REV_PWM_CHN_2, 0);

  // serial

  Serial.begin(9600);

  // WiFi

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi failed!");
    while(1){
      Serial.println("-not ok-  :( ");
      delay(1000);
    }
  }

  // PS4 controller and task
  //xTaskCreatePinnedToCore(
  //                  PS4_task_code,   /* Task function. */
  //                  "PS4_task",     /* name of task. */
  //                  150000,       /* Stack size of task */
  //                  NULL,        /* parameter of the task */
  //                  1,           /* priority of the task */
  //                  &PS4_task,      /* Task handle to keep track of created task */
  //                  0);          /* pin task to core 0 */     
  PS4.begin("60:5b:b4:c2:a5:f6");
  
  
  // udp

  if (udp.listen(1234)){
    Serial.print("UDP ready on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      int message_length = packet.length();

      uint8_t var = packet.data()[0];
      uint8_t sign = packet.data()[1];
      uint32_t uvalue = 0;

      uvalue |= packet.data()[2];
      uvalue <<= 8; 
      uvalue |= packet.data()[3];
      uvalue <<= 8; 
      uvalue |= packet.data()[4]; 

      float value = (float)uvalue * ((sign) ? -1.0f : 1.0f) / 1000.0f;

      switch (var) { // set values

      case 0b0000:
        p_gain = value;
        motor_pid_1.set_p_gain(value);
        motor_pid_2.set_p_gain(value);
        break;

      case 0b0001:
        i_gain = value;
        motor_pid_1.set_i_gain(value);
        motor_pid_2.set_i_gain(value);
        break;

      case 0b0010:
        d_gain = value;      
        motor_pid_1.set_d_gain(value);
        motor_pid_2.set_d_gain(value);
        break;

      case 0b0011:
        set_point = value;
        break;
      
      case 0b0100:
        lin_off_1 = value;
        break;

      case 0b0101:
        lin_off_2 = value;
        break;
      
      case 0b0110:
        scale_power_1 = value;
        break;
      
      case 0b0111:
        scale_power_2 = value;
        break;

      case 0b1000:
        p_gain_pos = value;
        position_pid_1.set_p_gain(value);
        position_pid_2.set_p_gain(value);
        break;

      case 0b1001:
        i_gain_pos = value;
        position_pid_1.set_i_gain(value);
        position_pid_2.set_i_gain(value);
        break;

      case 0b1010:
        d_gain_pos = value;
        position_pid_1.set_d_gain(value);
        position_pid_2.set_d_gain(value);
        break;
      
      case 0b1011:
        counter_1 = 0;
        counter_2 = 0;
        desidered_pos_1 = 0;
        desidered_pos_2 = 0;
        break;

      case 0b1100:
        position_smoothing_factor = value;
        break;

      default:
        Serial.print("unrecognized :(");
      }

    });

    // enable motors

    

    digitalWrite(IBT2_ENABLE_PIN, HIGH);
  }


  // MPU6050 setup

  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // set limits (and precision)

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x1B);
  Wire.write(B00010000);
  Wire.endTransmission(true);


  Wire.beginTransmission(MPU6050_ADDRESS); // read initial accelerometer values
  Wire.write(0x3B); // start from ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true); // true ends transmission
  Acx = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Acy = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Acz = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Acxf = Acx;
  Acyf = Acy;
  Aczf = Acz;

  Acy_angle = atan2(Acxf, Aczf);
  Acx_angle = atan2(Acyf, Aczf);

  x_angle = Acx_angle;
  y_angle = Acy_angle;

  

  // encoder setup

  pinMode(ENC_A_PIN_1, INPUT);
  pinMode(ENC_B_PIN_1, INPUT);

  pinMode(ENC_A_PIN_2, INPUT);
  pinMode(ENC_B_PIN_2, INPUT);

  prev_enc_1_A_state = digitalRead(ENC_A_PIN_1);
  prev_enc_2_A_state = digitalRead(ENC_A_PIN_2);

  // time setup

  prev_time = micros();
  prev_speed_time = prev_time;
}



int read_controller_timer = 0;


void loop() {

  // read time and dt


  current_time = micros();
  dt = int(current_time - prev_time);
  prev_time = current_time;


  // read MPU6050 and calculate pid

  read_mpu6050_timer -= dt;

  if (read_mpu6050_timer <= 0){ 
    read_mpu6050_timer += read_mpu6050_time_interval;
    
    float interval_seconds = float(read_mpu6050_time_interval) / 1000000.0f;

    read_controller_timer++;
    if (read_controller_timer > 10){


      read_controller_timer = 0;
      if (PS4.isConnected()){
      
        controller_input_y = PS4.LStickY();
        controller_input_x = PS4.LStickX();


        if (abs(controller_input_y) < 10){
          controller_input_y = 0;
        }
        if (abs(controller_input_x) < 10){
          controller_input_x = 0;
        }

      
      }
    }
    
    desidered_pos_1 += controller_input_y * controller_speed_y * interval_seconds  -  controller_input_x * controller_speed_x * interval_seconds;
    desidered_pos_2 += controller_input_y * controller_speed_y * interval_seconds  +  controller_input_x * controller_speed_x * interval_seconds;
    

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B); // start from ACCEL_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true); // true ends transmission
    Acx = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    Acy = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    Acz = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    Gyx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    Gyy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    Gyz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    // apply offsets

    Gyx += 639 / 4;
    Gyy += 389 / 4;
    Gyz -= 429 / 4;
    Acx -= 580;
    Acy += 175;
    Acz -= 2345;

    Acxf = Acxf * 0.9f + float(Acx) * 0.1f;
    Acyf = Acyf * 0.9f + float(Acy) * 0.1f;
    Aczf = Aczf * 0.9f + float(Acz) * 0.1f;

    Gyxf = Gyxf * 0.8f + float(Gyx) * 0.2f;
    Gyyf = Gyyf * 0.8f + float(Gyy) * 0.2f;
    Gyzf = Gyzf * 0.8f + float(Gyz) * 0.2f;

    // extract angles from accelerometer

    Acy_angle = atan2(Acxf, Aczf);
    Acx_angle = atan2(Acyf, Aczf);

    // get higher precision using gyroscope

    x_angle = Acx_angle * 0.005 + (x_angle + (double)Gyxf * 0.000532113 * (double)read_mpu6050_time_interval / 1000000.0) * 0.995;
    y_angle = Acy_angle * 0.005 + (y_angle - (double)Gyyf * 0.000532113 * (double)read_mpu6050_time_interval / 1000000.0) * 0.995;

    // calculate pid

    // set_point_offset_1 = -position_pid.compute(float(counter_1) / 3000.0f, interval_seconds);


    smoothed_desidered_pos_1 = (smoothed_desidered_pos_1 * position_smoothing_factor) + desidered_pos_1 * (1.0f - position_smoothing_factor);
    smoothed_desidered_pos_2 = (smoothed_desidered_pos_2 * position_smoothing_factor) + desidered_pos_2 * (1.0f - position_smoothing_factor);

    smoothed_counter_1 = (smoothed_counter_1 * position_smoothing_factor) + (float)counter_1 * (1.0f - position_smoothing_factor);
    smoothed_counter_2 = (smoothed_counter_2 * position_smoothing_factor) + (float)counter_2 * (1.0f - position_smoothing_factor);

    set_point_offset_1 = position_pid_1.compute(float(smoothed_desidered_pos_1 - smoothed_counter_1) / 3000.0f, interval_seconds);
    set_point_offset_2 = position_pid_2.compute(float(smoothed_desidered_pos_2 - smoothed_counter_2) / 3000.0f, interval_seconds);


    error_1 = (set_point - set_point_offset_1) - y_angle;
    error_2 = (set_point - set_point_offset_2) - y_angle;
    //error_1 = (set_point) - y_angle;
    //error_2 = (set_point) - y_angle;
    

    output_1 = motor_pid_1.compute(error_1, interval_seconds);
    output_2 = motor_pid_2.compute(error_2, interval_seconds);

    // "fall protection"

    if ((abs(error_1) > 0.9f) || (abs(error_2) > 0.9f)){
      output_1 = 0;
      output_2 = 0;
      digitalWrite(IBT2_ENABLE_PIN, LOW);
    }
      
    
    output_1 = -output_1; // reverse!
    output_2 = -output_2; // reverse!
    
    
    turn_on_timer += read_mpu6050_time_interval;
    if (turn_on_timer < 300000.0f){
      output_1 = (float)turn_on_timer / 300000.0f;
      output_2 = (float)turn_on_timer / 300000.0f;
    }
    
    output_2 *= scale_power_2;
    output_1 *= scale_power_1;

    


    // run motors


    if (output_1 >= 0.0f){
      output_1 = min(output_1, 255.0f);
      ledcWrite(FOR_PWM_CHN_1, linearization_function_1((int)output_1));
      ledcWrite(REV_PWM_CHN_1, 0);
    }
    else{
      output_1 = max(output_1, -255.0f);
      ledcWrite(REV_PWM_CHN_1, linearization_function_1(-(int)output_1));
      ledcWrite(FOR_PWM_CHN_1, 0);
    }

    if (output_2 >= 0.0f){
      output_2 = min(output_2, 255.0f);
      ledcWrite(FOR_PWM_CHN_2, linearization_function_2((int)output_2));
      ledcWrite(REV_PWM_CHN_2, 0);
    }
    else{
      output_2 = max(output_2, -255.0f);
      ledcWrite(REV_PWM_CHN_2, linearization_function_2(-(int)output_2));
      ledcWrite(FOR_PWM_CHN_2, 0);
    }


  }
  
    

  // read encoder

  enc_1_A_state = digitalRead(ENC_A_PIN_1);
  
  if (enc_1_A_state != prev_enc_1_A_state){
    if (digitalRead(ENC_B_PIN_1) != enc_1_A_state){ // could be optimizad by reading the pin b only if velocity is low
      counter_1 ++;
    }
    else{
      counter_1 --;
    }
  }


  enc_2_A_state = digitalRead(ENC_A_PIN_2);

  if (enc_2_A_state != prev_enc_2_A_state){
    if (digitalRead(ENC_B_PIN_2) != enc_2_A_state){ // could be optimizad by reading the pin b only if velocity is low
      counter_2 ++;
    }
    else{
      counter_2 --;
    }
  }

  prev_enc_1_A_state = enc_1_A_state;
  prev_enc_2_A_state = enc_2_A_state;
 
}

int linearization_function_1(int value){
  return int(float(value + lin_off_1) / (255.0f+lin_off_1) * 255.0f);
}

int linearization_function_2(int value){
  return int(float(value + lin_off_2) / (255.0f+lin_off_2) * 255.0f);
}






























