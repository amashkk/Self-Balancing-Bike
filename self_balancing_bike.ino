#include <Wire.h>
#define MPU6050       0x68         // Device address
#define ACCEL_CONFIG  0x1C         // Accelerometer configuration address
#define GYRO_CONFIG   0x1B         // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
#define BRAKE         8 
#define PWM           9
#define DIRECTION     7

const uint16_t PWM_FREQUENCY = 20000;                 
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;  

float X1 = 75.0; 
float X2 = 5.25;   //5.25
float X3 = 0.06;  
float loop_time = 10;  
int pwm_s = 0;
byte dir;
int32_t motor_speed; 
uint32_t timer;
long currentT, previousT_1 = 0; 
int16_t AcX, AcY, AcZ, GyZ, gyroZ;

#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     

int16_t  AcX_offset = -750;
int16_t  AcY_offset = 360;
int16_t  AcZ_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;
float alpha = 0.40; 
float gyroZfilt;
float robot_angle;
float Acc_angle;
bool vertical = false;  
uint8_t i2cData[14]; 

void setup() {
  Serial.begin(115200);
  // Set PWM frequency to 20kHz
  TCCR1B = (1 << WGM13) | (1 << CS10);  
  ICR1 = PWMVALUE;                      
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  setPWM(400); 
  
  Serial.print("PWM: "); Serial.println(PWMVALUE); 
  Serial.print("CPU_FREQ: "); Serial.println(F_CPU); 
  
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  angle_setup();
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning(); 
    angle_calc();
    if (vertical) {
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      pwm_s = -constrain(X1 * robot_angle + X2 * gyroZfilt + X3 * -motor_speed, -255, 255); 
      Motor_control(pwm_s);
      motor_speed += pwm_s;
    } else {
      Motor_control(0);
      digitalWrite(BRAKE, LOW);
      motor_speed = 0;
    }
    previousT_1 = currentT;
  }
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); 
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); 
  delay(100);
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
}

void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  
  GyZ = Wire.read() << 8 | Wire.read(); 
  
  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;
  
  robot_angle += GyZ * loop_time / 1000 / 65.536; 
  Acc_angle = atan2(AcY, -AcX) * 57.2958;       
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
  
  if (abs(robot_angle) > 9) vertical = false;
  if (abs(robot_angle) < 0.3) vertical = true;
}

void setPWM(uint16_t dutyCycle) {
    OCR1A = dutyCycle;
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIRECTION, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION, HIGH);
  }
  setPWM(map(pwm, 0, 255, PWMVALUE, 0));
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();              
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    X1 += 1;
      if (cmd == '-')    X1 -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+')    X2 += 0.01;
      if (cmd == '-')    X2 -= 0.01;
      printValues();
      break;
     case 's':
      if (cmd == '+')    X3 += 0.005;
      if (cmd == '-')    X3 -= 0.005;
      printValues();
      break;  
  }
}

void printValues() {
  Serial.print("X1: "); Serial.print(X1);
  Serial.print(" X2: "); Serial.print(X2);
  Serial.print(" X3: "); Serial.println(X3, 3);
}

// #include <Wire.h>
// #define MPU6050       0x68         // Device address
// #define ACCEL_CONFIG  0x1C         // Accelerometer configuration address
// #define GYRO_CONFIG   0x1B         // Gyro configuration address
// #define PWR_MGMT_1    0x6B
// #define PWR_MGMT_2    0x6C
// #define BRAKE         8 
// #define PWM           9
// #define DIRECTION     7

// const uint16_t PWM_FREQUENCY = 20000;                 
// const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;  

// float X1 = 75.0; 
// float X2 = 5.25;   
// float X3 = 0.04;  
// float loop_time = 10;  
// int pwm_s = 0;
// byte dir;
// int32_t motor_speed; 
// uint32_t timer;
// long currentT, previousT_1 = 0; 
// int16_t AcX, AcY, AcZ, GyZ, gyroZ;

// #define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
// #define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
// #define Gyro_amount 0.996     

// int16_t  AcX_offset = -750;
// int16_t  AcY_offset = 360;
// int16_t  AcZ_offset = 0;
// int16_t  GyZ_offset = 0;
// int32_t  GyZ_offset_sum = 0;
// float alpha = 0.40; 
// float gyroZfilt;
// float robot_angle;
// float Acc_angle;
// bool vertical = false;  
// uint8_t i2cData[14]; 

// float integral_error = 0.0;  // 用於積分項
// float previous_angle_error = 0.0;  // 用於計算微分項

// void setup() {
//   Serial.begin(115200);
//   // Set PWM frequency to 20kHz
//   TCCR1B = (1 << WGM13) | (1 << CS10);  
//   ICR1 = PWMVALUE;                      
//   TCCR1A = (1 << COM1A1) | (1 << COM1B1);
//   setPWM(400); 
  
//   Serial.print("PWM: "); Serial.println(PWMVALUE); 
//   Serial.print("CPU_FREQ: "); Serial.println(F_CPU); 
  
//   pinMode(PWM, OUTPUT);
//   pinMode(BRAKE, OUTPUT);
//   pinMode(DIRECTION, OUTPUT);
//   digitalWrite(BRAKE, HIGH);
//   delay(1000);
//   angle_setup();
// }

// void loop() {
//   currentT = millis();
//   if (currentT - previousT_1 >= loop_time) {
//     Tuning(); 
//     angle_calc();
//     if (vertical) {
//       digitalWrite(BRAKE, HIGH);
//       gyroZ = GyZ / 131.0; // Convert to deg/s
      
//       gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;

//       // 計算角度誤差、積分和微分
//       float angle_error = robot_angle;  
//       integral_error += angle_error * (loop_time / 1000.0);  
//       float derivative_error = (angle_error - previous_angle_error) / (loop_time / 1000.0);
//       previous_angle_error = angle_error;

//       // 使用PID控制公式
//       pwm_s = -constrain(X1 * angle_error + X2 * derivative_error + X3 * integral_error, -255, 255); 
//       Motor_control(pwm_s);
//       motor_speed += pwm_s;
//     } else {
//       Motor_control(0);
//       digitalWrite(BRAKE, LOW);
//       motor_speed = 0;
//       integral_error = 0;  // 停止時重置積分項
//     }
//     previousT_1 = currentT;
//   }
// }

// void writeTo(byte device, byte address, byte value) {
//   Wire.beginTransmission(device);
//   Wire.write(address);
//   Wire.write(value);
//   Wire.endTransmission(true);
// }

// void angle_setup() {
//   Wire.begin();
//   delay(100);
//   writeTo(MPU6050, PWR_MGMT_1, 0);
//   writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); 
//   writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); 
//   delay(100);
  
//   for (int i = 0; i < 1024; i++) {
//     angle_calc();
//     GyZ_offset_sum += GyZ;
//     delay(5);
//   }
//   GyZ_offset = GyZ_offset_sum >> 10;
//   Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
// }

// void angle_calc() {
//   Wire.beginTransmission(MPU6050);
//   Wire.write(0x3B);                  
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU6050, 4, true);
//   AcX = Wire.read() << 8 | Wire.read();
//   AcY = Wire.read() << 8 | Wire.read();
  
//   Wire.beginTransmission(MPU6050);
//   Wire.write(0x47);
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU6050, 2, true);  
//   GyZ = Wire.read() << 8 | Wire.read(); 
  
//   AcX += AcX_offset;
//   AcY += AcY_offset;  
//   AcZ += AcZ_offset;
//   GyZ -= GyZ_offset;
  
//   robot_angle += GyZ * loop_time / 1000 / 65.536; 
//   Acc_angle = atan2(AcY, -AcX) * 57.2958;       
//   robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
  
//   if (abs(robot_angle) > 9) vertical = false;
//   if (abs(robot_angle) < 0.3) vertical = true;
// }

// void setPWM(uint16_t dutyCycle) {
//     OCR1A = dutyCycle;
// }

// void Motor_control(int pwm) {
//   if (pwm <= 0) {
//     digitalWrite(DIRECTION, LOW);
//     pwm = -pwm;
//   } else {
//     digitalWrite(DIRECTION, HIGH);
//   }
//   setPWM(map(pwm, 0, 255, PWMVALUE, 0));
// }

// int Tuning() {
//   if (!Serial.available())  return 0;
//   delay(2);
//   char param = Serial.read();              
//   if (!Serial.available()) return 0;
//   char cmd = Serial.read();                
//   Serial.flush();
//   switch (param) {
//     case 'p':
//       if (cmd == '+')    X1 += 1;
//       if (cmd == '-')    X1 -= 1;
//       printValues();
//       break;
//     case 'i':
//       if (cmd == '+')    X2 += 0.01;
//       if (cmd == '-')    X2 -= 0.01;
//       printValues();
//       break;
//      case 's':
//       if (cmd == '+')    X3 += 0.005;
//       if (cmd == '-')    X3 -= 0.005;
//       printValues();
//       break;  
//   }
// }

// void printValues() {
//   Serial.print("X1: "); Serial.print(X1);
//   Serial.print(" X2: "); Serial.print(X2);
//   Serial.print(" X3: "); Serial.println(X3, 3);
// }
