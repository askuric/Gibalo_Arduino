#include <Wire.h>
#include <I2Cdev.h>
#include <math.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the library to work with DMP (see comments inside)

// Comment-out the version
//#define GIBALO_V2_0 // Milneko
#define GIBALO_V2_1  // Faks

/**
 * The orientation of the accelerometer
 */
#define USE_ROLL  // Faks & Milenko
//#define USE_PICH 

//#define I2C_SPEED 100000L
#define I2C_SPEED 400000L
//#define I2C_SPEED 800000L

// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define Gyro_Gain 0.03048
#define Gyro_Scaled(x) x*Gyro_Gain //Return the scaled gyro raw data in degrees per second

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535
#define MAX_THROTTLE 1000
#define MAX_VELOCITY_LIN 1
#define MAX_STEERING 60
#define MAX_ACCEL_LIN 70.0  
#define MAX_VELOCITY 20.0*pi

#define MAX_ACCEL 100 // maximal delta Throttle


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;


long timer_old;
long timer_value;
float dt;

#define ON 1
#define OFF 0
// ON OFF constant
int state = ON;

// class default I2C address is 0x68
MPU6050 mpu;

float angle_adjusted;

#ifdef GIBALO_V2_0
  #define ZERO_ANGLE -1.5
#elif defined GIBALO_V2_1
  #define ZERO_ANGLE 4.8 
#else
  #define ZERO_ANGLE 0
#endif
float throttle;
float steering;
float target_speed[2];
float target_steering;
float control_output[2];
float d_control_output;

int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors
uint16_t counter_m[2];        // counters for periods
uint16_t period_m[2][8];      // Eight subperiods
uint8_t period_m_index[2];    // index for subperiods

// stepper motor driver pinout
#ifdef GIBALO_V2_0
  #define DIR1  8 // 9 
    #define DIR1_PORT   PORTB
    #define DIR1_NUM    0
  #define STEP1  7 // 8
    #define STEP1_PORT  PORTD
    #define STEP1_NUM   7
  #define ENABLE1  9 
  #define DIR2  5 
    #define DIR2_PORT PORTD
    #define DIR2_NUM  5
  #define STEP2  6 
    #define STEP2_PORT  PORTD
    #define STEP2_NUM  6
  #define ENABLE2  4 
  
  // dynamic microstepping - pinout
  #define  M2  11
  #define  M1  12
  #define  M0  13
  
#elif defined GIBALO_V2_1
#define DIR1   9 
    #define DIR1_PORT   PORTB
    #define DIR1_NUM    1
  #define STEP1   8
    #define STEP1_PORT  PORTB
    #define STEP1_NUM   0
  #define ENABLE1  7
  #define DIR2  6
    #define DIR2_PORT PORTD
    #define DIR2_NUM  6
  #define STEP2  5 
    #define STEP2_PORT  PORTD
    #define STEP2_NUM  5
  #define ENABLE2  4 
#endif

// motor ID contants
#define ML 0  // left motor ID
#define MR 1  // rigth motor ID
 

// Regulacijske varijable i konstante
#define k 2
#define pi  3.14159265359
#define Ts 5
#define Ta 0.008

// variable stanja
double dx[3] = {0};
double d_x[3]  = {0};
double e[3] = {0};
double a[2][3] = {0};
double w[2][3] = {0};
double dT = 0.0;
double N = 0;
double R = 0.043;
double theta[3] = {0};
double x[k]={0};

long count_xl = 0; 
long count_xr = 0;



// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution
float dmpGetPhi() {
  mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.resetFIFO();  // We always reset FIFO

#ifdef USE_ROLL
  return asin(-2 * (q.x * q.z - q.w * q.y)) * 180 / M_PI + ZERO_ANGLE; //roll 
#elif defined USE_PICH 
  return(atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD)+ ZERO_ANGLE; // pich
#endif
}




/**
 *
 * Funkcija koja ostvaruje PD regulator
 *
 * @param  (float) T - vrijeme uzorkovanja
 * @param  (float) inoput - nagib segway-a
 * @param  (float) setPoint - referenca nagiba
 * @param  (float) Kp, Ti, Td - Koeficienti PID regulatoar
 *
 * @return (float) N - skalirana upravljačka veličina
 */
float anglePDControl(float T, float input, float setPoint)
{
  static float e_k, e_k1, a_k1;
  float Kp = -23.5, Td = 0.122, N = 40.5;
  float Tf = Td/N;
  
  // greska regualcije
  e_k = setPoint - input;
  
  // izracunavanje akceleracije kao urpavljačke veličine
  // primitivan PD
  //a[k] =  Kp *e[k] + Kd * (e[k] - e[k-1])/T;
  // PDF
  float a_k = 1/(T+Tf)*(Tf*a_k1 + Kp*(Td+Tf+T)*e_k - Kp*(Tf+Td)*e_k1);
  
  e_k1=e_k;
  a_k1 = a_k;
  return a_k;
}

// P control implementation.
float speedPControl(float input, float setPoint)
{
  static float e_k;
  float Kp = -5.0;
  // controller error
  e_k = setPoint - input;
  
  // PI equation
  float a_k =Kp*e_k;

  return a_k;
}

/**
  Motor2 PI velocity regulator
  PI controller constants are defined in the "motor.h"
*/
float anglePIControl(float dT, float input, float setPoint){

  static float e_k, e_k1, a_k1;
  float Kp = -25.2, Ti = 0.358; 
  // controller error
  e_k = setPoint - input;
  
  // PI equation
  float a_k = a_k1 + Kp*(dT/(2*Ti)+1)*e_k + Kp*(dT/(2*Ti)-1)*e_k1;

  e_k1 = e_k;
  a_k1 = a_k;
  return a_k;
    
}

float a2N(float T,float a_k, int m){
  
  // zastita od namotavanja integratora
  a[m][k] = constrain(a_k, -MAX_ACCEL_LIN, MAX_ACCEL_LIN);

  //** PRILAGODBA SIGNALA **
  // kutna brzina
  w[m][k] = w[m][k - 1] + a[m][k] * T / R;
  
  // zastita od prevelike brzine
  w[m][k] = constrain(w[m][k], -MAX_VELOCITY, MAX_VELOCITY);

  // vrijeme između koraka
  float dT = pi / (100 *w[m][k]);
  // skalirani signal za izračun mikrosteping-a
  N = 1.0 / dT * 1000.0 / 30000.0 * 16.0 ; // 30Khz   1/16 koraka
  // zastita od prevelike brzine
  N = constrain(N, -MAX_THROTTLE, MAX_THROTTLE);
   
  a[m][k - 1] = a[m][k];
  w[m][k - 1] = w[m][k];
  
  return N;
}
float N2speed(float N){
  
  // vrijeme između koraka
  float dT = 1.0/N *1000.0/30000.0*16.0; // 30Khz   1/16 koraka
  
  float w = pi / (100 *dT);
  
  return w;
}

/**
 *
 * Funkcija koja ostvaruje LQR regulator
 *
 * @param  (float) T - vrijeme uzorkovanja
 * @param  (float) inoput - nagib segway-a
 * @param  (float) setPoint - referenca nagiba
 * @param  (float) Kp, Ti, Td - Koeficienti PID regulatoar
 *
 * @return (float) N - skalirana upravljačka veličina
 */
float angleLQRControl(float T, float input, float setPoint,int flag )
{

  if(flag){
    w[MR][k - 1] = 0;
    d_x[k-1] = 0;
    d_x[k-2] = 0;
    dx[k-1] = 0;
    x[k-1] = 0;
    theta[k-1] = 0;
    return 0;
  }
  //LQR konstante
  // dobar
  float Kr[4] = {31.0325,4.1054,3.698,2.1209};
 /**
   * Izracun varijabli stanja
   */
  // spremanje theta kuta
  theta[k] = input;
  // izracun derivacije theta
  float d_theta = (theta[k] - theta[k-1])/T;
  // izracun x
  x[k] = count_xr / 16.0 * pi / 100.0 * R ; // microstepping
  // izracun x derivacije
  dx[k] = w[MR][k] * R ;
    //   Tustin filtar a - time constant
  //d_x[k] = 1/(2*Ta+T)*((2*Ta-T)*d_x[k-1]+T*(dx[k]-dx[k-1]));  
  //d_x[k] =  0.2*w[k] * R * 1.3  +  0.4*(x[k]-x[k-1])/T + 0.2*d_x[k-1] + 0.2*d_x[k-2];
  d_x[k] = w[MR][k] * R;
  
  /**
   * LQR algoritam
   */
  // izracunavanje akceleracije kao urpavljačke veličine
  float a_k = Kr[0]*theta[k] + Kr[1]* d_theta + Kr[3]*x[k] + Kr[2]* d_x[k];
 
  // spremanje varijabli za sljedecu iteraciju
  d_x[k-1] = d_x[k];
  d_x[k-2] = d_x[k-1];
  dx[k-1] = dx[k];
  x[k-1] = x[k];
  theta[k-1] = theta[k];
  
  return a_k;
}


// 200ns => 4 instructions at 16Mhz
void delay_200ns()
{
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}

ISR(TIMER1_COMPA_vect)
{
  
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
  {
    counter_m[0] = 0;
    if (period_m[0][0] == ZERO_SPEED)
      return;
    if (dir_m[0]){
     // count_xl++;
      CLR(DIR1_PORT, DIR1_NUM); // 8
    }else{
     // count_xl--;
      SET(DIR1_PORT, DIR1_NUM);  //8
    }
    // We need to wait at lest 200ns to generate the Step pulse...
    period_m_index[0] = (period_m_index[0] + 1) & 0x07; // period_m_index from 0 to 7
    //delay_200ns();
    SET(STEP1_PORT, STEP1_NUM);   // 7
    delayMicroseconds(2);
    CLR(STEP1_PORT, STEP1_NUM); // 7
  }
  if (counter_m[1] >= period_m[1][period_m_index[1]])
  {
    counter_m[1] = 0;
    if (period_m[1][0] == ZERO_SPEED)
      return;
    if (dir_m[1]){ 
#ifdef GIBALO_V2_0     
      count_xr++;
#elif defined GIBALO_V2_1       
      count_xr--;
#endif
      SET(DIR2_PORT, DIR2_NUM); //  5
    }else{
#ifdef GIBALO_V2_0     
      count_xr--;
#elif defined GIBALO_V2_1       
      count_xr++;
#endif
      CLR(DIR2_PORT, DIR2_NUM);  // 5
    }
    period_m_index[1] = (period_m_index[1] + 1) & 0x07;
    //delay_200ns();
    SET(STEP2_PORT, STEP2_NUM);
    delayMicroseconds(2);
    CLR(STEP2_PORT, STEP2_NUM);
  }
}


// Dividimos en 8 subperiodos para aumentar la resolucion a velocidades altas (periodos pequeños)
// subperiod = ((1000 % vel)*8)/vel;
// Examples 4 subperiods:
// 1000/260 = 3.84  subperiod = 3
// 1000/240 = 4.16  subperiod = 0
// 1000/220 = 4.54  subperiod = 2
// 1000/300 = 3.33  subperiod = 1
void calculateSubperiods(uint8_t motor)
{
  int subperiod;
  int absSpeed;
  uint8_t j;

  if (speed_m[motor] == 0)
  {
    for (j = 0; j < 8; j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
  }
  if (speed_m[motor] > 0 )   // Positive speed
  {
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
  }
  else                       // Negative speed
  {
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
  }

  for (j = 0; j < 8; j++)
    period_m[motor][j] = 1000 / absSpeed;
  // Calculate the subperiod. if module <0.25 => subperiod=0, if module < 0.5 => subperiod=1. if module < 0.75 subperiod=2 else subperiod=3
  subperiod = ((1000 % absSpeed) * 8) / absSpeed; // Optimized code to calculate subperiod (integer math)
  if (subperiod > 0)
    period_m[motor][1]++;
  if (subperiod > 1)
    period_m[motor][5]++;
  if (subperiod > 2)
    period_m[motor][3]++;
  if (subperiod > 3)
    period_m[motor][7]++;
  if (subperiod > 4)
    period_m[motor][0]++;
  if (subperiod > 5)
    period_m[motor][4]++;
  if (subperiod > 6)
    period_m[motor][2]++;
}



void setMotorSpeed(uint8_t motor, int16_t tspeed)
{
  // WE LIMIT MAX ACCELERATION
  if ((speed_m[motor] - tspeed) > MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed) < -MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;

  calculateSubperiods(motor);  // We use four subperiods to increase resolution

  // To save energy when its not running...
  if ((speed_m[0] == 0) && (speed_m[1] == 0)){
    digitalWrite(ENABLE1, HIGH);  // Disbale motors
    digitalWrite(ENABLE2, HIGH);  // Disbale motors
  }
  else{
    digitalWrite(ENABLE1, LOW);  // Disbale motors
    digitalWrite(ENABLE2, LOW);  // Disbale motors
  }
}


void setup()
{

  // STEPPER PINS
  pinMode(ENABLE1, OUTPUT); // ENABLE MOTOR 1
  pinMode(ENABLE2, OUTPUT); // ENABLE MOTORS 2
  pinMode(DIR1, OUTPUT); // DIR MOTOR 1
  pinMode(DIR2, OUTPUT); // DIR MOTOR 2
  pinMode(STEP1, OUTPUT); // STEP MOTOR 1
  pinMode(STEP2, OUTPUT); //STEP MOTOR 2
  digitalWrite(ENABLE1, HIGH);  // Disbale motors
  digitalWrite(ENABLE2, HIGH);  // Disbale motors

#ifdef GIBALO_V2_0 
  pinMode(M0, OUTPUT); // DIR MOTOR 2
  pinMode(M1, OUTPUT); // STEP MOTOR 1
  pinMode(M2, OUTPUT); //STEP MOTOR 2
  digitalWrite(M0, HIGH);  // Disbale motors
  digitalWrite(M1, HIGH);  // Disbale motors
  digitalWrite(M2, HIGH);  // Disbale motors
#endif
  

  Serial.begin(115200);

  // Join I2C bus
  Wire.begin();
  // 4000Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
  TWCR = 1 << TWEN;

  Serial.println("Initializing I2C devices...");
  //mpu.initialize();

  
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);


  delay(2000);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling phi DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

  } else { // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Gyro calibration
  // The robot must be steady during initialization
  delay(5000);   // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 phi connection successful" : "MPU6050 phi connection failed");
  timer_old = millis();

  //We are going to overwrite the Timer1 to use the stepper motors

  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  //OCR1A = 200;
  //OCR1A = 150; //10Khz
  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  //OCR1A = 80;   // 25Khz
  OCR1A = 55;   // 30Khz
  TCNT1 = 0;

  delay(2000);

  //Adjust sensor fusion gain
  Serial.println("Adjusting DMP sensor fusion gain...");
  dmpSetSensorFusionAccelGain(0x20);

  Serial.println("Initializing Stepper motors...");
  delay(1000);
  digitalWrite(ENABLE1, LOW);   // ENABLE_PIN stepper drivers
  digitalWrite(ENABLE2, LOW);   // ENABLE_PIN stepper drivers

  mpu.resetFIFO();
  timer_old = millis();


  // Initial throttle and steering reference
  throttle = 0;
  steering = 0;
  
  Serial.println("d2x; w; dx; x; theta; time;");
  count_xr = 0;
  count_xl = 0;

  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  
  TIMSK1 |= (1 << OCIE1A); // ENABLE_PIN Timer1 interrupt
}


// Main loop
void loop() {


  timer_value = millis();
  
  // New DMP Orientation solution?
  fifoCount = mpu.getFIFOCount();
  
  if (mpu.getFIFOCount() >= 18 ) {

    if (mpu.getFIFOCount() > 18) { // If we have more than one packet we take the easy path: discard the buffer
      mpu.resetFIFO();
      return;
    }

    dt = (timer_value - timer_old);
    timer_old = timer_value;

    if (Ts - dt > 0 ) delay(Ts - dt);

    angle_adjusted = dmpGetPhi();
    
    mpu.resetFIFO();

   if ((angle_adjusted < 40) && (angle_adjusted > -40) && state)
    {				

      // LQR stabilisation 
      /*
      control_output[MR] = angleLQRControl(dt / 1000.0, angle_adjusted * pi / 180.0 ,throttle,0);
      control_output[ML] = control_output[MR];
      */
      
      // P+PD cascade speed control     
      // prefiltering
      target_steering = target_steering * 0.9 + steering * 0.1;
      target_speed[ML] = target_speed[ML] * 0.95 + (throttle) * 0.05;
      target_speed[MR] = target_speed[MR] * 0.95 + (throttle) * 0.05;
      
      // P(speed controller for each motor) 
      // separate control enables turning
      control_output[MR] = speedPControl(w[MR][k]*R, target_speed[MR]);
      control_output[ML] = control_output[MR];
      //PD stabilisaiton controller - same for botth motors 
      d_control_output = anglePDControl(dt / 1000.0, angle_adjusted * pi / 180.0,0);
      control_output[ML] += d_control_output;
      control_output[MR] += d_control_output;

      Serial.println(angle_adjusted);
      // calculate tick values
      control_output[ML] = a2N(dt / 1000.0,control_output[ML],ML);
      control_output[MR] = a2N(dt / 1000.0,control_output[MR],MR);

      // set speed to motors
#ifdef GIBALO_V2_0
      setMotorSpeed(ML, (int16_t)(-control_output[ML]-target_steering));
      setMotorSpeed(MR, (int16_t)(control_output[MR]-target_steering));
#elif defined GIBALO_V2_1
      setMotorSpeed(ML, (int16_t)(control_output[ML]-target_steering));
      setMotorSpeed(MR, (int16_t)(-control_output[MR]-target_steering));
#endif
      
    }
   else
    {
      //angleLQRControl(dt / 1000.0, angle_adjusted * pi / 180.0 ,throttle,1);
      setMotorSpeed(ML, 0);
      setMotorSpeed(MR, 0);
   }
  }
  readUserCommands();

}

/**
*   Function receiving throttle and steering reference
*   over Bluetooth Serial port
*
* Receives byte via Serial/Bluetooth port
*   and decodes it to throttle and steering reference
*
*/

void readUserCommands() {
  int inByte;
  if (Serial.available() > 0) {
    while (Serial.available()) {
      inByte = Serial.read();
    }
    inByte = inByte - 100;
    if (inByte == 155) {
      // ON Byte
      state = ON;
      Serial.println("Gibalo ON");
    } else if (inByte == 154) {
      // OFF Byte
      state = OFF;
      Serial.println("Gibalo OFF");
    } else if (inByte >= -100 && inByte <= 100 && state) {
      // throttle set-point Byte
      throttle = (float)MAX_VELOCITY_LIN *  ((float)inByte) / 100.0;
      Serial.print(throttle);
      Serial.print(" | ");
      Serial.println(steering);
    } else if (inByte >= 110 && inByte <= 150 && state) {
      // steering set-point Byte
      steering = (float)MAX_STEERING * ((float)(inByte - 130.0)) / 20.0; 
      Serial.print(throttle);
      Serial.print(" | ");
      Serial.println(steering);
    } else {
      // Error Byte
      steering = 0;
      throttle = 0;
      Serial.println("Error number!");
    }
    Serial.flush();
  }

}



