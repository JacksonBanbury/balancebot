
/*

  Balacining Robot Arduino code by Jackson Banbury, with help from https://github.com/TKJElectronics/KalmanFilter

  Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
  function to get motors going in either CW, CCW, BRAKEVCC, or
  BRAKEGND. Use motorOff(int motor) to turn a specific motor off.

  The motor variable in each function should be either a 0 or a 1.
  pwm in the motorGo function should be a value between 0 and 255.
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

//--------------These are the parameters to adjust the MPU6050 readings ---------------------------------//
int buffersize = 10;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 2;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;


// -------------These are the initializations for the self_balancing_bot PID code -----------------------//
float angle = 0;
float multi = 0.6;                                        // This is the multiplication factor for the PID parameters
double Kp = 12 * multi, Ki = 1 * multi, Kd = 7 * multi;  // These are the PID parameters!
double Setpoint = 0, Input, Output;                       // To be used for PID
float angle_accln;                                        // acceleration due to angle
long t1 = 0;                                              // variable to store time in milliseconds

PID MotorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float dt1, anv1, angle1 = 0, an1;

int statpin = 13;


//--------------function for finding angle (Complementary Filter)---------------//

float find_angle(long tt){

             // Calling getMotion6 method to find raw values of all 6 components
  
  angle_accln=(double(mean_ay));                   // Angle calculated from orientation of acceleration
      
  dt1=float(millis()-tt)/1000;                                        // finding 'dt' in seconds
  anv1=(float(mean_gx))/131;                                              // angular velocity from Gyroscope
  angle1=angle1+(anv1*dt1);                                           // Angle calculated from gyroscope measurements
  
  an1=((0.96)*(an1+(anv1*dt1)))+(angle_accln*0.04);                   // COMPLEMENTARY FILTER IMPLEMENTATION
  Serial.println("Angle:");
  Serial.println(an1);
  return an1;                                                         // returns the filtered angle
}

//------------MPU6050 Functions-------------------------------------------------------------//
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  accelgyro.setXAccelOffset(-3857);
  accelgyro.setYAccelOffset(1563);
  accelgyro.setZAccelOffset(5296);

  accelgyro.setXGyroOffset(-2);
  accelgyro.setYGyroOffset(6);
  accelgyro.setZGyroOffset(-4);
  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
   // delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

//---------------------------------Motor Shield Functions-------------------------------------------//
void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either 0 or 1, will selet which of the two
  motors to be controlled

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between ? and 1023, higher the number, the faster
  it'll go
*/
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void setup()
{
  Serial.begin(9600);

  // ---------- MPU6050 Setup-----------------------------------------------------------------------------//
  Wire.begin();

  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  accelgyro.initialize();

  // reset IMU offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  //------------Motor Shield Setup-----------------------------------------------------------------------//

  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  // motorGo(0, CW, 1023);
  // motorGo(1, CCW, 1023);
}
int pwm = 6000;
void loop()
{

  //-----MPU6050 Calibration - These delays help it do a better job of calibrating, but make for a slow startup------//

  if (state == 0) {
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state == 1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state == 2) {
    meansensors(); //This gets the mean readings from the sensors, which can be called my parts
    //For this case, mean_ay and mean_gy will be the most useful
  }

  //-----------------self_balancing_bot motor control loop---------------------------------//
  angle = find_angle(t1);
  if (angle >= 0) {                                                   // Conditional statement for checking angle
    Input = angle + 7;                                                // Input for PID is Theta
    MotorPID.Compute();                                               // Calling PID output for given input

    motorGo(0, CW, pwm); //motorGo takes the form (motor, direction, pwm(i.e. speed))
    motorGo(1, CCW, pwm);
    Serial.println("\n Output Forwards (pwm): ");
    Serial.println(pwm);
    /*
      digitalWrite(forward, HIGH);                                      // Motor Control statements
      digitalWrite(backward, LOW);
      analogWrite(motor, (-1)*Output);
    */
  }
  else {
    Input = angle + 7;
    MotorPID.Compute();

    motorGo(0, CCW, pwm);
    motorGo(1, CW, pwm);
    Serial.println("\n Output Backwards (pwm): ");
    Serial.println(Output);
    /*
      digitalWrite(forward, LOW);
      digitalWrite(backward, HIGH);
      analogWrite(motor, Output);
    */
  }
}
  //----------------Monster motor shield loop (as a reference)---------------------------//
  /*
    motorGo(0, CW, pwm); //motorGo takes the form (motor, direction, pwm(i.e. speed))
    motorGo(1, CCW, pwm);
    delay(500);

    motorGo(0, CCW, pwm);
    motorGo(1, CW, pwm);
    delay(500);
    //pwm = pwm + 500;
    if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
      digitalWrite(statpin, HIGH);
    }
  */

