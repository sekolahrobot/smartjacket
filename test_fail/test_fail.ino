#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

#define LED      25
#define ON  HIGH
#define OFF LOW
#define SDA_PIN    16        // SDA PIN
#define SCL_PIN    17        // SCL PIN
#define I2C_CLK_FREQ 400000 // Fast I2C 400kHz Clk
float compAngleX=0, compAngleY=0, compAngleZ=0; // Calculate the angle using a complementary filter
boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred

int angleChange=0;
void setup() {
  Serial.begin(115200); 
  pinMode(LED,OUTPUT);

  Wire.setClock(I2C_CLK_FREQ);
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  delay(10);
 
 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);

}

void loop() {
  // put your main code here, to run repeatedly:
  mpu_read();
  ax = (AcX-1025)/16384.00;
  ay = (AcY-1482)/16384.00;
  az = (AcZ+1593)/16384.00;

   gx = (GyX+246)/131.07;
   gy = (GyY+731)/131.07;
   gz = (GyZ-36)/131.07;

 // calculating Amplitute vactor for 3 axis
 float Raw_AM = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
 int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied 
                        // it by for using if else conditions 

 Serial.println(AM);

 if (AM<=2){ //if AM breaks lower threshold (0.4g)
   trigger1=true;
   Serial.println("Fall Detected");

   
   }
//It appears that delay is needed in order not to clog the port
 delay(100);
}

void mpu_read(){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 }
