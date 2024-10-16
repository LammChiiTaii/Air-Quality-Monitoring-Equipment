
/*************************************************************
  Download latest ERa library here:
    https://github.com/eoh-jsc/era-lib/releases/latest
    https://www.arduino.cc/reference/en/libraries/era
    https://registry.platformio.org/libraries/eoh-ltd/ERa/installation

    ERa website:                https://e-ra.io
    ERa blog:                   https://iotasia.org
    ERa forum:                  https://forum.eoh.io
    Follow us:                  https://www.fb.com/EoHPlatform
 *************************************************************/

// Enable debug console
// Set CORE_DEBUG_LEVEL = 3 first
// #define ERA_DEBUG

#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "84d30e09-0085-41d3-8840-f5900b24ccbb"

#include <Arduino.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>

#include <Adafruit_SHT31.h>
#include <BH1750FVI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define         MQ135PIN                       A5     //define which analog input channel you are going to use
#define         RL_VALUE_MQ135                 2     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ135      3.59   //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the chart in datasheet

#define         MQ2PIN                         A6     //define which analog input channel you are going to use
#define         RL_VALUE_MQ2                   10     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ2        9.577  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the chart in datasheet

#define         CALIBARAION_SAMPLE_TIMES       50     //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL    500    //define the time interal(in milisecond) between each samples in the
                                                      //cablibration phase
#define         READ_SAMPLE_INTERVAL           50     //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES              5      //define the time interal(in milisecond) between each samples in 
                                                      //normal operation
#define         GAS_CARBON_DIOXIDE             9
#define         GAS_CARBON_MONOXIDE            3
#define         GAS_SMOKE                      5
#define         GAS_LPG                        1

float Ro2 = 123.64, Ro135 = 1.99;

const char ssid[] = "American Study HD";
const char pass[] = "66668888";

Adafruit_MPU6050 mpu;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
uint8_t ADDRESSPIN = 13;
BH1750FVI::eDeviceAddress_t DEVICEADDRESS = BH1750FVI::k_DevAddress_L;
BH1750FVI::eDeviceMode_t DEVICEMODE = BH1750FVI::k_DevModeContHighRes;
BH1750FVI LightSensor(ADDRESSPIN, DEVICEADDRESS, DEVICEMODE);

double a=0, b=0, x=0, y=0;
uint64_t tim = 0;
float volMax = 2.0, volMin = 0.4, volConvCons = 0.004882814, volSen = 0;
float windSpeedMin = 0;
float windSpeedMax = 30;
float WindSpeed = 0, prevWindSpeed = 0;
int Direction = 0, direc = 0;
float temp = 0, humi = 0, accX = 0, accY = 0, accZ = 0, preAccX = 0, preAccY = 0, preAccZ = 0;
uint16_t lux = 0;
float voMeasured = 0, calcVoltage = 0, dustDensity = 0;
int co2 = 0, co = 0, smoke = 0, lpg = 0;

ERaTimer timer;

/* This function print uptime every second */
void timerEvent() {
    ERa.virtualWrite(V0, (ERaMillis() / 1000L / 3600L));
    ERa.virtualWrite(V1, ((ERaMillis() / 1000L / 60L) - int(ERaMillis() / 1000L / 3600L) * 60L));
    ERa.virtualWrite(V2, (ERaMillis() / 1000L) % 60L);
    ERa.virtualWrite(V3 , temp     );
    ERa.virtualWrite(V4 , humi     );
    ERa.virtualWrite(V5 , lux      );
    ERa.virtualWrite(V6 , accX     );
    ERa.virtualWrite(V7 , accY     );
    ERa.virtualWrite(V8 , accZ     );
    ERa.virtualWrite(V9 , direc    );
    ERa.virtualWrite(V10, WindSpeed);
    if (dustDensity > 0) ERa.virtualWrite(V11, dustDensity);
    else ERa.virtualWrite(V11, 0);
  if (millis() - tim > 20000) {
    ERa.virtualWrite(V12, co2      );
    ERa.virtualWrite(V13, co       );
    ERa.virtualWrite(V14, smoke    );
    ERa.virtualWrite(V15, lpg      );
  }
    ERA_LOG("Timer", "Uptime: %d", ERaMillis() / 1000L);
}

void setup() {
    /* Setup debug console */
    Serial.begin(9600);
    Wire.begin();

    ERa.begin(ssid, pass);
    timer.setInterval(1000L, timerEvent);

  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  if (!sht31.begin(0x44)) {  // Địa chỉ I2C của cảm biến (0x44 hoặc 0x45)
    Serial.println("Không tìm thấy SHT31");
    while (1) delay(1);
  }

  LightSensor.begin();

  pinMode(23, OUTPUT);
  delay(100);

}

void loop() {
  temp = sht31.readTemperature();
  humi = sht31.readHumidity();
  lux = LightSensor.GetLightIntensity();
  DoGiaToc();
  DoHuongGio();
  DoTocDoGio();
  DoBui();
  co2 = MQ135GetGasPercentage(MQ135Read(MQ135PIN)/Ro135, GAS_CARBON_DIOXIDE) ;
  co  = MQ135GetGasPercentage(MQ135Read(MQ135PIN)/Ro135, GAS_CARBON_MONOXIDE);
  smoke = MQ2GetGasPercentage(MQ2Read(MQ2PIN)/Ro2, GAS_SMOKE);
  lpg   = MQ2GetGasPercentage(MQ2Read(MQ2PIN)/Ro2, GAS_LPG)  ;
  
  Serial.println(Direction);

    ERa.run();
    timer.run();
}

void DoGiaToc() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
}

void DoBui() {
  digitalWrite(23, LOW);
  delayMicroseconds(280);
  voMeasured = analogRead(A0);
  delayMicroseconds(40);
  digitalWrite(23, HIGH);
  delayMicroseconds(9680);

  calcVoltage = voMeasured * (3.3 / 4095);
  dustDensity = 0.17 * calcVoltage - 0.1;
}

void DoHuongGio() {
  int sensorValue = analogRead(A7);
  float voltage = sensorValue * 3.3 / 4095.0;
  Direction = map(sensorValue, 0, 4095, 0, 360);
  if (Direction < 43) direc = 0;
  else if (Direction >= 43 && Direction < 100) direc = 45;
  else if (Direction >= 100 && Direction < 160) direc = 90;
  else if (Direction >= 160 && Direction < 220) direc = 135;
  else if (Direction >= 220 && Direction < 260) direc = 180;
  else if (Direction >= 260 && Direction < 300) direc = 225;
  else if (Direction >= 300 && Direction < 330) direc = 270;
  else if (Direction >= 330 && Direction <= 360) direc = 315;
  delay(1);
}

void DoTocDoGio() {
  int sensorValue = analogRead(A4);
  float voltage = sensorValue * 3.3 / 4095.0;
  volSen = sensorValue * volConvCons / 4.0;

  if (volSen <= volMin) WindSpeed = 0;
  else WindSpeed = ((volSen - volMin) * windSpeedMax / (volMax - volMin)) * 2.232694;
  x = WindSpeed;
  if (x >= y) y = x; else y = y;
  a = volSen;
  if (a >= b) b = a; else b = b;
}

float MQ135Calibration(int mq_pin) {
  int i;
  float RS_AIR_val=0,r0;
  for (i=0; i<CALIBARAION_SAMPLE_TIMES; i++) {                      //take multiple samples
    RS_AIR_val += MQ135ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;               //calculate the average value
  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ135;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                  //according to the chart in the datasheet
  return r0; 
}

float MQ135ResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE_MQ135*(4095-raw_adc)/raw_adc));
}

float MQ135Read(int mq_pin) {
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQ135ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

int MQ135GetGasPercentage(float rs_ro_ratio, int gas_id) {
  if (gas_id == GAS_CARBON_DIOXIDE)         return (pow(10,((-2.890*(log10(rs_ro_ratio))) + 2.055)));
  else if (gas_id == GAS_CARBON_MONOXIDE)   return (pow(10,(1.457*pow((log10(rs_ro_ratio)), 2) - 4.725*(log10(rs_ro_ratio)) + 2.855)));
}

float MQ2Calibration(int mq_pin) {
  int i;
  float RS_AIR_val=0,r0;
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {                      //take multiple samples
    RS_AIR_val += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;               //calculate the average value
  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ2;                        //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                  //according to the chart in the datasheet 
  return r0; 
}

float MQ2ResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE_MQ2*(4095-raw_adc)/raw_adc));
}

float MQ2Read(int mq_pin) {
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

int MQ2GetGasPercentage(float rs_ro_ratio, int gas_id) {
  if (gas_id == GAS_SMOKE)    return (pow(10,(-0.976*pow((log10(rs_ro_ratio)), 2) - 2.018*(log10(rs_ro_ratio)) + 3.617)));
  else if (gas_id == GAS_LPG) return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)));
}