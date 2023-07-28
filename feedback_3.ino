#include <Stepper.h>
#include <Wire.h>
float Tapper_Size;
float flowrate;
int IRSensor = 7;  // IR sensor connected to Arduino pin 7
int rawADC = 0;
int const potPin = A1;  // Potentiometer connected with A1
int potVal;
int Pangle;  // Potentiometer angle values

int Z = 0;          // forward pressure (F) + backward pressure(B)  .....(wil give value when needle is attached)
float F;            // forward Pressure ......(Already saved values with needle)
float Saved_angle;  //saved values from excel
int B = 0;          // backward pressure.......B=Z-F
int n = 0;          // constant
int c = 0;          // contant
int y = 0;          // for integer of Z value

int index;  //this will check input value i.e flowrate from serial monitor in Array of r[]
//int Saved_angle ;   // index of Angle
//int F;    // index of Pressure
int r[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41 };                                 // Saved_flowrate  values saved from excel
int p[] = { 0, 17, 24, 31, 38, 42, 49, 52, 52, 59, 63, 66, 70, 73, 77, 80, 84, 87, 87, 95, 100, 105, 105, 109, 116, 119, 126, 126, 137, 144, 151, 161, 172, 186, 200, 214, 235, 253, 274, 313, 355, 443 };  // Saved_Angle values  from excel
int q[] = { 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 18, 20, 22, 23, 26, 27, 30, 33, 35, 39, 42, 45, 48, 52, 55, 59, 62, 67, 71, 76 };                                       // Saved Pressure values  from excel


const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
int stepCount = 0;  // number of steps the motor has taken

#define TCAADDR 0x70
int active_sensors[8];

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

const int ADDRESS = 0x08;             // Address for SLF3x Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 500;  // Scale Factor for flow rate measurement
const char *UNIT_FLOW = " ml/min";    // physical unit of the flow rate measurement
int ret;
int16_t signed_flow_value;
float scaled_flow_value;
byte sensor_flow_crc;

void setup() {
  // initialize the serial port:
  Serial.begin(9600);
  pinMode(IRSensor, INPUT);



Serial.print("Tapper size");
delay(3000);
Serial.println("   6");




Serial.print("Working length");
delay(3000);
Serial.println("   20");


Serial.print("Apical Size");
delay(3000);
Serial.println("   25");

Serial.print("Gauge");
delay(3000);
Serial.println("   24");




  Serial.println("Required flowrate value (integer value between 0-42): ");
  while (Serial.available() == 0) {}
  flowrate = Serial.parseFloat();
  //Serial.println("Required flowrate value is; ") ;
  Serial.println(flowrate);



  for (int i = 0; i < 65; i++) {
    if (r[i] == flowrate) {
      index = i;
      Saved_angle = p[index];
      F = q[index];
      Serial.print("Saved Angle value: ");
      Serial.println(Saved_angle);
      Serial.print("Saved Pressure value: ");
      Serial.print(F);
      break;
    }
  }

  int ret;
  Wire.begin();  // join i2c bus (address optional for master)
  tcaselect(0);  // only relevant if the I2C Multiplexer is used
  do {
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      //  delay(500); // wait long enough for chip reset to complete
    }
  } while (ret != 0);

  delay(50);  // wait long enough for chip reset to complete
  // Begin measurement
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error while sending start measurement command, retrying...");
  }


  delay(1000);
}
void loop() {
  int statusSensor = digitalRead(IRSensor);




  if (statusSensor == 0)  // IR sensor
  {
    if (scaled_flow_value <= flowrate)  // flowrate is Array saved and scaled_flow_value is in working process
    {
      myStepper.step(1);
      stepCount++;

      potVal = analogRead(potPin);
      Pangle = map(potVal, 1023, 0, 0, 3600);
      Serial.print("Angle:");
      Serial.print(Pangle);
      Serial.print(" , ");

      int rawADC = analogRead(A0);
      float voltage = (float)rawADC / 1024.0 * 5.0;
      float Z = (voltage - 0.5) / 4.0 * 150.0 + 0.99;
      y = Z;
      Serial.print("Z:");
      Serial.print(y);
      Serial.print(" , ");

      Wire.requestFrom(ADDRESS, 3);
      if (Wire.available() < 3) {
        Serial.println("Error while reading flow measurement");
      }
      signed_flow_value = Wire.read() << 8;  // read the MSB from the sensor
      signed_flow_value |= Wire.read();      // read the LSB from the sensor
      sensor_flow_crc = Wire.read();
      scaled_flow_value = ((float)signed_flow_value) / SCALE_FACTOR_FLOW;

      Serial.print("Q:");
      //Serial.print(scaled_flow_value);// flowrate( Q)
      int M = (scaled_flow_value * 100) / 100;
      Serial.print(M);
      Serial.print(" , ");


      Serial.print("F:");
      Serial.print(F);
      Serial.print(" , ");

      B = y - F;

      if (B >= 0) {
        Serial.print("B:");  // backwar20d Pressure
        Serial.println(B);
      }

      if (B < 0) {
        Serial.println("B:Na");  // backwar20d Pressure
      }
    }
    delay(50);  // milliseconds delay between reads (for demo purposes)

    if (scaled_flow_value > flowrate)  // flowrate is Array saved and scaled_flow_value is in working process
    {
      myStepper.step(-1);
      stepCount++;

      potVal = analogRead(potPin);
      Pangle = map(potVal, 1023, 0, 0, 3600);
      Serial.print("Angle:");
      Serial.print(Pangle);
      Serial.print(" , ");

      int rawADC = analogRead(A0);
      float voltage = (float)rawADC / 1024.0 * 5.0;
      float Z = (voltage - 0.5) / 4.0 * 150.0 + 0.99;
      y = Z;
      Serial.print("Z:");
      Serial.print(y);
      Serial.print(" , ");

      Wire.requestFrom(ADDRESS, 3);
      if (Wire.available() < 3) {
        Serial.println("Error while reading flow measurement");
      }
      signed_flow_value = Wire.read() << 8;  // read the MSB from the sensor
      signed_flow_value |= Wire.read();      // read the LSB from the sensor
      sensor_flow_crc = Wire.read();
      scaled_flow_value = ((float)signed_flow_value) / SCALE_FACTOR_FLOW;

      Serial.print("Q:");
      //Serial.print(scaled_flow_value);// flowrate( Q)
      int M = (scaled_flow_value * 100) / 100;
      Serial.print(M);
      Serial.print(" , ");


      Serial.print("F:");
      Serial.print(F);
      Serial.print(" , ");

      B = y - F;

      if (B >= 0) {
        Serial.print("B:");  // backwar20d Pressure
        Serial.println(B);
      }

      if (B < 0) {
        Serial.println("B:Na");  // backwar20d Pressure
      }
    }
    delay(50);  // milliseconds delay between reads (for demo purposes)

  }

 
}
