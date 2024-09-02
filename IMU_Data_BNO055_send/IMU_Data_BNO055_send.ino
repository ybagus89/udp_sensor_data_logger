#include <M5Stack.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

// Set up WiFi AP mode
const char* apSsid = "M5Stack_AP";
const char* apPassword = "12345678";

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double xPos = 0, yPos = 0, headingVel = 0, xvel=0, yvel=0, xdist=0, ydist=0;
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329252; //trig functions require radians, BNO055 outputs degrees

WiFiUDP udp;

void setup() {
  M5.begin();
  Serial.begin(115200);

  // Initialize WiFi AP mode
  WiFi.softAP(apSsid, apPassword);
  Serial.println("WiFi AP mode enabled");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // Initialize UDP server
  udp.begin(55555);
  Serial.println("UDP server started");
}

void loop() {
  // Read IMU data
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  // yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
    
  // xvel = xvel + linearAccelData.acceleration.x + ACCEL_VEL_TRANSITION;
  // yvel = yvel + linearAccelData.acceleration.y + ACCEL_VEL_TRANSITION;

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

   // Calculate the distance traveled
  // xdist += abs(xvel) * ACCEL_VEL_TRANSITION;  // Accumulate distance along X
  // ydist += abs(yvel) * ACCEL_VEL_TRANSITION;  // Accumulate distance along Y

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  // Create a UDP packet with the IMU data
  char buffer[1024];
  int bufferSize = sizeof(buffer) / sizeof(buffer[0]);
  int bytesWritten = snprintf(buffer, bufferSize, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f",
          orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z,
          angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z,
          linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z,
          magnetometerData.magnetic.x, magnetometerData.magnetic.y, magnetometerData.magnetic.z,
          accelerometerData.acceleration.x, accelerometerData.acceleration.y, accelerometerData.acceleration.z,
          gravityData.acceleration.x, gravityData.acceleration.y, gravityData.acceleration.z,
          xPos, yPos, headingVel);

  if (bytesWritten >= bufferSize) {
    Serial.println("Buffer overflow! Increase buffer size.");
  } else { 
  // Send the UDP packet to the PC
  udp.beginPacket("192.168.4.2", 55555); // Replace with the PC's IP address
  //udp.write(buffer);
  udp.write((uint8_t*)buffer, bytesWritten);
  udp.endPacket();
    // Send the IMU data to the PC via serial
  Serial.print("IMU Data: ");
  Serial.println(buffer);  
  }

  // Display yaw (orientation) data on the LCD
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Yaw: ");
  M5.Lcd.print(orientationData.orientation.x);
  M5.Lcd.print("Deg");

  M5.Lcd.setCursor(175, 0);
  M5.Lcd.print("H: ");
  M5.Lcd.print(headingVel);
  M5.Lcd.print("m/s");

  M5.Lcd.setCursor(0, 20);
  M5.Lcd.print("XPos: ");
  M5.Lcd.print(xPos);
  M5.Lcd.setCursor(150, 20);
  M5.Lcd.print("YPos: ");
  M5.Lcd.print(yPos);

  // M5.Lcd.setCursor(0, 40);
  // M5.Lcd.print("Xdist: ");
  // M5.Lcd.print(xdist);
  // M5.Lcd.setCursor(150, 40);
  // M5.Lcd.print("Ydist: ");
  // M5.Lcd.print(ydist);

  //   // Draw pitch and roll bar lines
  // int barWidth = 10;
  // int pitchBarLength = map(orientationData.orientation.y, -90, 90, -50, 50);  // Map pitch from -90 to 90 degrees
  // int rollBarLength = map(orientationData.orientation.z, -180, 180, -50, 50); // Map roll from -180 to 180 degrees

  // M5.Lcd.fillRect(120, 100 - pitchBarLength / 2, barWidth, pitchBarLength, RED);  // Pitch bar
  // M5.Lcd.fillRect(150, 100 - rollBarLength / 2, barWidth, rollBarLength, BLUE);   // Roll bar

  // Draw a simple compass rose
  int centerX = M5.Lcd.width() / 2;
  int centerY = M5.Lcd.height() / 2; 
  int radius = 50;
  M5.Lcd.drawCircle(centerX, centerY, radius, WHITE);
  M5.Lcd.drawLine(centerX, centerY, centerX + radius * cos(DEG_2_RAD * orientationData.orientation.x), centerY + radius * sin(DEG_2_RAD * orientationData.orientation.x), WHITE);

  M5.update();
  delay(10);
}