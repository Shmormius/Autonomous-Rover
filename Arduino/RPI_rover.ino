#include <DHT11.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(TXPin, RXPin);
HMC5883L compass;
DHT11 dht11(A0);

String nom = "Arduino";
String msg;
double storedLAT;
double storedLON;
double storedHeading;
double storedSAT = 0;
bool toggle = true;

void setup() {
  Serial.begin(9600);

  ss.begin(GPSBaud);
  
  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset
  compass.setOffset(126, -65, 0);

  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);

  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(13,OUTPUT);
  analogWrite(11, 255);
  analogWrite(10, 255);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    execute(command);
  }
}

void execute(String command){
  if(command == "f" || command == "b" || command == "l" || command == "r" || command == "s"){
    
    drive(command);

  }else if(command.substring(0, 3) == "gps"){

    int spaceIndex = command.indexOf(' ');
    String coordinates = command.substring(spaceIndex + 1);
    int commaIndex = coordinates.indexOf(', ');

    String latitudeStr = coordinates.substring(0, commaIndex);
    String longitudeStr = coordinates.substring(commaIndex + 2);

    double latitude = latitudeStr.toDouble();
    double longitude = longitudeStr.toDouble()*-1.0;
    goToWaypoint(latitude, longitude);

  }else if(command == "temperature"){

    int temperature = dht11.readTemperature();
    if (temperature != DHT11::ERROR_CHECKSUM && temperature != DHT11::ERROR_TIMEOUT){
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
    }else{
      Serial.print("Temperature Reading Error: ");
      Serial.println(DHT11::getErrorString(temperature));
    }

  }else if(command == "humidity"){

    int humidity = dht11.readHumidity();
    if (humidity != DHT11::ERROR_CHECKSUM && humidity != DHT11::ERROR_TIMEOUT){
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    }else{
      Serial.print("Humidity Reading Error: ");
      Serial.println(DHT11::getErrorString(humidity));
    }

  }else if(command == "led"){
    if(toggle){
      digitalWrite(13, HIGH);
      toggle = false;
      Serial.print("Turning LED on");
    }else{
      digitalWrite(13, LOW);
      toggle = true;
      Serial.print("Turning LED off");
    }
  }else{
     Serial.print("Command not recognized: ");
     Serial.println(command);
  }
  command = "";
}

void drive(String command){
  analogWrite(11, 255);
  analogWrite(10, 255);
  if(command=="f"){
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    digitalWrite(4,HIGH);
    Serial.println("Driving forward");
  }else if(command=="b"){
    digitalWrite(7,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    digitalWrite(4,LOW);
    Serial.println("Driving backward");
  }else if(command=="l"){
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,HIGH);
    digitalWrite(4,LOW);
    Serial.println("Turning left");
  }else if(command=="r"){
    digitalWrite(7,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    digitalWrite(4,HIGH);
    Serial.println("Turning right");
  }else if(command=="s"){
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    digitalWrite(4,LOW);
    Serial.println("Stopping");
  }
}

float getHeadingDEG(){
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle
  float declinationAngle = (7.0 + (48.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees and return
  return heading * 180/M_PI;
}

void getGPS(){
  while (ss.available() > 0){
    gps.encode(ss.read());
  }
  storedLAT = gps.location.lat();
  storedLON = gps.location.lng();
  storedSAT = gps.satellites.value();
}

double getDistance(double LAT, double LON, double waypointLAT, double waypointLON){
  return gps.distanceBetween(LAT,LON,waypointLAT,waypointLON);//returns distance to point in meters
}

double angleFromCoordinate(double lat1, double long1, double lat2, double long2) {
    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double brng = atan2(y, x);

    brng = brng * 180/M_PI;
    brng = fmod((brng + 360), 360);
    brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise

    return fmod(brng, 360);
}

void rotateToHeading(double angle){
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);

  while(abs(angle - getHeadingDEG()) > 1){
    int clockwiseDiff = fmod((getHeadingDEG() - angle + 360), 360);
    int counterclockwiseDiff = fmod((angle - getHeadingDEG() + 360), 360);

    if(counterclockwiseDiff < clockwiseDiff){
      int speed = (abs(abs(360-angle)-abs(360-getHeadingDEG())))*255;
      analogWrite(11, speed);
      analogWrite(10, speed);
      digitalWrite(7,HIGH);
      digitalWrite(6,LOW);
      digitalWrite(5,LOW);
      digitalWrite(4,HIGH);
    }else{
      int speed = (abs(angle - getHeadingDEG()))*255;
      analogWrite(11, speed);
      analogWrite(10, speed);
      digitalWrite(7,LOW);
      digitalWrite(6,HIGH);
      digitalWrite(5,HIGH);
      digitalWrite(4,LOW);
    }
  }
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
}

void goToWaypoint(double waypointLAT, double waypointLON){
  while(storedSAT < 3){
    getGPS();
  }
  double startLAT = storedLAT;
  double startLON = storedLON;
  double heading = angleFromCoordinate(storedLAT, storedLON, waypointLAT, waypointLON);
  rotateToHeading(heading);
  while(getDistance(storedLAT, storedLON, waypointLAT, waypointLON) > 2){
    getGPS();
    analogWrite(11, 255);
    analogWrite(10, 255);
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    digitalWrite(4,HIGH);
    getHeadingDEG();
    if((abs(heading - storedHeading)>= 45.0)){
      heading = angleFromCoordinate(storedLAT, storedLON, waypointLAT, waypointLON);
      rotateToHeading(heading);
    }else if(abs(heading - storedHeading)>180){
      if((abs(heading - storedHeading)<= (360.0 - 45.0))){
        heading = angleFromCoordinate(storedLAT, storedLON, waypointLAT, waypointLON);
        rotateToHeading(heading);
      }
    }
  }
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  Serial.println("Waypoint Reached");
}
