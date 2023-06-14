#include <DHT.h>
#include<SoftwareSerial.h>
#define DHTPIN 2       // Pin connected to the DHT11 sensor
#define DHTTYPE DHT11// DHT sensor type (DHT11 or DHT22)
#define soil_sensor_pin A1
#define gas_sensor_pin A0
#define rain_sensor_pin A2
#define ldr_pin A3
#define buzzer 3
#define led_pin 9
#define ENA 5
#define IN1 6
#define IN2 7
#define flame 4

SoftwareSerial BTSerial(0, 1);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  BTSerial.begin(9600);
  pinMode(buzzer, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(flame, INPUT);
//  Serial.begin(9600);
  dht.begin();
}

void loop() {
  Temp_hum();
  soil();
  gas();
  ldr();
  fLame();
}


void Temp_hum() {
  delay(2000); // Wait for 2 seconds between measurements

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
//  Serial.print("Temperature: ");
//  Serial.print(temperature);
  BTSerial.print(temperature);
  BTSerial.print(" C");
  BTSerial.print(",");
  BTSerial.print(humidity);
  BTSerial.print(",");
//  Serial.print(" °C\t");
//  Serial.print("Humidity: ");
//  Serial.print(humidity);
//  Serial.println(" %");
}

void soil() {
  float moisture_percentage;
  int sensor_analog;
  sensor_analog = analogRead(soil_sensor_pin);
  moisture_percentage = ( 100 - ( (sensor_analog / 1023.00) * 100 ) );
//  Serial.print("Moisture Percentage = ");
//  Serial.print(moisture_percentage);
  BTSerial.print(moisture_percentage);
  BTSerial.print(" %");
  BTSerial.print(",");
//  Serial.print("%\n\n");
//  delay(1000);

  int rainValue = rain();

  if (moisture_percentage <= 40 and rainValue <= 99) {
    motorControl(255, HIGH, LOW);
  }

  else {
    motorControl(0, LOW, LOW);
  }
}

void motorControl(int speedA, int in1, int in2) {
  analogWrite(ENA, speedA);  // Set the motor speed
  digitalWrite(IN1, in1);  // Set the motor direction
  digitalWrite(IN2, in2);  // Set the motor direction
}

void gas() {
  unsigned int sensorValue = analogRead(gas_sensor_pin);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  int flm = fLame();
  if (outputValue > 65) {
    digitalWrite(buzzer, HIGH);
  }// generate PWM signal

  else if (flm == 0) {
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
  }

  else
    digitalWrite(buzzer, LOW);

//  Serial.print("Gas amount: ");
//  Serial.println(outputValue);
  BTSerial.print(outputValue);
  BTSerial.print(",");
}

int rain() {
  int sensorValue = analogRead(rain_sensor_pin);  // Read the analog value from sensor

  int outputValue = map(sensorValue, 0, 1023, 255, 0); // map the 10-bit data to 8-bit data


  if (outputValue >= 100) {
    digitalWrite(led_pin, HIGH);
  }

  else {
    digitalWrite(led_pin, LOW);
  }
//  Serial.print("Rain: ");
//  Serial.println(outputValue);
  BTSerial.print(outputValue);
  BTSerial.print(",");
  return outputValue;
}

void ldr() {
  int ldrVal = analogRead(ldr_pin);

//  Serial.print("Light Intensity: ");
//  Serial.println(ldrVal);
  BTSerial.print(ldrVal);
  BTSerial.print(",");
  BTSerial.print(";");
  delay(500);
}

int fLame() {
  int rd = digitalRead(flame);
  return rd;
}
