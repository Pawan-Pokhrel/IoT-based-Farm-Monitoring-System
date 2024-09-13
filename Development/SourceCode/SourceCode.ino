#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHT11_PIN  4 
DHT dht11(DHT11_PIN, DHT11);
#define TRIG_PIN   5 // ESP32 pin GPIO26 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN   18 // ESP32 pin GPIO25 connected to Ultrasonic Sensor's ECHO pin
#define BUZZER_PIN 19 // ESP32 pin GPIO17 connected to Piezo Buzzer's pin
#define DISTANCE_THRESHOLD 7 // centimeters
#define MOISTURE_THRESHOLD 30
const int relay = 26;


// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
int _moisture,sensor_analog;
const int sensor_pin = 34;    /* Soil moisture sensor O/P pin */
float duration_us, distance_cm;
// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

void setup(){
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
 
  dht11.begin(); // initialize the DHT11 sensor
  Serial.begin (9600);         // initialize serial port
  pinMode(TRIG_PIN, OUTPUT);   // set ESP32 pin to output mode
  pinMode(ECHO_PIN, INPUT);    // set ESP32 pin to input mode
  pinMode(BUZZER_PIN, OUTPUT); // set ESP32 pin to output mode
  pinMode(relay, OUTPUT);
}

void loop(){

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);
  // calculate the distance
  distance_cm = 0.017 * duration_us;

  if (distance_cm > DISTANCE_THRESHOLD)
    digitalWrite(BUZZER_PIN, HIGH); // turn on Piezo Buzzer
  else
    digitalWrite(BUZZER_PIN, LOW);  // turn off Piezo Buzzer

  // read humidity
  float humi  = dht11.readHumidity();
  // read temperature in Celsius
  float tempC = dht11.readTemperature();
  // read temperature in Fahrenheit
  float tempF = dht11.readTemperature(true);
  // set cursor to first column, first row

  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT11 sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");
  }

  sensor_analog = analogRead(sensor_pin);
  _moisture = ( 100 - ( ( sensor_analog/4095.00) * 100 ) );
  Serial.print("Moisture = ");
  Serial.print(_moisture);  /* Print Temperature on the serial window */
  Serial.println("%");
  delay(1000);              /* Wait for 1000mS */

  if (_moisture < MOISTURE_THRESHOLD) {
        // If moisture is low, turn on relay
        digitalWrite(relay, HIGH);
    } else {
        // If moisture is not low, turn off relay
        digitalWrite(relay, LOW);
    }

  // wait a 2 seconds between readings
  
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Moisture%=");
  
  lcd.print(_moisture);
 
 
  // clears the display to print new message
  
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  lcd.print("T=");
   lcd.print(tempC);


   lcd.print("H%=");
   lcd.print(humi);
   delay(1000);
   lcd.clear();

   Serial.print("DIstance=");
   Serial.print(distance_cm);


  

   

}