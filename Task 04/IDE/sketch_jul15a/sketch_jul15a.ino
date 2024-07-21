#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <Servo.h>  // Include the Servo library

#define BUZZERPIN 12       // Define buzzer pin
#define DHTPIN 8           // DHT11 data pin is connected to Arduino pin 8
#define DHTTYPE DHT11      // DHT11 sensor is used

DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT library

// Set the LCD address to 0x27 for a 16 characters and 2 lines display
LiquidCrystal_I2C lcd(0x27, 16, 2);
  
int Gas = A0;
int ir = A1;
int pir = A2;

char temperature[] = "Temp = 00.0 C  ";
char humidity[]    = "RH   = 00.0 %  ";

unsigned long previousMillis = 0;      // Stores the last time temperature was updated
const long interval = 2000;            // Interval at which to read temperature (milliseconds)
bool motorState = false;
bool buzzerState = false;
bool gasAlertState = false;
bool motionAlertState = false;
bool pirAlertState = false;
unsigned long buzzerMillis = 0;        // Stores the last time the buzzer was updated
const int buzzerInterval = 500;        // Interval for buzzer tone change (milliseconds)

// Define L298N Motor Driver pins
const int motorPin1 = 2;    // Connect to L298N IN1
const int motorPin2 = 3;    // Connect to L298N IN2
const int enablePin = 6;    // Connect to L298N ENA for motor A control

// Define Servo motor pin
const int servoPin = 3;
Servo myservo;  // Create a Servo object

void setup() {
  dht.begin();
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  pinMode(BUZZERPIN, OUTPUT); // Set buzzer pin as output
  
  // Initialize the LCD with 16 columns and 2 rows
  lcd.begin(16, 2);
  // Turn on the backlight and print a message
  lcd.backlight();
  // Initial LCD display
  lcd.setCursor(0, 0);
  lcd.print(temperature);
  lcd.setCursor(0, 1);
  lcd.print(humidity);
  
  // Attach the servo to the designated pin
  myservo.attach(servoPin);

  pinMode(pir, INPUT); // Set PIR sensor pin as input
  pinMode(ir, INPUT);  // Set IR sensor pin as input
  pinMode(Gas, INPUT); // Set Gas sensor pin as input
}

void loop() {
  if (digitalRead(pir) == HIGH) {
    lcd.clear(); // Clear the entire LCD
    lcd.setCursor(0, 0);
    lcd.print("Person Detected");
    delay(50);
    lcd.clear();
    lcd.print("Home Monitoring");
    lcd.clear();
    
    // Activate motion detection alert for PIR sensor
    pirAlertState = true;
  } else {
    pirAlertState = false;
  }

  // Check IR sensor
  if (digitalRead(ir) == LOW) {
    lcd.clear(); // Clear the entire LCD
    lcd.setCursor(0, 0);
    lcd.print("Motion Detected");
    delay(50);
    lcd.clear();
    lcd.print("Home Monitoring");
    lcd.clear();
    
    // Activate motion detection alert for IR sensor
    motionAlertState = true;

    // Open the door (move the servo to the open position)
    lcd.print("Door Opened");
    myservo.write(35); // Adjust the angle as needed for your door
    delay(5000);
    lcd.clear();
  } else {
    motionAlertState = false;
    noTone(BUZZERPIN);  // Ensure the buzzer is off when no motion is detected

    // Close the door (move the servo to the closed position)
    myservo.write(130); // Adjust the angle as needed for your door
  }
  
  if (digitalRead(Gas) == HIGH) {
    lcd.clear(); // Clear the entire LCD
    lcd.setCursor(0, 0);
    lcd.print("Gas Detected");
    delay(50);
    lcd.clear();
    lcd.print("Home Monitoring");
    lcd.clear();
    
    // Activate gas leakage alert
    gasAlertState = true;
  } else {
    gasAlertState = false;
    noTone(BUZZERPIN);  // Ensure the buzzer is off when gas is not detected
  }

  unsigned long currentMillis = millis();
  
  // Check if it's time to read the temperature and humidity
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read humidity
    float RH = dht.readHumidity();      // Read humidity as float
    // Read temperature in degree Celsius
    float Temp = dht.readTemperature(); // Read temperature as float

    // Check if any reads failed and exit early (to try again)
    if (isnan(RH) || isnan(Temp)) {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("Error");
      return;
    }

    // Update temperature and humidity display
    dtostrf(Temp, 5, 1, temperature + 7); // Update temperature value
    dtostrf(RH, 5, 1, humidity + 7);      // Update humidity value

    // Display updated values on LCD
    lcd.setCursor(0, 0);
    lcd.print(temperature);
    lcd.setCursor(0, 1);
    lcd.print(humidity);

    // Motor control logic using L298N module
    if (Temp > 33) {
      if (!motorState) {
        motorState = true;
        analogWrite(enablePin, 255);  // Full speed
        digitalWrite(motorPin1, HIGH);
        
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("Fan On");
        delay(2000);  // Display "Fan On" for 2 seconds
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(temperature);
        lcd.setCursor(0, 1);
        lcd.print(humidity);
      }
    } else {
      if (motorState) {
        motorState = false;
        analogWrite(enablePin, 0);  // Turn off motor
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
        
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("Fan Off");
        delay(2000);  // Display "Fan Off" for 2 seconds
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(temperature);
        lcd.setCursor(0, 1);
        lcd.print(humidity);
      }
    }

    // Buzzer control logic
    if (Temp > 40) {
      lcd.clear();
      lcd.print("High Temp");
      delay(250);
      lcd.clear();
      buzzerState = true;
    } else {
      buzzerState = false;
      noTone(BUZZERPIN);      // Ensure the buzzer is off when temp is below 40
    }
  }

  // Handle the buzzer tone change for temperature alert
  if (buzzerState && !gasAlertState && !motionAlertState && !pirAlertState) {
    if (currentMillis - buzzerMillis >= buzzerInterval) {
      buzzerMillis = currentMillis;
      tone(BUZZERPIN, 1200);  // Pleasant tone for general alerts (1.2 kHz)
      delay(100);  // Delay for tone duration
      noTone(BUZZERPIN);  // Turn off buzzer after tone duration
    }
  }
  
  // Handle the gas alert buzzer tone
  if (gasAlertState) {
    if (currentMillis - buzzerMillis >= buzzerInterval) {
      buzzerMillis = currentMillis;
      tone(BUZZERPIN, 3000);  // More urgent tone for gas leakage alert (3 kHz)
      delay(100);  // Delay for tone duration
      noTone(BUZZERPIN);  // Turn off buzzer after tone duration
    }
  }
  
  // Handle the motion alert buzzer tone
  if (motionAlertState) {
    if (currentMillis - buzzerMillis >= buzzerInterval) {
      buzzerMillis = currentMillis;
      tone(BUZZERPIN, 2000);  // Distinctive tone for motion detection alert (2 kHz)
      delay(100);  // Delay for tone duration
      noTone(BUZZERPIN);  // Turn off buzzer after tone duration
    }
  }

  // Handle the PIR alert buzzer tone
  if (pirAlertState) {
    if (currentMillis - buzzerMillis >= buzzerInterval) {
      buzzerMillis = currentMillis;
      tone(BUZZERPIN, 2000);  // Distinctive tone for PIR motion detection alert (2 kHz)
      delay(100);  // Delay for tone duration
      noTone(BUZZERPIN);  // Turn off buzzer after tone duration
    }
  }
}