# GloRobo-Projects



## 🅿️ Smart Parking Management System (Arduino Mega 2560)
This project implements a fully automated, real-time smart parking solution designed to manage vehicle entry, track occupancy, and calculate parking fees upon exit. Built around the high-capacity Arduino Mega 2560, the system provides a robust solution for simulating modern, efficient parking facilities.

### Key Features:

* Real-Time Occupancy Tracking: Utilizes multiple ultrasonic sensors to monitor the status of 5 dedicated parking slots and update the available count instantly.

* Automated Entry Gate: The entry gate servo automatically opens when an incoming vehicle is detected and closes after it passes, contingent on slot availability.

* Dynamic Billing System: Calculates parking fees based on the duration of stay, using a tiered, hour-based rate structure.

* Keypad-Controlled Exit: Drivers use a 4x4 keypad at the exit gate to enter their assigned slot ID and confirm payment, triggering the exit gate servo.

* I2C LCD Interface: A 20x4 I2C Liquid Crystal Display is used to provide real-time updates on available slots, entry instructions, and calculated parking fees.

### Technologies Used:
* Microcontroller: Arduino Mega 2560

* Sensing: Ultrasonic Sensors (HC-SR04)

* Actuation: Servo Motors (for gate control)

* Interface: 4x4 Keypad and I2C 20x4 LCD

* Simulation Environment: Tested using Wokwi for hardware-software integration verification.



### Code Architecture Overview 
The project code is built on a modular, state-driven architecture, designed to manage concurrent sensor inputs and sequential logical operations (Entry, Parking, Billing, Exit) efficiently.

#### 1. Libraries and Setup
---

* The code begins by integrating four essential libraries to handle external hardware components:

* Wire.h and LiquidCrystal_I2C.h: Manages all output messages via the 4-pin I2C LCD, conserving valuable I/O pins.

* Servo.h: Controls the two servo motors responsible for opening and closing the entry and exit gates.

* Keypad.h: Handles the matrix input for the 4x4 keypad used by drivers during the billing process.

* math.h: Used specifically for the ceil() function to correctly round up partial hours in the fee calculation.

The setup() function initializes all components: setting up the LCD, attaching the servos, defining all 38+ I/O pins (Trig, Echo, Red/Green LEDs) as inputs or outputs, and ensuring both gates are closed at startup.


#### 2. Data Management and State Tracking
---
The system's real-time intelligence is managed by three key global arrays:

| Array/Variable | Type | Purpose | 
| ---- | -- | -- |
| entryTime[5]	| long |	Stores the millis() timestamp (simulated RTC) when a car is assigned to a specific slot. This is the start time for billing. A value of 0 indicates the slot is logically vacant. |
| slotOccupied[5] |	bool |	A boolean flag indicating if a car has physically triggered the spot sensor and is present. |
| availableSlots |	int	| The primary counter, initialized to 5, which guides the Entry Gate logic.

#### 3. The Main Loop Logic (The System Flow)

The loop() function runs continuously, sequentially handling four distinct phases:

##### A. Parking Slot Status Update (The Core State)

* Iterates through all 5 parking slots.

* Uses readDistance() to check if a car is physically present (<20 cm).

* Updates the Red/Green LEDs to reflect the physical presence. Crucially, the Green LED only turns back ON when the car has successfully exited AND its entryTime is cleared, preventing flicker(moving back and forth).

##### B. Entry Gate Logic

* Uses readDistance() on the Entry Gate Sensor.

If a car is detected:

* Checks if availableSlots > 0. If full, it displays a "PARKING FULL" message and keeps the gate closed(0 degrees).

* If slots are available, it opens the gate (90 degrees), assigns the car to the first available slot (by recording the current millis() timestamp into entryTime[i]), decreases availableSlots, and then closes the gate.


##### C. Exit Gate and Billing Logic (The Financial Transaction)

* Uses readDistance() on the Exit Gate Sensor.

If a car is detected:

* The system prompts the user to enter their Slot ID using the keypad.getKey().

* The calculateFee() function determines the fee based on the time elapsed (current millis() - entryTime[i]).

* It waits for the user to press the # key to simulate payment.

* Upon payment confirmation, the Exit Gate opens (90 degrees), the slot's entryTime is reset to 0, and availableSlots is incremented.

##### D. Display Update

The function ensures the number of available slots is continuously displayed on the LCD




This is the code used for the simulation(using Liquid Crystal I2C)

```C

// 1. LIBRARIES 

#include <Wire.h>            //for I2C communication (SDA/SCL)
#include <LiquidCrystal_I2C.h> //for the 4-pin I2C LCD
#include <Servo.h>           // for servo motor control
#include <Keypad.h>          // Library for handling keypad input
#include <math.h>            // for the ceil() function in fee calculation


// 2. CONFIGURATION & CONSTANTS

const int TOTAL_SLOTS = 5; // Define the total capacity of the parking lot, can be changed

// Define time and fee structure
const int FIRST_HOUR_RATE_CENTS = 100; // $1.00 for the first hour (in cents)
const int HOURLY_RATE_CENTS = 50;      // $0.50 for every subsequent hour
const long ONE_HOUR_MS = 3600000;      // 1 hour in milliseconds (3600 * 1000)

// Ultrasonic Sensor Threshold (Distance in cm)
const int DISTANCE_THRESHOLD = 20; 


// 3. PIN MAPPING (Arduino Mega 2560)

// Keypad setup (4x4)
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = { //defining keypad characters
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
//assigning pins on the mcu to rows and columns of the keypad
byte rowPins[ROWS] = {22, 23, 24, 25}; 
byte colPins[COLS] = {26, 27, 28, 29};
//bringing all together
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// LCD setup: I2C LCD (4 pins VCC, GND, SDA, SCL)
// 0x27 represents the I2C address of the I2C adapter connected to the LCD display.
LiquidCrystal_I2C lcd(0x27, 20, 4); 

// Gate Pins i.e. of the servo motors and ultrasonic sensors at entry and exit
const int ENTRY_TRIG_PIN = 30; 
const int ENTRY_ECHO_PIN = 31; 
const int ENTRY_SERVO_PIN = 8; 
const int EXIT_TRIG_PIN = 32;  
const int EXIT_ECHO_PIN = 33;  
const int EXIT_SERVO_PIN = 9;  

// Parking Slot Pins (Mega Digital Pins 34 to 53)- for leds nad ultrasonic sensors
int trigPins[TOTAL_SLOTS]  = {34, 38, 42, 46, 50}; 
int echoPins[TOTAL_SLOTS]  = {35, 39, 43, 47, 51}; 
int greenLEDs[TOTAL_SLOTS] = {36, 40, 44, 48, 52}; 
int redLEDs[TOTAL_SLOTS]   = {37, 41, 45, 49, 53}; 

// 4. GLOBAL VARIABLES

Servo entryServo;
Servo exitServo;
int availableSlots = TOTAL_SLOTS;
long entryTime[TOTAL_SLOTS] = {0, 0, 0, 0, 0}; 
bool slotOccupied[TOTAL_SLOTS] = {false};  //initialization


// 5. FUNCTIONS

// Reads distance from an Ultrasonic sensor 
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Returns distance in cm
}

// Controls the servo motor angle (0=Closed, 90=Open)
void setGate(Servo& servo, int angle) {
  servo.write(angle);
}

long calculateFee(long duration_ms) {
  if (duration_ms <= 0) return 0; 

  float duration_hours = (float)duration_ms / ONE_HOUR_MS;
  long totalFee = 0;
  
  if (duration_hours <= 1.0) {
    totalFee = FIRST_HOUR_RATE_CENTS;
  } else {
    totalFee = FIRST_HOUR_RATE_CENTS;
    
    float additionalTime = duration_hours - 1.0;
    int additionalHours = (int)ceil(additionalTime); 
    
    totalFee += additionalHours * HOURLY_RATE_CENTS;
  }
  return totalFee;
}


// 6. SETUP 


void setup() {
  Serial.begin(9600); 
  
  // LCD Initialization
  lcd.init();        // Initialize the I2C LCD
  lcd.backlight();   // Turn on the backlight 
  
  lcd.print("GLORY'S Smart Parking System");
  lcd.setCursor(0, 1);
  lcd.print("Total Slots: 5");
  
  // Servo Initialization
  entryServo.attach(ENTRY_SERVO_PIN);
  exitServo.attach(EXIT_SERVO_PIN);
  // Close both gates
  setGate(entryServo, 0); 
  setGate(exitServo, 0);

  // Pin Mode Setup for all slot sensors and LEDs
  for (int i = 0; i < TOTAL_SLOTS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    pinMode(greenLEDs[i], OUTPUT);
    pinMode(redLEDs[i], OUTPUT);
    digitalWrite(greenLEDs[i], HIGH); 
    digitalWrite(redLEDs[i], LOW);
  }
  
  // Setup for gate sensors
  pinMode(ENTRY_TRIG_PIN, OUTPUT);
  pinMode(ENTRY_ECHO_PIN, INPUT);
  pinMode(EXIT_TRIG_PIN, OUTPUT);
  pinMode(EXIT_ECHO_PIN, INPUT);
}


// 7. MAIN LOOP


void loop() {
  // A. UPDATE PARKING SLOT STATUS (LEDs)
  for (int i = 0; i < TOTAL_SLOTS; i++) {
    long slotDist = readDistance(trigPins[i], echoPins[i]);
    if (slotDist < DISTANCE_THRESHOLD) {
      slotOccupied[i] = true;
      digitalWrite(redLEDs[i], HIGH);
      digitalWrite(greenLEDs[i], LOW);
    } else {
      if (entryTime[i] == 0) {
        slotOccupied[i] = false;
        digitalWrite(redLEDs[i], LOW);
        digitalWrite(greenLEDs[i], HIGH);
      }
    }
  }

  // B. HANDLE ENTRY GATE LOGIC
  long entryDist = readDistance(ENTRY_TRIG_PIN, ENTRY_ECHO_PIN);
  if (entryDist < DISTANCE_THRESHOLD) {
    if (availableSlots > 0) {
      // Logic: Allow Entry
      lcd.setCursor(0, 2);
      lcd.print("Gate Opening...      ");
      setGate(entryServo, 90); 
      
      for (int i = 0; i < TOTAL_SLOTS; i++) {
        if (entryTime[i] == 0) {
          entryTime[i] = millis(); 
          availableSlots--;
          lcd.setCursor(0, 3);
          lcd.print("Park at Slot: ");
          lcd.print(i + 1);
          delay(4000); 
          setGate(entryServo, 0); // Close gate
          break;
        }
      }
    } else {
      // Logic: Parking is Full
      lcd.setCursor(0, 2);
      lcd.print("PARKING FULL!        ");
      lcd.setCursor(0, 3);
      lcd.print("Gate Closed.         ");
      setGate(entryServo, 0); 
    }
  }
  
  // C. HANDLE EXIT GATE & BILLING LOGIC
  long exitDist = readDistance(EXIT_TRIG_PIN, EXIT_ECHO_PIN);
  char key = keypad.getKey(); 
  
  if (exitDist < DISTANCE_THRESHOLD) {
    lcd.setCursor(0, 2);
    lcd.print("Enter Slot ID (1-5): ");

    if (key >= '1' && key <= ('0' + TOTAL_SLOTS)) {
      int slotID = key - '0';
      int index = slotID - 1;

      if (entryTime[index] != 0) { 
        long duration_ms = millis() - entryTime[index];
        long fee_cents = calculateFee(duration_ms);
        float fee_dollars = (float)fee_cents / 100.0;
        
        lcd.setCursor(0, 3);
        lcd.print("Fee: $");
        lcd.print(fee_dollars, 2); 
        lcd.print(" Press # to Pay");

        while (key != '#') {
            key = keypad.getKey();
            if (key == '*') break; // Allows escape
            delay(10); 
        }

        if (key == '#') {
          // Logic: Payment confirmed, allow exit
          lcd.setCursor(0, 2);
          lcd.print("Payment OK. EXITING! ");
          lcd.setCursor(0, 3);
          lcd.print("Thank You.           ");
          
          setGate(exitServo, 90); 
          
          entryTime[index] = 0;
          availableSlots++;
          
          delay(4000); 
          setGate(exitServo, 0); // Close gate
        }
      } else {
        lcd.setCursor(0, 3);
        lcd.print("Vacant Slot or Invalid ID.");
      }
    }
  }

  // D. UPDATE LCD SLOT COUNT 
  lcd.setCursor(14, 1);
  lcd.print(availableSlots);
  lcd.print("  "); 
}
```






