#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Tone.h>

#define dirPin 12
#define stepPin 13
long microStep = 16;
#define stepsPerRevolution 1600
#define SPEED 250

#define BUTTON 9
#define PGM 2
#define RELAY 4

volatile int pgmmode = 0;
boolean relayState = false;
boolean centered = false;
char serInput;
unsigned long Timeout = millis();
unsigned long timeOut;
int relayTimeout;
int speed = 5000;
boolean down = false;
boolean testing = false;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define WIRE Wire
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declarations for click encoder
ClickEncoder *encoder;
int16_t last, value;

Tone motor;

// Function declarations
void i2cScan();
void blankHeader();
void timerIsr();
int readRotaryEncoder();
void pgmButton();
boolean encoderButton();
void relayTrigger(int);
void moveBook(boolean);
int getReading();
void findSlop();
void linearityTest();
void findCenter();

void setup()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x64)
  display.clearDisplay();
  // display.drawBitmap(0, 0, Outer_Edge_Small_Logo, 128, 64, 1);
  display.display();
  WIRE.begin();

  pinMode(RELAY, OUTPUT);
  pinMode(PGM, INPUT_PULLUP);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  motor.begin(stepPin);

  attachInterrupt(digitalPinToInterrupt(PGM), pgmButton, FALLING);

  Serial.begin(115200);
  Serial.println(F("Livescan Motor Test"));
  encoder = new ClickEncoder(7, 8, 9);
  encoder->setAccelerationEnabled(false);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  last = -1;
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(F(" Livescan Motor Test "));
  display.println(F("      Version 1.0    "));
  display.println(F("    Copyright 2023   "));
  display.println(F(" The Livescanner LLC "));
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.display();
  pgmmode = 0;
  relayTrigger(0);
}

void loop()
{
  if (millis() >= Timeout) // Stub for timer operations
  {
    Timeout = millis() + 10000;
    // relayTrigger(0);
  }
  if (Serial.available())
  {
    serInput = Serial.read();
  }
  if (serInput == 'c')
  {
    Serial.flush();
    testing = false;
  }
  if (serInput == 'l')
  {
    Serial.flush();
    testing = true;
  }
  if (serInput == 'r')
  {
    Serial.flush();
    switch (digitalRead(dirPin))
    {
    case true:
      digitalWrite(dirPin, LOW);
      break;
    case false:
      digitalWrite(dirPin, HIGH);
      break;
    }
  }
  if (serInput == 'f')
  {
    Serial.flush();
    speed = speed + 100;
    Serial.println(speed);
    serInput = ' ';
  }

  if (serInput == 's')
  {
    Serial.flush();
    speed = speed - 100;
    Serial.println(speed);
    serInput = ' ';
  }
  if (testing)
  {
    linearityTest();
  }
  Serial.println(getReading());
}

void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(10000);
}
void blankHeader()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.print(F("                    "));
  display.setCursor(0, 3);
  display.print(F("                    "));
  display.setTextColor(WHITE, BLACK);
  display.setCursor(1, 1);
}
void timerIsr()
{
  encoder->service();
}
int readRotaryEncoder()
{
  value = encoder->getValue();
  if (last != value && value != 0)
  {
    last = value;
    if (value > 0)
    {
      return 1;
    }
    if (value < 0)
    {
      return -1;
    }
  }
  last = value;
  return 0;
}
void pgmButton()
{
  // static unsigned long last_interrupt_time = 0;
  // unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  // if (interrupt_time - last_interrupt_time > 200)
  // {
  if (pgmmode == 1)
  {
    pgmmode = 0;
  }
  else
  {
    pgmmode = 1;
  }
  delay(100);
#ifdef DEBUG
  Serial.print(F("pgmmode = "));
  Serial.println(pgmmode);
#endif
  // }
  //   if (pgmmode == 0)
  //   {
  //     timeOut = millis();
  // #ifdef DEBUG
  //     Serial.print(F("timeOut = "));
  //     Serial.println(timeOut);
  // #endif
  //   }
  //   last_interrupt_time = interrupt_time;
}
boolean encoderButton()
{
  if (!digitalRead(BUTTON))
  {
    return true;
  }
  else
  {
    return false;
  }
}
void relayTrigger(int x)
{
  if (x == 1)
  {
#ifdef DEBUG
    Serial.println(F("Turning on relay"));
#endif
    digitalWrite(RELAY, HIGH);
    relayState = true;
    Timeout = millis() + (relayTimeout * 1000);
  }
  if (x == 0)
  {
#ifdef DEBUG
    Serial.println(F("Shutting off relay"));
#endif
    digitalWrite(RELAY, LOW);
    relayState = false;
  }
}

void moveBook(boolean u)
{
  int y = EEPROM.read(60);
  long stepsMove = y * (64 * microStep); // inches to move
  Serial.print("EEPROM.read(60) = ");
  Serial.println(y);
  Serial.print("64 * microStep = ");
  Serial.println(64 * microStep);
  Serial.print("stepsMove = ");
  Serial.println(stepsMove);
#ifdef DEBUG
  Serial.println("Raising book back to starting position.");
#endif
  // Set the spinning direction
  if (u)
  {
    digitalWrite(dirPin, HIGH);
    down = false;
  }
  if (!u)
  {
    digitalWrite(dirPin, LOW);
    down = true;
  }

  long accel = microStep * 30;
  long dccel = accel;

  for (int x = accel; x > 0; x--)
  {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(SPEED + (x * 2));
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SPEED + (x * 2));
  }
  for (long x = 0; x < (stepsMove - (accel + dccel)); x++)
  {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(SPEED);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SPEED);
  }
  for (int x = 0; x <= dccel; x++)
  {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(SPEED + (x * 2));
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SPEED + (x * 2));
  }

  digitalWrite(dirPin, LOW);
}
int getReading()
{
  int samples = 10;
  int reading = 0;
  for (int x = 0; x < samples; x++)
  {
    reading = reading + analogRead(A1);
  }
  return reading / samples;
}
void findCenter()
{
  while (getReading() < 490 || getReading() > 510)
  {
    Serial.println(getReading());
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    if (getReading() < 490)
    {
      digitalWrite(dirPin, HIGH);
    }
    if (getReading() > 510)
    {
      digitalWrite(dirPin, LOW);
    }
  }
  centered = true;
  Serial.println("Starting position found.");
}
void findSlop()
{
  while (!centered)
  {
    findCenter(); // Find the starting position
  }
  int start = getReading();

  if (digitalRead(dirPin) == HIGH)
  {
    digitalWrite(dirPin, LOW);
  }
  int stepCount = 0;
  while (getReading() == start)
  {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    delay(10); // This delay is necessary to allow the ADC to settle
    stepCount++;
  }
  Serial.print("Slop stepCount = ");
  Serial.println(stepCount);
  start = getReading();
}
void linearityTest()
{
  motor.play(speed, 250);
  // delay(10); // This delay is necessary to allow the ADC to settle
}