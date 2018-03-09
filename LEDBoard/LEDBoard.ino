/*
   Program to control NXP PCA9531 LED driver, which is an 8-bit device that can control
   LEDs with PWM control, and/or as GPIOs.

   Respond to input from the Serial line to allow control in a manner for demonstration,
   and reply with status on the IO pins.
*/

#include <Wire.h>


//The device address, in write mode (add 1 to the LSB for reading):
#define ADDRESS 0b1100000  //7 bits here because it's a "feature" of the Wire library...

enum pinState {PIN_ON = 0b00, PIN_OFF = 0b01, PIN_FLASH0 = 0b10, PIN_FLASH1 = 0b11};  //These are defined in the datasheet for the PCA9531 led driver.

byte ledStatusL, ledStatusH;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  //Hold here until the COM port is connected.
  Serial.println("Connected to LED Board control");
  Serial.println("\"wx,y\" where x = pin number, y=mode");
  Serial.println("MODE  |  SETTING");
  Serial.println("  0   | PIN ON");
  Serial.println("  1   | PIN OFF");
  Serial.println("  2   | SLOW FLASH");
  Serial.println("  3   | QUICK FLASH");

  //Initialise the globals
  ledStatusL = 0b01010101;
  ledStatusH = 0b01010101;

  //Some setup required here to configure the blink rates that we want to use:
  /*
    Frequency prescaler 0
    PWM register 0
    Frequency prescaler 1
    PWM register 1

  */
  Wire.beginTransmission(ADDRESS); //enter the address of the device here (all address pins held low), bit 8 0=write, 1=read
  //Use the control register to select the correct data to send next.
  Wire.write(byte(0b00010001)); //b4 set to AutoIncrement so that we can also update the next registers at the same time.
  delay(100);
  Wire.write(byte(0b0101000));  //prescaler0 - period = 1.69s
  delay(100);
  Wire.write(byte(0b1000000));  //PWM0 50% duty cycle
  delay(100);
  Wire.write(byte(0b00001100));  //prescaler1 = 0.3s
  delay(100);
  Wire.write(byte(0b1000000));  //PWM1 50% duty cycle
  Wire.endTransmission();
  delay(100); //Let things happen on the bus and in the device.

  writeToDevice();  //
}

void loop() {

  /* Want to read the Serial buffer to see if there's any instructions waiting
      If there are some, then action those, otherwise sit and wait...

      FORMAT
      ======

      w0,2 = write LED0, mode 2
      w3,0 = write LED3, mode 0

      terminate with newline character

      MODES
      ======

      0 = off/input
      1 = on
      2 = flash slow
      3 = flash fast
  */

  String command;
  byte reading;

  if (Serial.available()) {
    //Read the data and handle it
    command = Serial.readStringUntil('\n');
    if (command.length() > 5) {
      Serial.println("Error: Command too long");
    } else {
      if (command.startsWith("w")) {
        //We have a write event
        int led = command.substring(1, 2).toInt();
        int ledMode = command.substring(3, 4).toInt();

        setPin(led, ledMode);
        writeToDevice();  //

      } else if (command.startsWith("r")) {
        //We have a read event
        reading = readPin();
        //Serial.print("Reading: ");
        // Serial.println(reading, BIN);
      } else if (command.startsWith("i")) {
        Serial.println("Reinitialising");
        //initialise the device again -- we'll just run setup() again.
        setup();
      } else {
        Serial.println("Error: Command not recognised");
      }
    }

  }

  //Might want to poll for a keepalive or something in here (e.g. could power off the LINK light on the PCB if nothing detected for >xx seconds).

}

void setPin(int pinNo, byte state) {
  //sets a pin to the stated condition. (off, on, pulsed)
  if (pinNo <= 3) {
    //Write to LEDstatusL
    bitClear(ledStatusL, (2 * pinNo)); //Clear the pin register values first.
    bitClear(ledStatusL, (1 + 2 * pinNo));
    ledStatusL = ledStatusL |= state << (2 * pinNo);
  } else {
    //Write into LEDstatusH
    pinNo = pinNo - 4; //Get things back to the same offsets as for the low register.
    bitClear(ledStatusH, (2 * pinNo)); //Clear the pin register values first.
    bitClear(ledStatusH, (1 + 2 * pinNo));
    ledStatusH = ledStatusH |= state << (2 * pinNo);
  }
  Serial.print("L: ");
  Serial.println(ledStatusL, BIN);
  Serial.print("H: ");
  Serial.println(ledStatusH, BIN);

}

byte readPin() {
  //Read the input pins (and maybe all pins)
  byte reading = 0;
  Serial.println("Reading...");
  Wire.beginTransmission(ADDRESS);  //set the LSB and make the device a read for this operation.
  //Use the control register to select the correct data to send next.
  Wire.write(byte(0b00000000));
  Wire.endTransmission();
  
  Wire.requestFrom(ADDRESS, 1); //ask for a byte from the device.
  while (Wire.available()) {
    reading = Wire.read();
    Serial.println(reading, BIN);
  }
  return reading;
}

void writeToDevice() {
  //Connect to the device in write mode
  //Write the data to control the LEDs
  //Disconnect
  Wire.beginTransmission(ADDRESS); //enter the address of the device here (all address pins held low), bit 8 0=write, 1=read
  //Use the control register to select the correct data to send next.
  Wire.write(byte(0b00010101)); //LED0 to 3, b4 set to AutoIncrement so that we can also update the high register at the same time.
  Wire.write(byte(ledStatusL));  //Led Low register (LEDs 0-3)
  Wire.write(byte(ledStatusH));  //Leds 4-7
  Wire.endTransmission();

}



