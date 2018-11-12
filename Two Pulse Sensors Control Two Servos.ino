/* 
   Original Arduino Sketch to detect pulses from two PulseSensors 
   has been modified by Omar Ansari and Aaron Soloway to include 
   two servo motors that move 30˚ each time a heartbeat is detected.

   This program was modified/created to explore Arduino 
   robotics' ability to facilitate interconnectedness between people meditating by 
   allowing participants to externalize their pulse using the servo motors. 
   The motor is attached to a drumstick which strikes a gong each time a 
   heart beat is detected. The ideal version of this program would include 
   a function that measures the distance between the two participants' pulses and upon 
   reaching a particular threshold creates a third output that signifies 
   that the participants' heartbeats have become more synchronous.

   Omar Ansari & Aaron Soloway
   California College of the Arts
   Interaction Design - Undergraduate Program

   Here is a link to the tutorial
   https://pulsesensor.com/pages/two-or-more-pulse-sensors

   Copyright World Famous Electronics LLC - see LICENSE
   Contributors:
     Joel Murphy, https://pulsesensor.com
     Yury Gitman, https://pulsesensor.com
     Bradford Needham, @bneedhamia, https://bluepapertech.com

   Licensed under the MIT License, a copy of which
   should have been included with this software.

   This software is not intended for medical use.
*/

/*
   Every Sketch that uses the PulseSensor Playground must
   define USE_ARDUINO_INTERRUPTS before including PulseSensorPlayground.h.
   Here, #define USE_ARDUINO_INTERRUPTS false tells the library to
   not use interrupts to read data from the PulseSensor.

   If you want to use interrupts, simply change the line below
   to read:
     #define USE_ARDUINO_INTERRUPTS true

   Set US_PS_INTERRUPTS to false if either
   1) Your Arduino platform's interrupts aren't yet supported
   by PulseSensor Playground, or
   2) You don't wish to use interrupts because of the side effects.

   NOTE: if US_PS_INTERRUPTS is false, your Sketch must
   call pulse.sawNewSample() at least once every 2 milliseconds
   to accurately read the PulseSensor signal.
*/

#include <Servo.h>

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>


/*
   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the multi-sensor Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensorAmped_2_Sensors

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Number of PulseSensor devices we're reading from.
*/
const int PULSE_SENSOR_COUNT = 2;

/*
     PULSE_POWERx = the output pin that the red (power) pin of
      the first PulseSensor will be connected to. PulseSensor only
      draws about 4mA, so almost any micro can power it from a GPIO.
      If you don't want to use pins to power the PulseSensors, you can remove
      the code dealing with PULSE_POWER0 and PULSE_POWER1.
     PULSE_INPUTx = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINKx = digital Output. Connected to an LED (must have at least
      470 ohm resistor) that will flash on each detected pulse.
     PULSE_FADEx = digital Output. PWM pin onnected to an LED (must have
      at least 470 ohm resistor) that will smoothly fade with each pulse.

     NOTE: PULSE_FADEx must be pins that support PWM.
       If USE_INTERRUPTS is true, Do not use pin 9 or 10 for PULSE_FADEx
       because those pins' PWM interferes with the sample timer.
*/
const int PULSE_INPUT0 = A0;
const int PULSE_BLINK0 = 11;    // Pin 11 is the on-board LED
//const int PULSE_FADE0 = 5;

const int PULSE_INPUT1 = A1;
const int PULSE_BLINK1 = 3;
//const int PULSE_FADE1 = 5;

const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

/*
   All the PulseSensor Playground functions.
   We tell it how many PulseSensors we're using.
*/
PulseSensorPlayground pulseSensor(PULSE_SENSOR_COUNT);

int window1[3] = {0, 0, 0};
int window2[3] = {0, 0, 0};
int threshold = 40;
unsigned long lastBeatTime1 = 0;
unsigned long lastBeatTime2 = 0;





Servo heart01;
const int SERVO_PIN01 = 6;
int pos01 = 90;
int strikePos01 = 120;


Servo heart02;
const int SERVO_PIN02 = 5;
int pos02 = 90;
int strikePos02 = 120;


void setup() {
  /*
     Use 250000 baud because that's what the Processing Sketch expects to read,
     and because that speed provides about 25 bytes per millisecond,
     or 50 characters per PulseSensor sample period of 2 milliseconds.

     If we used a slower baud rate, we'd likely write bytes faster than
     they can be transmitted, which would mess up the timing
     of readSensor() calls, which would make the pulse measurement
     not work properly.
  */
  Serial.begin(115200);

  /*
     Configure the PulseSensor manager,
     telling it which PulseSensor (0 or 1)
     we're configuring.
  */

  heart01.attach(SERVO_PIN01);
  heart01.write(pos01);

  heart02.attach(SERVO_PIN02);
  heart02.write(pos02);



  pulseSensor.analogInput(PULSE_INPUT0, 0);
  pulseSensor.blinkOnPulse(PULSE_BLINK0, 0);
  //pulseSensor.fadeOnPulse(PULSE_FADE0, 0);

  pulseSensor.analogInput(PULSE_INPUT1, 1);
  pulseSensor.blinkOnPulse(PULSE_BLINK1, 1);
  //pulseSensor.fadeOnPulse(PULSE_FADE1, 1);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);


  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try changing USE_ARDUINO_INTERRUPTS to false.
    */
    for (;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PULSE_BLINK0, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK0, HIGH);
      delay(50);
    }
  }
}

void loop() {



  // write the latest sample to Serial.
  pulseSensor.outputSample();

  //moveServo01(pulseSensor.getLatestSample(0));
  //moveServo02(pulseSensor.getLatestSample(1));


  unsigned long currTime = millis();

  /*
     If a beat has happened on a given PulseSensor
     since we last checked, write the per-beat information
     about that PulseSensor to Serial.
  */
  for (int i = 0; i < PULSE_SENSOR_COUNT; ++i) {
    if (pulseSensor.sawStartOfBeat(i)) {
      pulseSensor.outputBeat(i);
      if (i == 0) {
        //moveServo01(strikePos01);
        heart01.write(strikePos01);
        Serial.println("beat1");
        lastBeatTime1 = currTime;
      }
      else {
        //moveServo02(strikePos02);
        heart02.write(strikePos02);
        Serial.println("beat2");
        lastBeatTime2 = currTime;

      }

    }
  }

  /*
     Wait a bit.
     We don't output every sample, because our baud rate
     won't support that much I/O.
  */
  delay(20);

  if ((currTime - lastBeatTime1) > 250) {
    heart01.write(pos01);
  }

  if ((currTime -  lastBeatTime2) > 250) {
    heart02.write(pos02);
  }
}

void moveServo01(int value) {
  pos01 = map(value, 0, 1023, 0, 180);
  heart01.write(pos01);
}

void moveServo02(int value) {
  pos02 = map(value, 0, 1023, 0, 180);
  heart02.write(pos02);
}

void updateWindow1(int value) {
  window1[2] = window1[1];
  window1[1] = window1[0];
  window1[0] = value;
}

void updateWindow2(int value) {
  window2[2] = window2[1];
  window2[1] = window2[0];
  window2[0] = value;
}

bool isTransient1(int thresh) {
  if ((window1[2] - window1[0]) > thresh) {
    return true;
  }
  else {
    return false;
  }
}
bool isTransient2(int thresh) {
  if ((window2[2] - window2[0]) > thresh) {
    return true;
  }
  else {
    return false;
  }
}

bool isbeat1(unsigned long currentTime, int thresh, unsigned long beatTimeout) {
  if ((isTransient1(thresh) && ((currentTime - lastBeatTime1) > beatTimeout))) {
    lastBeatTime1 = currentTime;
    return true;
  }
  else {
    return false;
  }
}

bool isbeat2(unsigned long currentTime, int thresh, unsigned long beatTimeout) {
  if ((isTransient2(thresh) && ((currentTime - lastBeatTime2) > beatTimeout))) {
    lastBeatTime2 = currentTime;
    return true;
  }
  else {
    return false;
  }
}






