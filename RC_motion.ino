#include <Servo.h>
#include <RedBot.h>
#include <CircularBuffer.h>

#define FRAME_ELEMS   3                               /** Number of elements in the frame */
#define FRAME_SIZE    2 + 2 * FRAME_ELEMS - 1         /** Number of bytes in frame (including '[', ']', ',') */
byte read_elems[FRAME_ELEMS];                         /** Here we store the read elements */
bool elems_available = false;                         /** Becomes true when data is read; false when data has been used */
CircularBuffer<byte, FRAME_SIZE> ser_buff;            /** Here we stora the raw buffer */

#define TRIGGER_PIN   9                               /** This pin is activated from the voice assistant */

#define KNOB_PIN      A6

#define GRABBER_PIN   3                               /** The control pin of the servo grabber */
#define GRABBER_SENSOR 11             
Servo grabber;                                        /** Servo instance of the grabber */



RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);
int buttonPin = 12;

void setup() {
  // put your setup code here, to run once:
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  pinMode(GRABBER_SENSOR, INPUT_PULLUP);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  
  Serial.begin(9600);

  grabber.attach(GRABBER_PIN);
}

void loop() {
  WAITING();

  ROTATE1();
  FWDCENTER();
  
//  ser_comms_handle();

//  if (elems_available) {
//    elems_available = false;

//    digitalWrite(13, 1);
//    delay(100);
//    digitalWrite(13, 0);

//    if (read_elems[0] == 2) {
//      grab();
//    }
//    else {
//      letgo();
//    }
//  }
}

void WAITING() {
  while(digitalRead(TRIGGER_PIN)) {
//    //Serial.println("Waiting");
    //digitalWrite(13, 1);
    //delay(10);
//    break;
  }
  digitalWrite(13, 0);
}

void ROTATE1() {
//  float v = analogRead(KNOB_PIN) / 1023.0;
  float v = 0.245;
  set_speed(-v, v);
  Serial.flush();

  while (true) {  
    ser_comms_handle();  
    if (elems_available)
      elems_available = false;
      if (read_elems[0] == 2)
        break;
  }

  set_speed(v, -v);
  delay(450);
  set_speed(0, 0);
  delay(2000);
  Serial.flush();  

  while (true) {
    ser_comms_handle();

    if (elems_available) {
      elems_available = false;

      if (read_elems[0] == 2) {
        float x = 2.0 * read_elems[1] / 255.0 - 1.0;
        if (x > -0.25 && x < 0.25) {          
          set_speed(0, 0);
          break;
        }
        else {
          float c = 0.38;
          set_speed(c*x, -c*x);
        }
//        else if (x <= -0.25) {                    
//          set_speed(v, 0);
//        }
//        else {                              
//          set_speed(0, v);
//        }
        digitalWrite(13, 1);
      }
      else {
        digitalWrite(13, 0);
      }
    }
  }
}

int countsPerRev = 192;
float wheelDiam = 6.5;
float wheelCirc = PI*wheelDiam;

void driveStraight(float dist, int Pwr){
  long lCount = 0;
  long rCount = 0;
  long targetCount;
  float numRev;
  
// variables for tracking the left and right encoder counts
  long prevlCount, prevrCount;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  // variables for setting left and right motor power
  int leftPower = Pwr;
  int rightPower = Pwr;

  // variable used to offset motor power on right vs left to keep straight.
  int offset = 5;  // offset amount to compensate Right vs. Left drive

  numRev = dist / wheelCirc;  // calculate the target # of rotations
  targetCount = numRev * countsPerRev;    // calculate the target gear count

  encoder.clearEnc(BOTH);    // clear the encoder count
  delay(100);  // short delay before starting the motors.

  motors.drive(Pwr);  // start motors 

  while (digitalRead(GRABBER_SENSOR))
  {
    // while the right encoder is less than the target count -- debug print 
    // the encoder values and wait -- this is a holding loop.
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);

    motors.leftDrive(leftPower);
    motors.rightDrive(rightPower);

    // calculate the rotation "speed" as a difference in the count from previous cycle.
    lDiff = (lCount - prevlCount);
    rDiff = (rCount - prevrCount);

    // store the current count as the "previous" count for the next cycle.
    prevlCount = lCount;
    prevrCount = rCount;

    // if left is faster than the right, slow down the left / speed up right
    if (lDiff > rDiff) 
    {
      leftPower = leftPower - offset;
      rightPower = rightPower + offset;
    }
    // if right is faster than the left, speed up the left / slow down right
    else if (lDiff < rDiff) 
    {
      leftPower = leftPower + offset;  
      rightPower = rightPower - offset;
    }
    delay(50);  // short delay to give motors a chance to respond.
  }
  // now apply "brakes" to stop the motors.
  motors.brake();  
}

void FWDCENTER() {
//  set_speed(0.5, 0.5);
//  while (digitalRead(GRABBER_SENSOR)) {}
//  set_speed(0, 0);
  driveStraight(100, 100);
  delay(500);
  grab();
  delay(500);
  set_speed(-0.5, -0.5);
}

void ser_comms_handle(void) {
  int n = Serial.available(); // # of pending input bytes

  for (int i = 0; i < n; i++) {
    ser_buff.push(Serial.read());
  }

  while (ser_buff.size()) {
    if (ser_buff.first() != '[') {
      ser_buff.shift();
    }
    else {
      break;
    }
  }

  if (ser_buff.size() == FRAME_SIZE &&
      ser_buff.first() == '[' &&
      ser_buff.last() == ']') {
      for (int i = 0; i < FRAME_ELEMS; i++) {
        ser_buff.shift();
        read_elems[i] = ser_buff.shift();
        elems_available = true;
        Serial.println(read_elems[i]);
      }
      ser_buff.clear();
  }
}

void grab() {
  grabber.write(160);
  delay(15);
}

void letgo() {
  grabber.write(52);
  delay(15);
}

void set_speed(float right_sp, float left_sp){
  float r_v = right_sp*255;
  float l_v = left_sp*255;
  motors.rightDrive(r_v); 
  motors.leftDrive(l_v);
}
