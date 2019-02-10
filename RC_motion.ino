#include <Servo.h>
#include <RedBot.h>
#include <CircularBuffer.h>

#define FRAME_ELEMS   2                               /** Number of elements in the frame */
#define FRAME_SIZE    2 + 2 * FRAME_ELEMS - 1         /** Number of bytes in frame (including '[', ']', ',') */
byte read_elems[FRAME_ELEMS];                         /** Here we store the read elements */
bool elems_available = false;                         /** Becomes true when data is read; false when data has been used */
CircularBuffer<byte, FRAME_SIZE> ser_buff;            /** Here we stora the raw buffer */

#define GRABBER_PIN   3                               /** The control pin of the servo grabber */
Servo grabber;                                        /** Servo instance of the grabber */


RedBotMotors motors;
int buttonPin = 12;

void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(9600);

  grabber.attach(GRABBER_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  int j = 0;
  ser_comms_handle();

  if (elems_available) {
    elems_available = false;

    if (read_elems[0] == '1') {
      grab();
    }
    else {
      letgo();
    }
  }
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
