/*
 * Author: Colin Fraser
 * DC Motor Test
 * November 4, 2021
 * 
 * Use an Arduino to drive a DC motor through the DRV8833 motor board. Want to test varying speeds and directions. 
 * Should also add tests for how to reliably do 120degree turns
 * 
 */

#define M1 5  // Motor A pins
#define M2 6

#define FORWARD     1
#define BACKWARD    2
#define STOP        0


int incomingByte = 0; // for incoming serial data

void setup() {

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

  Serial.println("select direction of movement");
  Serial.println("0.stop");
  Serial.println("1.forward1");
  Serial.println("2.forward2");
  Serial.println("3.forward3");
  Serial.println("4.forward4");
  Serial.println("5.forward5");
  Serial.println("q.backward1");
  Serial.println("w.backward2");
  Serial.println("e.backward3");
  Serial.println("r.backward4");
  Serial.println("t.backward5");

}
int  input = 0;
void loop() {

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();


  switch (incomingByte) { 
    case 'q':         // if input=q       motor turns backward slightly
      move(BACKWARD, 1);
      Serial.print("Backwards 1\n");
      break;
      
    case 'w':         // if input=w       motor turns backward slowly
      move(BACKWARD, 2);
      Serial.print("Backwards 2\n");
      break;
      
    case 'e':         // if input=e       motor turns backward moderately
      move(BACKWARD, 3);
      Serial.print("Backwards 3\n");
      break;
      
    case 'r':         // if input=r       motor turns backward quickly
      move(BACKWARD, 4);
      Serial.print("Backwards 4\n");
      break;
      
    case 't':         // if input=t       motor turns backward fast
      move(BACKWARD, 5);
      Serial.print("Backwards 5\n");
      break;
      
    case '0':         // if input=0       motor stops
      move(STOP, 0);
      Serial.print("STOP!\n");
      break;
      
    case '1':         // if input=1       motor turns forward slightly
      move(FORWARD, 1);
      Serial.print("Forwards 1\n");
      break;
      
    case '2':         // if input=2       motor turns forward slowly
      move(FORWARD, 2);
      Serial.print("Forwards 2\n");
      break;
      
    case '3':         // if input=3       motor turns forward moderately
      move(FORWARD, 3);
      Serial.print("Forwards 3\n");
      break;
      
    case '4':         // if input=4       motor turns forward quickly
      move(FORWARD, 4);
      Serial.print("Forwards 4\n");
      break;
      
    case '5':         // if input=5       motor turns forward fast
      move(FORWARD, 5);
      Serial.print("Forwards 5\n");
      break;
  }
  delay(200);
  incomingByte=0;
}
}

// Control motor direction and speed
void move(int direction, int speed) {
  int pwmVal = 50*speed; //Give 5 discrete pwm steps from 0 - 255
  if(speed == 5)
    pwmVal +=5;

  switch (direction) {
    case STOP:
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
      break;
    case FORWARD:
      analogWrite(M1, pwmVal);
      analogWrite(M2, 0);
      break;
    case BACKWARD:
      analogWrite(M1, 0);
      analogWrite(M2, pwmVal);
      break;
  }
  
}
