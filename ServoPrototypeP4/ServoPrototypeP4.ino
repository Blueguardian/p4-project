#include <Servo.h>

//Servo wrist;
Servo fingers;

//int wristpin = 9;
int fingerPin = 9;

String input;
int servoid;
int servopos = 0;

unsigned long oldtimer;
unsigned long newtimer;
int timeresult;

int newinput;

void setup() {
  //wrist.attach(wristpin);
  fingers.attach(fingerPin);
  Serial.begin(9600);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

}

void loop() {

  input = "";

  while(Serial.available()){
    input = Serial.read();  
  }
  
  servoid = InId(input);

  //Servo id's : 0 = analysis, 1 = wrist, 2 = close, 3 = open


  switch(servoid){
    case 49:
      //Serial.print("1");
      //digitalWrite(2, HIGH);
      //delay(100);
      //digitalWrite(2, LOW);
      break;

    case 50:
      //Serial.print("2");
      //digitalWrite(3, HIGH);
      //delay(100);
      //digitalWrite(3, LOW);
      break;

    case 51:
      oldtimer = millis();
      fingers.write(0);
      //delay(50);
      //Serial.print("3");
      //digitalWrite(4, HIGH);
      //delay(100);
      //digitalWrite(4, LOW);
      break;

    case 52:
      oldtimer = millis();
      fingers.write(180);
      //delay(50);
      //Serial.print("4");
      //digitalWrite(5, HIGH);
      //delay(100);
      //digitalWrite(5, LOW);
      break;
    
  }

      newtimer = millis();
      timeresult = newtimer - oldtimer;
      if(timeresult > 30){
       fingers.write(90); 
      }
  
  
}


int InId (String id){
  String output = id.substring(0,2);
  return output.toInt();
}

int InPos (String pos){
  String output = pos.substring(2,5);
  return output.toInt();
}
