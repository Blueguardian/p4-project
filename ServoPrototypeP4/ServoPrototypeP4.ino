#include <Servo.h>

Servo wrist;
Servo fingers;


int wristpin = 10;
int fingerPin = 9;

int inputinfo[2];



unsigned long oldtimer;
unsigned long newtimer;
int timeresult;

int newinput;

void setup() {
  //wrist.attach(wristpin);
  fingers.attach(fingerPin);
  Serial.begin(9600);
/*
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
*/
}

void loop() {
  
  if(Serial.available() > 0){
    inputproc();
  }
  
  //Servo id's : 0 = analysis, 1 = wrist, 2 = close, 3 = open


  switch(inputinfo[0]){
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
     // wrist.write(InPos(input));
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

  inputinfo[0] = 0;
  
  newtimer = millis();
  timeresult = newtimer - oldtimer;
  if(timeresult > 30){
    fingers.write(90); 
  }
  
  
}


//int InId (String id){
  //String output = id.substring(0,2);
  //return output.toInt();
//}

/*int InPos (String pos){
  String input1 =  
  String input2
  String input3
  //String output = pos.substring(2,8);
  return output.toInt();
} */
void inputproc(){
  String info;
  for(int i=0; i<=3; i++){
    info = Serial.read();
    inputinfo[i]=info.toInt();
  }
}



  
