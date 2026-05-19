#include<IRremote.h>
int IR_pin = 11;

int relays[] = {30,31,32,33,34};
bool states[] = {0,0,0,0}; // on_off staes, initialy off all

void setup(){
  Serial.begin(9600);
  IrReceiver.begin(IR_pin);

  for(int i=0;i<4;i++){
    pinMode(relays[i],OUTPUT);
    digitalWrite(relays[i],HIGH); // off the relay initialy
  }  
}

void loop(){
  if(IrReceiver.decode()){
    int code = IrReceiver.decodedIRData.command;

    switch(code){
      case 48:
        states[0]= !states[0];
        digitalwrite(relays[0],states[0] ? LOW:HIGH);
        break;

      case 24:
        states[1]= !states[1];
        digitalwrite(relays[1],states[1] ? LOW:HIGH);
        break;

      case 122:
        states[2]= !states[2];
        digitalwrite(relays[2],states[2] ? LOW:HIGH);
        break;

      case 16:
        states[3]= !states[3];
        digitalwrite(relays[3],states[3] ? LOW:HIGH);
        break;
    }
    IrReceiver.resume();
  }
}