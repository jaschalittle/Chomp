#include "Arduino.h"
#include "pins.h"
#include "weapons.h" // weaponsEnabled
#include "utils.h"   // safeDigitalWrite

void selfRightLeft(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_DO, HIGH);
        safeDigitalWrite(SELF_RIGHT_RIGHT_DO, LOW);
    }
}


void selfRightRight(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_DO, LOW);
        safeDigitalWrite(SELF_RIGHT_RIGHT_DO, HIGH);
    }
}


static void selfRightBoth(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_DO, HIGH);
        safeDigitalWrite(SELF_RIGHT_RIGHT_DO, HIGH);
    }
}


void selfRightOff(){
    digitalWrite(SELF_RIGHT_LEFT_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_DO, LOW);
}


void selfRightSafe(){
    digitalWrite(SELF_RIGHT_LEFT_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_DO, LOW);
    pinMode(SELF_RIGHT_LEFT_DO, OUTPUT);
    pinMode(SELF_RIGHT_RIGHT_DO, OUTPUT);
}
