

#include "../gcode.h"

long lastFireEvent = 0;
void GcodeSuite::fireHp(int16_t info){
    digitalWrite(MJM_fire,LOW);
    SERIAL_ECHOPGM("",info);
    pinMode(MJM_fire,OUTPUT);
    while(lastFireEvent+1 >= millis()){;}
    for ( int i = 0; i < 12 ; i++){
        if (info >> i & 1){
            SERIAL_ECHOPGM("",i);
            if (i >> 0 & 1)digitalWriteFast(MJM_data1,HIGH);
            if (i >> 1 & 1)digitalWriteFast(MJM_data2,HIGH);
            if (i >> 2 & 1)digitalWriteFast(MJM_data3,HIGH);
            if (i >> 3 & 1)digitalWriteFast(MJM_data4,HIGH);
            digitalWriteFast(MJM_fire,HIGH);
            delayMicroseconds(5);
            digitalWriteFast(MJM_fire,LOW);
            delayMicroseconds(1);
            if (i >> 0 & 1)digitalWriteFast(MJM_data1,LOW);
            if (i >> 1 & 1)digitalWriteFast(MJM_data2,LOW);
            if (i >> 2 & 1)digitalWriteFast(MJM_data3,LOW);
            if (i >> 3 & 1)digitalWriteFast(MJM_data4,LOW);

            
        }
    }
    lastFireEvent = millis();
}
