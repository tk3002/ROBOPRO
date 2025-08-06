#include "mbed.h"
#include "aTB6643KQ.hpp"

Serial pc(USBTX,USBRX);
DigitalOut led(LED1);

rob::aTB6643KQ motor(PB_3,PA_10);

int main(){
	
	motor.freq(5000);//周波数の設定　単位Hz
	
	//0→ブレーキ　1→最速正転
	
	//This is a test code
	while(true){
		led=!led;
		motor=0.8;
		wait(1);
		led=!led;
		motor=0;
		wait(1);
		
		
	}
	
    return 0;
}