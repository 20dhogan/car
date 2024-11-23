#include "stm32l476xx.h"
#include "CAR.h"

extern int state;
extern int value[];
extern int command[];
extern int numberOfCommands;
extern int currentValue;

int main(void){	
	//Add init functions here
	motor_init();
	keypad_init();
	ultraSonic_init();
	TIM2_Init();
	while(1){
		float cmDistance = sonicDistance();
	}
	

	while(1){
		if(state == 0){
			user_input();
		}
		
		if(state == 1){
			//Drew's assignment
			for(int i = 0; i < numberOfCommands; i++){
				delay_ms(200);
				switch(command[i]){
					case -1:
						DRIVE(value[i]);
					break;
					
					case -2:
						BACKWARDS(value[i]);
					break;
					
					case -3:
						LEFT(value[i]);
					break;
					
					case -4:
						RIGHT(value[i]);
					break;					
				}					
			}
			state = 0;
			numberOfCommands = 0;
			currentValue = 0;
		}		
	}		
}	
