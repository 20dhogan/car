#include "stm32l476xx.h"
#include "CAR.h"

extern int mode;
extern int state;
extern int value[];
extern int command[];
extern int numberOfCommands;
extern int currentValue;

int main(void){
	//init functions
	SystemClock_Config();
	ultraSonic_init();
	motor_init();
	keypad_init();
	SysTick_Initialize(1000000);
	
	bool done = false;
	while(!done){
		int mo = keypad_scan();
		if(mode == 0 || mode == 1){
			mode = mo;
			done = true;
		}
	}

	if(mode == 0){
		int ans = 0;
		while(ans != -5){
			ans = keypad_scan();
		}
		FollowWallLeft();
			
	} else {
		
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
}	
