#include "CAR.h"
#include "stm32l476xx.h"
#include <stdbool.h>
#define wheel_to_feet .785
#define turn_value .3245

//Function Headers
void gpio_Write(GPIO_TypeDef *port, uint8_t pin, bool val);
void LCD_Send4Bits(uint8_t data);
void Toggle(void);
bool gpio_Read(GPIO_TypeDef *port, uint8_t pin);

bool gpio_Read(GPIO_TypeDef *port, uint8_t pin){
	uint16_t x = 0;
	x |= port->IDR;
	x &= (1 << pin);
	if(x == (1 << pin)){
		return true;
	}
	else{
		return false;
	}
}

void delay_ms(unsigned int ms) {
	unsigned int i,j;
	for (i=0;i<ms;i++){
		{
			for(j=0;j<300;j++);
		}
	}
}

void delay_us(unsigned int us) {
	unsigned int i,j;
	for (i=0;i<us;i++){
		{
			for(j=0;j<30;j++);
		}
	}
}

void gpio_Write(GPIO_TypeDef *port, uint8_t pin, bool val){
	if(val == true){
		port->ODR |= (1 << pin); //write a 1
	} else {
		port->ODR &= ~(1 << pin); //write a 0
	} 
}

//Global Variables
	
	volatile int state = 0;
	volatile int numberOfCommands = 0;
	volatile int value[100];
	volatile int command[100];
	//volatile int j = 0;
  volatile int currentValue = 0;

	static bool motorRight[4][4] = {
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1},
    {1, 0, 0, 1}
};
	
	static bool motorLeft[4][4] = {
    {1, 1, 0, 0},
    {1, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 1, 1, 0}
};
		//forward: -1
		//Backwards: -2
		//Left: -3
		//right: -4
		//#: -5
static int key_map[4][4] = {
	{1, 2, 3, -1},
	{4, 5, 6, -2},
	{7, 8, 9, -6},
	{-3, 0, -5, -4},	
};

void buzz() {
    for (int i = 0; i < 400; i++) {
        gpio_Write(GPIOB, 12, 1);   // Turn on buzzer
        delay_ms(1);                // delay
        gpio_Write(GPIOB, 12, 0);   // Turn off buzzer
        delay_ms(1);
    }
}

void keypad_init(void){
	//clock initialization
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	//Columns
	GPIOB->MODER &= 0xFF00FFFF; // sets PB8-11 as inputs 
	
	GPIOB->PUPDR &= 0xFF00FFFF; // Clear pupdr
	GPIOB->PUPDR |= 0x00550000; // sets PB8-11 as pull up
	
	//Rows
	GPIOB->MODER &= 0x00FFFFFF; //clears MODER B12-15
	GPIOB->MODER |= 0x55000000; //Sets PB12-15 as outputs

	GPIOB->OTYPER &= 0x0FFF; //Clears OTYPER PB12-15
	GPIOB->OTYPER |= 0xF000;	//Sets PB12-15 as open-drain
}

int keypad_scan(void){	
	//Initialize varialbes
	int row, col, ColumnPressed;
	int key = 0;
	bool press = false;
	
		gpio_Write(GPIOB, 12, 0);
		gpio_Write(GPIOB, 13, 0);
		gpio_Write(GPIOB, 14, 0);
		gpio_Write(GPIOB, 15, 0);
	
	while(!press){
		//check whether any key has been pressed
		//1. output zeros an all row pins		
		if(GPIOB->IDR != 0x0000F00){
			press = true;
		}	
	}	
	delay_ms(25);
		

	//Identify the column of the key pressed
		ColumnPressed = 0;
		for(col = 0; col < 4; col++){
			if(!gpio_Read(GPIOB, col+8)){
					ColumnPressed = col;
			}
		}
		
	//Identify the row of the column pressed
		for(row = 0; row < 4; row++){
		//set up the row outputs
			gpio_Write(GPIOB, row + 12, 0);
		//Read the column inputs after a short delay
			delay_ms(1);
						
		//Check the column inputs			
			if(!gpio_Read(GPIOB, ColumnPressed + 8)){ //If the input from column pin ColumnPressed is zero
				key = key_map[row][ColumnPressed];
			}				
			gpio_Write(GPIOB, row + 12, 1); //Set row back to 1			
		}
		delay_ms(300);
		return key;
}

void motor_init(){
	//clock initialization
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	//Motor
	GPIOB->MODER &= 0xFFFF0000; // Clears MODER bits for PB0-PB3
	GPIOB->MODER |= 0x00005555; // Set B0-B7 as outputs
}
void DRIVE(int feet){
		//preSet
		float wheelRotations = (feet * wheel_to_feet);
		float motorRotations = wheelRotations*512;
		long fullTurns = (int)motorRotations;
			
		for(int k = 1; k <= fullTurns; k++){
			for(int i = 0; i <= 3; i++){
				for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorLeft[i][j]);
					gpio_Write(GPIOB, j+4, motorRight[i][j]);
				}
				delay_ms(1); //edit accordingly
				delay_us(7);
			}
		}	
	}

void turnLeft(){
	float motorRotaions = ((turn_value) * 512);
	long fullTurns = (int)motorRotaions;
	for(int k = 1; k <= fullTurns; k++){
		for(int i = 0; i <= 3; i++){
			for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorRight[i][j]);
					gpio_Write(GPIOB, j+4, motorRight[i][j]);
				}
				delay_ms(1); //edit accordingly
				delay_us(7);
		}
	}	
}

void turnRight(){
	float motorRotaions = ((turn_value) * 512);
	long fullTurns = (int)motorRotaions;
	for(int k = 1; k <= fullTurns; k++){
			for(int i = 0; i <= 3; i++){
				for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorLeft[i][j]);
					gpio_Write(GPIOB, j+4, motorLeft[i][j]);
				}
				delay_ms(1); //edit accordingly
				delay_us(7);
			}
		}	
}

void turn_180(){
	float motorRotaions = ((turn_value * .985) * 1024);
	long fullTurns = (int)motorRotaions;
	for(int k = 1; k <= fullTurns; k++){
			for(int i = 0; i <= 3; i++){
				for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorRight[i][j]);
					gpio_Write(GPIOB, j+4, motorRight[i][j]);
				}
				delay_ms(1); //edit accordingly
				delay_us(7);
		}
	}	
}

void LEFT(int feet){
	turnLeft();
	delay_ms(200);
	DRIVE(feet);
}

void RIGHT(int feet){
	turnRight();
	delay_ms(200);
	DRIVE(feet);
}

void BACKWARDS(int feet){
	turn_180();
	delay_ms(200);
	DRIVE(feet);
}

void user_input(void) {
    int input = keypad_scan();

    if (input == -5) { //changes state if # entered
        state = 1;
    } 
    else if (input < 0) {	//if a command is input, store the value and command arrays
        value[numberOfCommands] = currentValue;
        currentValue = 0;
        command[numberOfCommands] = input;
        //j++;
        numberOfCommands++;
    } 
    else if (input >= 0) { //stores user input values
        currentValue = currentValue * 10 + input; //multi digit number
    }
}
