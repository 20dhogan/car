#include "CAR.h"
#include "stm32l476xx.h"
#include <stdbool.h>
#define wheel_to_feet .785
#define turn_value .3245
#define MAX_TIMEOUT 50000

//Function Headers
void gpio_Write(GPIO_TypeDef *port, uint8_t pin, bool val);
void LCD_Send4Bits(uint8_t data);
void Toggle(void);
bool gpio_Read(GPIO_TypeDef *port, uint8_t pin);
void LCD_Send4Bits(uint8_t data);
/*
		//forward: -1
		//Backwards: -2
		//Left: -3
		//right: -4
		//#: -5
		*/
//Global Variables
volatile int mode = 0;
volatile int state = 0;
volatile int numberOfCommands = 0;
volatile int value[100];
volatile int command[100];
volatile int currentValue = 0;
static float distanceLeft = 0;
static float distanceFront = 0;
static float errorDistanceReading = -1;
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
static int key_map[4][4] = {
	{1, 2, 3, -1},
	{4, 5, 6, -2},
	{7, 8, 9, -6},
	{-3, 0, -5, -4},	
};

//Maze Functions*********************************************************************
void FollowWallLeft(){
	distanceLeft = sonicDistanceLeft();
	distanceFront = sonicDistanceFront();
	int skipLeft = 0;
	int skipRight = 0;
	bool skipL = false;
	bool skipR = false;
	
	while(1){
		
		if(distanceLeft == errorDistanceReading){
			distanceLeft = sonicDistanceLeft();
			if(distanceLeft == errorDistanceReading){
				reCenter();
			}
		}
		
		
		if(distanceLeft > 35 && distanceLeft != errorDistanceReading){
			distanceLeft = sonicDistanceLeft();
			if(distanceLeft > 35 && distanceLeft != errorDistanceReading){
				//Our wall on the left has opened up. We need to turn left.
				DRIVE_inches(10);
				turnLeft();
				distanceFront = sonicDistanceFront();
				if(distanceFront > 20.0 && distanceFront != errorDistanceReading){
					DRIVE_inches(8);
					distanceLeft = sonicDistanceLeft();
				}
			}
		}
		
		if(distanceFront < 25.0 && distanceFront != errorDistanceReading){
			//Then we have hit a wall in the front
			if(distanceFront < 10.0){
				REVERSE_inches(5);
			}
			turnRight();
			distanceLeft = sonicDistanceLeft();
		}
		

		//Check if we should skip left wheel motor cycle
		if(skipLeft >= 2 && distanceLeft > 18 && distanceLeft != errorDistanceReading){
			skipLeft = 0;
			skipL = true;
		} else {
			if(distanceLeft > 18 && distanceLeft != errorDistanceReading){
				skipLeft++;
			}
		}

		//Check if we should skip right wheel motor cycle
		if(skipRight >= 2 && distanceLeft < 16 && distanceLeft != errorDistanceReading){
			skipRight = 0;
			skipR = true;
		} else {
			if(distanceLeft < 16 && distanceLeft != errorDistanceReading){
				skipRight++;
			}
		}
		

		for(int i = 0; i <= 3; i++){
			for(int j = 0; j <= 3; j++){
				if(skipL == false){
					gpio_Write(GPIOB, j, motorLeft[i][j]);
				}
						
				if(skipR == false){
					gpio_Write(GPIOB, j+4, motorRight[i][j]);
				}
				delay_micro_s(4); //edit accordingly
			}
		}
		
		if(skipR == true){
			skipR = false;
		}
		if(skipL == true){
			skipL = false;
		}
	}
}


void reCenter(void){
	bool edgeFound = false;
	int turnDeg = 200;
	int offSet = 10;
	
	for(int i = 0; i < turnDeg; i++){
		turnRight_deg(1);
		distanceLeft = sonicDistanceLeft();
		if(distanceLeft != errorDistanceReading){
			i = turnDeg;
			edgeFound = true;
		}
	}
		
		if(edgeFound == true){
			//we have found the edge, we need to go a little farther to be perpendeicular to wall
			turnRight_deg(offSet);
		} else {
			
			//We need to turn back and try the other way.
			turnRight_deg(turnDeg);
			//Now that we have turned back, lets try the other direction
			for(int i = 0; i < turnDeg; i++){
				turnRight_deg(1);
				distanceLeft = sonicDistanceLeft();
				if(distanceLeft != errorDistanceReading){
					i = turnDeg;
					edgeFound = true;
				}
			}
			
			if(edgeFound == true){
				//we have found the edge, we need to go a little farther to be perpendeicular to wall
				turnRight_deg(offSet);
			}
			
		}
}


//Funciton from book
void SysTick_Initialize(unsigned int ticks){
	//Disable SysTick IRQ and SysTick counter
	SysTick->CTRL = 0;
	
	//Set reload register
	SysTick->LOAD = ticks - 1;
	
	//set interupt priority of SysTick
	//Make SysTick Least urgent (ie, highest priority number)
	//_NVIC_PRIO_BITS: number of bites for priority levels, defined in CMSIS
	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 5);
	
	//Reset the SysTick counter value
	SysTick->VAL = 0;
	
	//Select processor clock
	//1 = processor clock; 0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	
	//Enables Systick exception request
	//1 = counting down to zero asserts the SysTick exception request
	//0 = counting down to zero does not assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	//enable SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void){
	if(mode == 0){
			distanceLeft = sonicDistanceLeft();
			distanceFront = sonicDistanceFront();
	}
}

//Motor Functions********************************************************************
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
				delay_ms(2); //edit accordingly
				delay_us(50);
			}
		}	
}
void DRIVE_inches(float inches){
		//preSet
		float feet = inches/12;
		float wheelRotations = (feet * wheel_to_feet);
		float motorRotations = wheelRotations*512;
		long fullTurns = (int)motorRotations;
			
		for(int k = 1; k <= fullTurns; k++){
			for(int i = 0; i <= 3; i++){
				for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorLeft[i][j]);
					gpio_Write(GPIOB, j+4, motorRight[i][j]);
				}
				delay_ms(2); //edit accordingly
				delay_us(50);
			}
		}	
}
void REVERSE_inches(float inches){
		//preSet
		float feet = inches/12;
		float wheelRotations = (feet * wheel_to_feet);
		float motorRotations = wheelRotations*512;
		long fullTurns = (int)motorRotations;
			
		for(int k = 1; k <= fullTurns; k++){
			for(int i = 0; i <= 3; i++){
				for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorRight[i][j]);
					gpio_Write(GPIOB, j+4, motorLeft[i][j]);
				}
				delay_ms(2); //edit accordingly
				delay_us(50);
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
				delay_ms(2); //edit accordingly
				delay_us(50);
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
				delay_ms(2); //edit accordingly
				delay_us(50);
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
void turnLeft_deg(int rotation){
	for(int k = 1; k <= rotation; k++){
		for(int i = 0; i <= 3; i++){
			for(int j = 0; j <= 3; j++){
					gpio_Write(GPIOB, j, motorRight[i][j]);
					gpio_Write(GPIOB, j+4, motorRight[i][j]);
				}
				delay_ms(2); //edit accordingly
				delay_us(50);
		}
	}
}

void turnRight_deg(int rotation){
	for(int k = 1; k <= rotation; k++){
		for(int i = 0; i <= 3; i++){
			for(int j = 0; j <= 3; j++){
				gpio_Write(GPIOB, j, motorLeft[i][j]);
				gpio_Write(GPIOB, j+4, motorLeft[i][j]);
			}
			delay_ms(2); //edit accordingly
			delay_us(50);
		}
	}	
}

//Keypad Functions *************************************************************
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


//UltraSonic Functions*********************************************************************
void ultraSonic_init(void){
	//Enalbe gpioc clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	//set pc0,1,2 as outputs
	GPIOC->MODER &= 0xFFFFFFFC0; //clear pins
	GPIOC->MODER |= 0x0000000D5;
	
	//set pc3,4,5 as inputs
	GPIOC->MODER &= 0xFFFFF03F;
	
	//set pc3,4,5 as pull down
	GPIOC->PUPDR &= 0xFFFFF03F; //clear pins
	GPIOC->PUPDR |= 0x00000A80;
	
	//initalize timer
	TIM2_Init();
}

void TIM2_Init(void) {
    // 1. Enable the TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // 2. Configure prescaler for 1 MHz (1 탎 per tick)
    TIM2->PSC = 16 - 1;  // System Clock = 16 MHz

    // 3. Set auto-reload value to max (32-bit counter)
    TIM2->ARR = 0xFFFFFFFF;

    // 4. Reset the counter to zero
    TIM2->CNT = 0;

    // 5. Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;
}


float sonicDistanceFront(void) {
    // Send a 10 탎 pulse on the Trigger pin (PC0)
    gpio_Write(GPIOC, 0, 1);
    delay_us(10);  // Ensure a 10 탎 pulse
    gpio_Write(GPIOC, 0, 0);

    uint32_t start_time = 0;
    uint32_t stop_time = 0;
    uint32_t timeout = 0;

    // Wait for rising edge on Echo pin (PC1) with timeout
    timeout = TIM2->CNT;  // Capture the current timer value for timeout reference
    while (!gpio_Read(GPIOC, 3)) { // Block until pin goes HIGH
        if ((TIM2->CNT - timeout) > MAX_TIMEOUT) { // Check timeout
            return errorDistanceReading;  // Return -1 to indicate timeout error
        }
    }
    start_time = TIM2->CNT;  // Start timer count

    // Wait for falling edge on Echo pin (PC1) with timeout
    timeout = TIM2->CNT;  // Reset timeout reference
    while (gpio_Read(GPIOC, 3)) { // Block until pin goes LOW
        if ((TIM2->CNT - timeout) > MAX_TIMEOUT) { // Check timeout
            return errorDistanceReading;  // Return -1 to indicate timeout error
        }
    }
    stop_time = TIM2->CNT;  // Stop timer count

    // Calculate pulse duration in microseconds
    uint32_t pulse_duration = stop_time - start_time;

    // Calculate distance in cm
    float distance = ((pulse_duration * 0.0343f) / 2.0f) / 4;

    return distance;
}

float sonicDistanceLeft(void) {
    // Send a 10 탎 pulse on the Trigger pin (PC1)
    gpio_Write(GPIOC, 1, 1);
    delay_us(10);  // Ensure a 10 탎 pulse
    gpio_Write(GPIOC, 1, 0);

    uint32_t start_time = 0;
    uint32_t stop_time = 0;
    uint32_t timeout = 0;

    // Wait for rising edge on Echo pin (PC4) with timeout
    timeout = TIM2->CNT;  // Capture the current timer value for timeout reference
    while (!gpio_Read(GPIOC, 4)) { // Block until pin goes HIGH
        if ((TIM2->CNT - timeout) > MAX_TIMEOUT) { // Check timeout
            return errorDistanceReading;  // Return -1 to indicate timeout error
        }
    }
    start_time = TIM2->CNT;  // Start timer count

    // Wait for falling edge on Echo pin (PC4) with timeout
    timeout = TIM2->CNT;  // Reset timeout reference
    while (gpio_Read(GPIOC, 4)) { // Block until pin goes LOW
        if ((TIM2->CNT - timeout) > MAX_TIMEOUT) { // Check timeout
            return errorDistanceReading;  // Return -1 to indicate timeout error
        }
    }
    stop_time = TIM2->CNT;  // Stop timer count

    // Calculate pulse duration in microseconds
    uint32_t pulse_duration = stop_time - start_time;

    // Calculate distance in cm
    float distance = ((pulse_duration * 0.0343f) / 2.0f) / 4;

    return distance;
}

//Helper Functions***************************************************************************
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

void delay_micro_s(unsigned int ms) {
	unsigned int i,j;
	for (i=0;i<ms;i++){
		{
			for(j=0;j<30;j++);
		}
	}
}

void delay_us(uint32_t us) {
    TIM2->CNT = 0; // Reset the counter
    while (TIM2->CNT < us); // Wait until the counter reaches the delay time
}


void gpio_Write(GPIO_TypeDef *port, uint8_t pin, bool val){
	if(val == true){
		port->ODR |= (1 << pin); //write a 1
	} else {
		port->ODR &= ~(1 << pin); //write a 0
	} 
}

void SystemClock_Config(void) {
    //Enable HSI. From Phillips slides.
		RCC->CR |= RCC_CR_HSION;
		while((RCC->CR  & RCC_CR_HSIRDY) == 0);
}
