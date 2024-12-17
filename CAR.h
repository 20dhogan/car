#ifndef __STM32L476R_NUCLEO_LCD_H
#define __STM32L476R_NUCLEO_LCD_H
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

//Helper Functions
void delay_ms(unsigned int ms);
void delay_us(unsigned int ms);
void delay_micro_s(unsigned int ms);
void SystemClock_Config(void);

//Keypad Functions
int keypad_scan(void);
void user_input(void);
void keypad_init(void);

//Motor Functions
void motor_init();
void DRIVE(int feet);
void DRIVE_inches(float inches);
void REVERSE_inches(float inches);
void LEFT(int feet);
void RIGHT(int feet);
void BACKWARDS(int feet);
void turnLeft(void);
void turnLeft_deg(int rotation);
void turnRight(void);
void turnRight_deg(int rotation);
void turn_180(void);

//UltraSonic functions
void ultraSonic_init(void);
float sonicDistanceFront(void);
float sonicDistanceLeft(void);
float sonicDistanceRight(void);
void TIM2_Init(void);

//Maze Solver
void FollowWallLeft(void);
void reCenter(void);
void SysTick_Initialize(uint32_t ticks);
void SysTick_handler(void);
#endif