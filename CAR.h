#ifndef __STM32L476R_NUCLEO_LCD_H
#define __STM32L476R_NUCLEO_LCD_H

void delay_ms(unsigned int ms);
int keypad_scan(void);
void buzz(void);
void delay_us(unsigned int ms);
void motor_init();
void DRIVE(int feet);
void LEFT(int feet);
void RIGHT(int feet);
void BACKWARDS(int feet);
void turnLeft(void);
void turnRight(void);
void turn_180(void);
void user_input(void);
void keypad_init(void);
void ultraSonic_init(void);
float sonicDistance(void);
void TIM2_Init(void);

#endif