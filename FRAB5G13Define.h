#ifndef FRAB5G13DEFINE_H
#define	FRAB5G13DEFINE_H

// Global Interrupts Enable/Disable Configuration Zone
#define GLOBAL_INT_DISABLE __builtin_disable_interrupts()       // Disable Global Interrupts
#define GLOBAL_INT_ENABLE __builtin_enable_interrupts()         // Enable Global Interrupts

// INT0 Configuration Zone
#define INT0_ENABLE IEC0bits.INT0IE = 1
#define INT0_DISABLE IEC0bits.INT0IE = 0
#define INT0_TRIG_EDGE(x) INTCON2bits.INT0EP = x                // Set Positive/Negative Edge INT0
#define INT0_PRIORITY(x) IPC0bits.INT0IP = x                    // Set Priority of INT0


// INT1,INT2 Configuration Zone
#define INT1_ENABLE IEC1bits.INT1IE = 1
#define INT1_DISABLE IEC1bits.INT1IE = 0
#define INT1_TRIG_EDGE(x) INTCON2bits.INT1EP = x                // Set Positive/Negative Edge INT1
#define INT1_PRIORITY(x) IPC5bits.INT1IP = x                    // Set Priority of INT1
#define INT2_ENABLE IEC1bits.INT2IE = 1
#define INT2_DISABLE IEC1bits.INT2IE = 0
#define INT2_TRIG_EDGE(x) INTCON2bits.INT2EP = x                // Set Positive/Negative Edge INT2
#define INT2_PRIORITY(x) IPC7bits.INT2IP = x                    // Set Priority of INT2

// Timer1 Configuration Zone
#define TIMER1_PRESCALE(x) T1CONbits.TCKPS = x                  // Set Timer1 Prescaler
#define TIMER1_PERIOD(x) PR1 = x                                // Set Timer1 Period
#define TIMER1INT_ENABLE _T1IE = 1                          // Enable Interrupt Timer1
#define TIMER1INT_DISABLE _T1IE = 0                          // Enable Interrupt Timer1
#define TIMER1_INT_PRIORITY(x) _T1IP = x                        // Set Priority of Interrupt Timer1
#define TIMER1_ON T1CONbits.TON = 1                         // Enable Timer1
#define TIMER1_OFF T1CONbits.TON = 0                          // Enable Timer1

// Timer23 Configuration Zone
#define TIMER2_PRESCALE(x) T2CONbits.TCKPS = x                  // Set Timer2 Prescaler
#define TIMER3_PRESCALE(x) T3CONbits.TCKPS = x                  // Set Timer2 Prescaler
#define TIMER23_COMBINE(x) T2CONbits.T32 = x                    // Enable Combine Timer23
#define TIMER3HB_PERIOD(x) PR3 = x                              // Set Timer3 Period
#define TIMER2LB_PERIOD(x) PR2 = x                              // Set Timer2 Period
#define TIMER3INT_ENABLE _T3IE = 1                          // Enable Interrupt Timer1
#define TIMER3INT_DISABLE _T3IE = 0                          // Enable Interrupt Timer1
#define TIMER3_INT_PRIORITY(x) _T3IP = x                        // Set Priority of Interrupt Timer3
#define TIMER2_ON T2CONbits.TON = 1                         // Enable Timer1
#define TIMER2_OFF T2CONbits.TON = 0                          // Enable Timer1
#define TIMER3_ON T3CONbits.TON = 1                         // Enable Timer1
#define TIMER3_OFF T3CONbits.TON = 0                          // Enable Timer1

// Timer45 Configuration Zone
#define TIMER4_PRESCALE(x) T4CONbits.TCKPS = x                  // Set Timer4 Prescaler
#define TIMER5_PRESCALE(x) T5CONbits.TCKPS = x                  // Set Timer2 Prescaler
#define TIMER45_COMBINE(x) T4CONbits.T32 = x                    // Enable Combine Timer45
#define TIMER5HB_PERIOD(x) PR5 = x                              // Set Timer5 Period
#define TIMER4LB_PERIOD(x) PR4 = x                              // Set Timer4 Period
#define TIMER5INT_ENABLE _T5IE = 1                          // Enable Interrupt Timer1
#define TIMER5INT_DISABLE _T5IE = 0                          // Enable Interrupt Timer1
#define TIMER5_INT_PRIORITY(x) _T5IP = x                        // Set Priority of Interrupt Timer5
#define TIMER4_ON T4CONbits.TON = 1                         // Enable Timer1
#define TIMER4_OFF T4CONbits.TON = 0                          // Enable Timer1
#define TIMER5_ON T5CONbits.TON = 1                         // Enable Timer1
#define TIMER5_OFF T5CONbits.TON = 0                          // Enable Timer1

// Frequency Configuration Zone
#define FCY 40000000UL                                          // Cycle Frequency in Hz
#define MOTOR_PWM_FREQ 50                                      // Motor PWM Frequency in Hz

// Motor Configuration Zone
#define M1 0x01
#define M2 0x02
#define SERVO1 0x01
#define SERVO2 0x02
#define ALL2 0x66
#define ALL3 0x67
#define FORWARD 0x00
#define BACKWARD 0x01

#define PWM1_ENABLE P1TCONbits.PTEN = 1
#define PWM1_DISABLE P1TCONbits.PTEN = 0
#define PWM2_ENABLE P2TCONbits.PTEN = 1
#define PWM2_DISABLE P2TCONbits.PTEN = 0

#define POSITIVE_EDGE 0
#define NEGATIVE_EDGE 1

// LED8 Configuration Zone
#define SHIFT_LED_MODE 0x01
#define NORMAL_MODE 0x02

#define MARGIN 13

#define BOOL unsigned char
#define TRUE 1
#define FALSE 0

double PID_Kp, PID_Ki, PID_Kd;
double PID_Integrated;
double PID_Prev_Input;
double PID_MinOutput, PID_MaxOutput;
BOOL PID_First_Time;
BOOL PID_is_working;

char DIR = 0;
unsigned int set_position = 0;

#define PPSUnlock __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock __builtin_write_OSCCONL(OSCCON | 0x40)

volatile uint64_t t = 0;

#endif	/* XC_HEADER_TEMPLATE_H */

