/*
 * Program      :   Test PWM Motor Control
 * Description  :   Generate PWM signal drive DC motor by IC L298N
 * Frequency    :   7.378 MHz at PLL
 * Filename     :   TestMCPWM.c
 * C Compiler   :   XC16 Complier by Microship Technology
 * Author       :   Mr.Krittamet Thawong and Mr.Wisarut Kasemphumirat KMUTT FIBO FRAB#5
 * Update       :   30 September 2020 11:27 PM UTC+7 Bangkok
 */
#include "xc.h"
#include <stdio.h>
#include "configuration.h"

// Global Interrupts Enable/Disable Configuration Zone
#define GLOBAL_INT_DISABLE __builtin_disable_interrupts()       // Disable Global Interrupts
#define GLOBAL_INT_ENABLE __builtin_enable_interrupts()         // Enable Global Interrupts

// INT0 Configuration Zone
#define INT0_ENABLE(x) IEC0bits.INT0IE = x
#define INT0_TRIG_EDGE(x) INTCON2bits.INT0EP = x

// INT1,INT2 Configuration Zone
#define INT1_ENABLE(x) IEC1bits.INT1IE = x
#define INT1_TRIG_EDGE(x) INTCON2bits.INT1EP = x
#define INT2_ENABLE(x) IEC1bits.INT2IE = x
#define INT2_TRIG_EDGE(x) INTCON2bits.INT2EP = x

// Timer1 Configuration Zone
#define TIMER1_PRESCALE(x) T1CONbits.TCKPS = x                  // Set Timer1 Prescaler
#define TIMER1_PERIOD(x) PR1 = x                                // Set Timer1 Period
#define TIMER1_INT_ENABLE(x) _T1IE = x                          // Enable Interrupt Timer1
#define TIMER1_INT_PRIORITY(x) _T1IP = x                        // Set Priority of Interrupt Timer1
#define TIMER1_ON(x) T1CONbits.TON = x                          // Enable Timer1

// Timer23 Configuration Zone
#define TIMER2_PRESCALE(x) T1CONbits.TCKPS = x                  // Set Timer2 Prescaler
#define TIMER23_COMBINE(x) T2CONbits.T32 = x                    // Enable Combine Timer23
#define TIMER3HB_PERIOD(x) PR3 = x                              // Set Timer3 Period
#define TIMER2LB_PERIOD(x) PR2 = x                              // Set Timer2 Period
#define TIMER3_INT_ENABLE(x) _T3IE = x                          // Enable Interrupt Timer1
#define TIMER3_INT_PRIORITY(x) _T3IP = x                        // Set Priority of Interrupt Timer1
#define TIMER2_ON(x) T2CONbits.TON = x                          // Enable Timer2

// Timer45 Configuration Zone
#define TIMER4_PRESCALE(x) T1CONbits.TCKPS = x                  // Set Timer2 Prescaler
#define TIMER45_COMBINE(x) T2CONbits.T32 = x                    // Enable Combine Timer23
#define TIMER5HB_PERIOD(x) PR3 = x                              // Set Timer3 Period
#define TIMER4LB_PERIOD(x) PR2 = x                              // Set Timer2 Period
#define TIMER5_INT_ENABLE(x) _T1IE = x                          // Enable Interrupt Timer1
#define TIMER5_INT_PRIORITY(x) _T1IP = x                        // Set Priority of Interrupt Timer1
#define TIMER3_ON(x) T1CONbits.TON = x                          // Enable Timer2

// Frequency Configuration Zone
#define FCY 40000000UL
#define MOTOR_PWM_FREQ 500        //motor PWM frequency in Hz
#define MOTOR_DUTY 70           //motor duty in %

char __dir;
char __STOPM1status = 0;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){
    __dir ^= 0x01;
    _T1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    _T3IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void){
    _T5IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void){
    __STOPM1status = 1;
    _INT0IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void){
    __STOPM1status = 0;
    _INT1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void){
    _INT2IF = 0;
}

void __setIO(int __byte1,int __byte2,int __byte3){
    AD1PCFGL = __byte1;
    TRISA = __byte2;
    TRISB = __byte3;
}

void __ExINT12_MAPPING(char __selectINT, char __selectRP)
{
    OSCCON = 0x46;
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 0;
    if(__selectINT == 0x01)
    {
        RPINR0bits.INT1R = __selectRP;
    }
    else if(__selectINT == 0x02)
    {
        RPINR1bits.INT2R = __selectRP;
    }
    OSCCON = 0x46;
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 1;
}
void initPWM()
{
     //setup PWM ports
    
     PWM1CON1 = 0;                  //  Clear all bits (use defaults)
     
     PWM1CON1bits.PMOD1 = 1;        //  PWM1L1,PWM1H1 are in independent running mode
     PWM1CON1bits.PMOD2 = 1;        //  PWM1L2,PWM1H2 are in independent running mode
     PWM1CON1bits.PMOD3 = 1;        //  PWM1L3,PWM1H3 are in independent running mode
     
     PWM1CON1bits.PEN1L = 0;        //  PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN1H = 1;        //  PWM1H1 PWM OUTPUT
     PWM1CON1bits.PEN2L = 0;        //  PWM1L2 NORMAL I/O
     PWM1CON1bits.PEN2H = 1;        //  PWM1H2 PWM OUTPUT
     PWM1CON1bits.PEN3L = 0;        //  PWM1L3 NORMAL I/O
     PWM1CON1bits.PEN3H = 1;        //  PWM1H3 PWM OUTPUT
     
 
     //PWM mode and prescaler
     //PWM1, MOTORS 0,1,2    
     P1TCON = 0;                    //  Clear all bits (use defaults)
     P1TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P1TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
 
     //Setup desired frequency by setting period for 1:64 prescaler
     P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;    

     //ENABLE PWM1
     P1TMR = 0;
     P1TCONbits.PTEN = 1; 
}

void initPLL() // Set Fcy to 40 MHz
{
    PLLFBD = 150;           // M  = 152
    CLKDIVbits.PLLPRE = 5;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used
    
    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01);    // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);    // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!=1) {};    // Wait for PLL to lock
}
void initADC()
{
     /* ADC Reading */
    AD1CON1bits.AD12B = 0;
    AD1CON3bits.ADCS = 2;
    AD1CON1bits.ADON = 1;
    int i;
    for(i = 0; i < 800; i++); Nop(); // wait ADC module stabilize 20us
}
int adc_value(int channel){
    AD1CHS0 = channel;
    AD1CON1bits.SAMP = 1; // Start sampling
    int i;
    for(i = 0; i < 15; i++); // Wait 5 * TAD
    AD1CON1bits.SAMP = 0; // Start converting
    while(!AD1CON1bits.DONE); // Conversion done?
    AD1CON1bits.DONE = 0; // Clear conversion done status bit
    return ADC1BUF0;
}
void run_motor(char ch, char dir, char pow)
{
    if(ch == 0x01)
    {
        if(dir == 0x01){
            LATBbits.LATB0 = 1;
            LATBbits.LATB1 = 0;
        }
        else if(dir == 0x02){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
        }
            //update duty cycle 
    P1DC1 = (2UL*P1TPER+2)*pow/100;  
    }
    else if(ch == 0x02)
    {
        if(dir == 0x01){
            LATBbits.LATB2 = 1;
            LATBbits.LATB3 = 0;
        }
        else if(dir == 0x02){
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 1;
        }
    P1DC2 = (2UL*P1TPER+2)*pow/100;
    }
    else if(ch == 0x03)
    {
        if(dir == 0x01){
            LATBbits.LATB0 = 1;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 1;
            LATBbits.LATB3 = 0;
        }
        else if(dir == 0x02){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 1;
        }
     P1DC1 = (2UL*P1TPER+2)*pow/100;
     P1DC2 = (2UL*P1TPER+2)*pow/100;
    }

}
void stop_motor(char channel){
    switch(channel)
    {
        case 1:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            break;
        }
        case 2:
        {
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        }
        case 12:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        }
        default:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        } 
    }
}
int main(void) {
    
    /* Set Port */
    __setIO(0xFFFF, 0x0000, 0x0090); // Argument 1 Set all ports are Digital I/O, 
    // Argument 2 Set all ports A are output, and Argument 3 Set all ports B are output except RB4 and RB7 are input.
    
    /* Disable Global Interrupt*/
    GLOBAL_INT_DISABLE;
    initPLL();
    initPWM();
    initADC();
    
    __ExINT12_MAPPING(0x01,0x04);
    
    INT0_ENABLE(1);
    INT1_ENABLE(1);
    INT0_TRIG_EDGE(0);
    INT1_TRIG_EDGE(0);
    
    /* Timer1 Configuration */
    TIMER1_PRESCALE(0b11);
    TIMER1_PERIOD(15625);
    TIMER1_INT_ENABLE(1);
    TIMER1_INT_PRIORITY(3);
    TIMER1_ON(1);
    
    /*enable global interrupt*/
    GLOBAL_INT_ENABLE;
    
    double ADC0;
    char __motor_duty;
    while(1)
    {
        ADC0 = adc_value(0);
        __motor_duty = (ADC0/1023.0)*100;
        if(__STOPM1status)
        {
            stop_motor(0x01);
            printf("Motor1 has been stopped\n");
        }
        else
        {
            run_motor(0x01, __dir, __motor_duty);
            printf("M1DIR: %1d, ADC0: %4.2f, SPD: %3d\n", __dir, ADC0, __motor_duty);
        }
    }
    
    return 0;
}