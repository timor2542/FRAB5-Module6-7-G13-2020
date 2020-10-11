/*
 * Program      :   Test PWM Motor Control
 * Description  :   Generate PWM signal drive DC motor by IC L298N
 * Frequency    :   7.378 MHz at PLL
 * Filename     :   TestMCPWM.c
 * C Compiler   :   XC16 Complier by Microship Technology
 * Author       :   Mr.Krittamet Thawong and Mr.Wisaruth Kasemphumirat KMUTT FIBO FRAB#5
 * Update       :   30 September 2020 11:27 PM UTC+7 Bangkok
 * Test Port
 */
#include "xc.h"                                                 // Include XC16 Complier
#include <stdio.h>                                              // Standard I/O Library
#include "configuration.h"                                      // Bit Configuration

// Global Interrupts Enable/Disable Configuration Zone
#define GLOBAL_INT_DISABLE __builtin_disable_interrupts()       // Disable Global Interrupts
#define GLOBAL_INT_ENABLE __builtin_enable_interrupts()         // Enable Global Interrupts

// INT0 Configuration Zone
#define INT0_ENABLE(x) IEC0bits.INT0IE = x                      // Enable INT0
#define INT0_TRIG_EDGE(x) INTCON2bits.INT0EP = x                // Set Positive/Negative Edge INT0

// INT1,INT2 Configuration Zone
#define INT1_ENABLE(x) IEC1bits.INT1IE = x                      // Enable INT1
#define INT1_TRIG_EDGE(x) INTCON2bits.INT1EP = x                // Set Positive/Negative Edge INT1
#define INT2_ENABLE(x) IEC1bits.INT2IE = x                      // Enable INT2
#define INT2_TRIG_EDGE(x) INTCON2bits.INT2EP = x                // Set Positive/Negative Edge INT2

// Timer1 Configuration Zone
#define TIMER1_PRESCALE(x) T1CONbits.TCKPS = x                  // Set Timer1 Prescaler
#define TIMER1_PERIOD(x) PR1 = x                                // Set Timer1 Period
#define TIMER1_INT_ENABLE(x) _T1IE = x                          // Enable Interrupt Timer1
#define TIMER1_INT_PRIORITY(x) _T1IP = x                        // Set Priority of Interrupt Timer1
#define TIMER1_ON(x) T1CONbits.TON = x                          // Enable Timer1

// Timer23 Configuration Zone
#define TIMER2_PRESCALE(x) T2CONbits.TCKPS = x                  // Set Timer2 Prescaler
#define TIMER23_COMBINE(x) T2CONbits.T32 = x                    // Enable Combine Timer23
#define TIMER3HB_PERIOD(x) PR3 = x                              // Set Timer3 Period
#define TIMER2LB_PERIOD(x) PR2 = x                              // Set Timer2 Period
#define TIMER3_INT_ENABLE(x) _T3IE = x                          // Enable Interrupt Timer3
#define TIMER3_INT_PRIORITY(x) _T3IP = x                        // Set Priority of Interrupt Timer3
#define TIMER2_ON(x) T2CONbits.TON = x                          // Enable Timer2

// Timer45 Configuration Zone
#define TIMER4_PRESCALE(x) T4CONbits.TCKPS = x                  // Set Timer4 Prescaler
#define TIMER45_COMBINE(x) T4CONbits.T32 = x                    // Enable Combine Timer45
#define TIMER5HB_PERIOD(x) PR5 = x                              // Set Timer5 Period
#define TIMER4LB_PERIOD(x) PR4 = x                              // Set Timer4 Period
#define TIMER5_INT_ENABLE(x) _T5IE = x                          // Enable Interrupt Timer5
#define TIMER5_INT_PRIORITY(x) _T5IP = x                        // Set Priority of Interrupt Timer5
#define TIMER4_ON(x) T4CONbits.TON = x                          // Enable Timer4

// Frequency Configuration Zone
#define FCY 40000000UL                                          // Cycle Frequency in Hz
#define MOTOR_PWM_FREQ 500                                      // Motor PWM Frequency in Hz
#define MOTOR_DUTY 70                                           // Motor Duty in %

#define ALL2 0x66
#define ALL3 0x67
#define FORWARD 0x01
#define BACKWARD 0x02

char __dir = 0x01, __RUNMotor = 0;                                 //
char millis = 0;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){
    
    if(millis == 50){
     __dir ^= 0x03;
     millis = 0;
    }
    else{
        millis++;
    }
    _T1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    _T3IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void){
    _T5IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void){
    __RUNMotor = 1;
    _INT0IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void){
    __RUNMotor = 0;
    _INT1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void){
    _INT2IF = 0;
}

void __setIO(unsigned int __byte1,unsigned int __byte2,unsigned int __byte3){
    AD1PCFGL = __byte1;
    TRISA = __byte2;
    TRISB = __byte3;
}

void __ExINT_MAPPING(char __selectINT, char __selectRP)
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
     P1TCONbits.PTEN = 0;
     PWM1CON1 = 0;                                          //  Clear all bits (use defaults)
     
     PWM1CON1bits.PMOD1 = 1;                                //  PWM1L1,PWM1H1 are in independent running mode
     PWM1CON1bits.PMOD2 = 1;                                //  PWM1L2,PWM1H2 are in independent running mode
     PWM1CON1bits.PMOD3 = 1;                                //  PWM1L3,PWM1H3 are in independent running mode
     
     PWM1CON1bits.PEN1L = 0;                                //  PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN1H = 1;                                //  PWM1H1 PWM OUTPUT
     PWM1CON1bits.PEN2L = 0;                                //  PWM1L2 NORMAL I/O
     PWM1CON1bits.PEN2H = 1;                                //  PWM1H2 PWM OUTPUT
     PWM1CON1bits.PEN3L = 0;                                //  PWM1L3 NORMAL I/O
     PWM1CON1bits.PEN3H = 1;                                //  PWM1H3 PWM OUTPUT
     
 
     //PWM Mode and Pre-scaler
     //PWM1,PWM2,PWM3,MOTOR1,MOTOR2,and MOTOR3    
     P1TCON = 0;                    //  Clear all bits (use defaults)
     P1TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P1TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
 
     //Setup desired frequency by setting period for 1:64 prescaler
     P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;    

     //ENABLE PWM1
     P1TMR = 0; 
}
void __updateFPWM(unsigned int __MOTOR_PWM_FREQ)
{
    P1TCONbits.PTEN = 0;
    P1TPER = (FCY  / 64 / __MOTOR_PWM_FREQ) - 1;
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
    P1DC1 = ((2UL*P1TPER+2)*pow)/100;  // Update duty cycle 
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
    P1DC2 = ((2UL*P1TPER+2)*pow)/100;  // Update duty cycle 
    }
    else if(ch == 0x03)
    {
        LATBbits.LATB6 = 0;
        if(dir == 0x01){
            LATBbits.LATB4 = 1;
        }
        else if(dir == 0x02){
            LATBbits.LATB4 = 0;
        }
    P1DC3 = ((2UL*P1TPER+2)*pow)/100; // Update duty cycle 
    }
    else if(ch == 0x66)
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
     P1DC1 = ((2UL*P1TPER+2)*pow)/100; // Update duty cycle 
     P1DC2 = ((2UL*P1TPER+2)*pow)/100; // Update duty cycle
        
    }
    else if(ch == 0x67)
    {
        if(dir == 0x01){
            LATBbits.LATB0 = 1;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 1;
            LATBbits.LATB3 = 0;
            LATBbits.LATB4 = 1;
        }
        else if(dir == 0x02){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 1;
            LATBbits.LATB4 = 0;
        }
     P1DC1 = ((2UL*P1TPER+2)*pow)/100; // Update duty cycle 
     P1DC2 = ((2UL*P1TPER+2)*pow)/100; // Update duty cycle
     P1DC3 = ((2UL*P1TPER+2)*pow)/100; // Update duty cycle   
    }
}
void stop_motor(char channel){
    switch(channel)
    {
        case 0x01:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            break;
        }
        case 0x02:
        {
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        }
        case 0x03:
        {
            LATBbits.LATB6 = 1;
            P1DC3 = 0;
        }
        case 0x66:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        }
        case 0x67:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            P1DC3 = 0;
            break;
        }
        default:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            P1DC3 = 0;
            break;
        } 
    }
}
void LED8(char ch,unsigned int data)
{
    
}
int main(void) {
    
    // Set Port
    __setIO(0xFFFE, 0x0001, 0x0090); //0x0090); // Argument 1 Set all ports are Digital I/O except RA0 (AN0) is analog, 
    // Argument 2 Set all ports A are output except RA0 (AN0) is input, and Argument 3 Set all ports B are output except RB4 and RB7 are input.
    
    // Disable Global Interrupt
    GLOBAL_INT_DISABLE;
    initPLL();  // Initialize PLL at FCY = 40 MHz
    initPWM();  // Initialize PWM
    initADC();  // Initialize ADC
    
    // External Interrupt Configuration
    __ExINT_MAPPING(0x01,0x05);     // Remap External INT1 on RP5
    
    INT0_ENABLE(1);                 // Enable INT0
    INT1_ENABLE(1);                 // Enable INT1
    INT0_TRIG_EDGE(0);              // Set INT0 Positive Edge
    INT1_TRIG_EDGE(0);              // Set INT1 Positive Edge
    
    // Timer1 Configuration
    TIMER1_PRESCALE(0b11);          // Timer1 1:256 Prescaler
    TIMER1_PERIOD(15625);           // Set period every 0.1 milliseconds
    TIMER1_INT_ENABLE(1);           // Enable Timer1 Interrupt
    TIMER1_INT_PRIORITY(3);         // Set Timer1 Interrupt Priority by 3
    TIMER1_ON(1);                   // Enable Timer1 ON

    // Enable Global Interrupt
    GLOBAL_INT_ENABLE;
    
    __updateFPWM(100);              // Update Frequency PWM 300 Hz
    
    double ADC0,__motor_duty;
    while(1)
    {
        ADC0 = adc_value(0);                            // Reading ADC Channel 0
        __motor_duty = (ADC0/1023.0)*100;               // Calculate 10 Bits-ADC to Duty Cycle Conversion
        //__LED8Value = (ADC0/1023.0)*255;
        if(__RUNMotor)                              // If __STOPM1status = 1
        {
                                       // Stop 3 Motor
            run_motor(0x66, __dir, (int)__motor_duty);
            //printf("All Motor has been stopped\n");
        }
        else
        {
            stop_motor(ALL2);       // Run Motor1,2
            //printf("M1DIR: %1d, ADC0: %4.2f, SPD: %3d, TP1: %6d, TP1ON: %6d\n", __dir, ADC0, __motor_duty, P1TPER, P1DC1/2);
        }
    }
    
    return 0;
}