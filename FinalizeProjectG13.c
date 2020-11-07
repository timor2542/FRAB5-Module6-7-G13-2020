/*
 * Program      :   Module 6-7 Milestone 3
 * Description  :   Generate PWM signal drive DC motor by Cytron MDD10A Rev. 2.0
 * Frequency    :   8 MHz at PLL
 * Filename     :   FinalizeProjectG13.c
 * C Compiler   :   XC16 Complier by Microship Technology
 * Author       :   Mr.Krittamet Thawong and Mr.Wisaruth Kasemphumirat KMUTT FIBO FRAB#5
 * Update       :   7 November 2020 10:08 AM UTC+7 Bangkok
 * Test Port
 */
#include "math.h" 
#include "xc.h"                                                 // Include XC16 Complier
#include <stdio.h>                                              // Standard I/O Library
#include "configuration.h"                                      // Bit Configuration
#include "UART_rev3.h"
#include "FRAB5G13Define.h"
long MIN(long x,long y)
{
    return (x < y) ? x : y;
}
long MAX(long x,long y)
{
    return (x > y) ? x : y;
}
long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double mapf(double val, double in_min, double in_max, double out_min, double out_max) 
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Reset_PID()
{
    PID_Integrated = 0.0;
    PID_Prev_Input = 0.0;
    PID_First_Time = TRUE;
}
void Init_PID(double Kp, double Ki, double Kd, double MinOutput, double MaxOutput)
{
    PID_Kp         = Kp;
    PID_Ki         = Ki;
    PID_Kd         = Kd;
    PID_MinOutput  = MinOutput;
    PID_MaxOutput  = MaxOutput;
    PID_Integrated = 0.0;
    PID_Prev_Input = 0.0;
    PID_First_Time = TRUE;
}
double PID_Calculate(double SetPoint, double InputValue)
{
    double Err, ErrValue, DiffValue, Result;
//    char dir = 0;

    Err = SetPoint - InputValue;
    
    ErrValue  = Err * PID_Kp;

    // --- Calculate integrated value ---
    PID_Integrated = PID_Integrated + (Err * PID_Ki);
    // limit it to output minimum and maximum
    if (PID_Integrated < PID_MinOutput) 
      PID_Integrated = PID_MinOutput;
    if (PID_Integrated > PID_MaxOutput)
      PID_Integrated = PID_MaxOutput;

    // --- calculate derivative value ---
    if (PID_First_Time)
    {
      // to avoid a huge DiffValue the first time (PID_Prev_Input = 0)
      PID_First_Time = FALSE;
      PID_Prev_Input = InputValue;
    }
    DiffValue = (InputValue - PID_Prev_Input) * PID_Kd;
    PID_Prev_Input = InputValue;

    // --- calculate total ---
    Result = ErrValue + PID_Integrated + DiffValue; // mind the minus sign!!!
    // limit it to output minimum and maximum
    if (Result < PID_MinOutput)
    {
        Result = PID_MinOutput;
    }
    else if (Result > PID_MaxOutput)
    {
        Result = PID_MaxOutput;
    }
    
    if(Result < 0)
    {
        DIR = 1;
    }
    else if (Result >= 0)
    {
        DIR = 0;
    }
    return (abs(Result));
}
void setADIO(unsigned int __byte1,unsigned int __byte2,unsigned int __byte3)// Set
{
    AD1PCFGL = __byte1;
    TRISA = __byte2;
    TRISB = __byte3;
}
void XINT(char __selectINT, char __selectRP) // External Mapping Function
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
     
     P2TCONbits.PTEN = 0;
     PWM2CON1 = 0; 
     
     PWM1CON1bits.PMOD1 = 1;                                //  PWM1L1,PWM1H1 are in independent running mode
     PWM1CON1bits.PMOD2 = 1;                                //  PWM1L2,PWM1H2 are in independent running mode
     PWM1CON1bits.PMOD3 = 1;                                //  PWM1L3,PWM1H3 are in independent running mode
     PWM2CON1bits.PMOD1 = 1;                                //  PWM2L1,PWM2H1 are in independent running mode
     
     PWM1CON1bits.PEN1L = 0;                                //  PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN1H = 1;                                //  PWM1H1 PWM OUTPUT
     PWM1CON1bits.PEN2L = 0;                                //  PWM1L2 NORMAL I/O
     PWM1CON1bits.PEN2H = 1;                                //  PWM1H2 PWM OUTPUT
     PWM1CON1bits.PEN3L = 0;                                //  PWM1L3 NORMAL I/O
     PWM1CON1bits.PEN3H = 0;                                //  PWM1H3 NORMAL I/O
     PWM2CON1bits.PEN1L = 0;                                //  PWM2L1 NORMAL I/O
     PWM2CON1bits.PEN1H = 1;                                //  PWM2H1 PWM OUTPUT

     //PWM1 Mode and Pre-scaler
     //PWM1H,PWM2H ON DC MOTOR 2-Channel and 1-Channel    
     P1TCON = 0;                    //  Clear all bits (use defaults)
     P1TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P1TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
     P2TCON = 0;                    //  Clear all bits (use defaults)
     P2TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P2TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
 
     //Setup desired frequency by setting period for 1:64 prescaler
     P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;
     P2TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;

     //ENABLE PWM1
     P1TMR = 0;
     P2TMR = 0; 
}
void PWM1Freq(unsigned int __MOTOR_PWM_FREQ)
{
    P1TCONbits.PTEN = 0;
    P1TPER = (FCY  / 64 / __MOTOR_PWM_FREQ) - 1;
    P1TMR = 0;
    //P1TCONbits.PTEN = 1;
}
void PWM2Freq(unsigned int __MOTOR_PWM_FREQ)
{
    P2TCONbits.PTEN = 0;
    P2TPER = (FCY  / 64 / __MOTOR_PWM_FREQ) - 1;
    P2TMR = 0;
    //P2TCONbits.PTEN = 1;
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
int adc_value(int channel)
{
    AD1CHS0 = channel;
    AD1CON1bits.SAMP = 1; // Start sampling
    int i;
    for(i = 0; i < 15; i++); // Wait 5 * TAD
    AD1CON1bits.SAMP = 0; // Start converting
    while(!AD1CON1bits.DONE); // Conversion done?
    AD1CON1bits.DONE = 0; // Clear conversion done status bit
    return ADC1BUF0;
}
void initQEI()
{
    __builtin_write_OSCCONL(OSCCON & 0xBF); // PPS RECONFIG UNLOCK 
    
    RPINR18bits.U1RXR = 6;                  // Set UART1 RX to MCU pin
    RPOR2bits.RP5R = 0b00011;               // Set UART1 TX from MCU pin
    RPINR14bits.QEA1R = 2;          // Remap __chQEI1A connect to QEI1_A
    RPINR14bits.QEB1R = 3;          // Remap __chQEI1B connect to QEI1_B
    RPINR16bits.QEA2R = 0;          // Remap __chQEI2A connect to QEI2_A
    RPINR16bits.QEB2R = 1;          // Remap __chQEI2B connect to QEI2_B
    
    __builtin_write_OSCCONL(OSCCON | 0x40); //  PPS RECONFIG LOCK 
    
    /*QEI1 Mode Select*/
    QEI1CONbits.QEIM = 0b000;           // QEI1 Mode disable
    QEI1CONbits.PCDOUT = 0;             // no direction pin out
    QEI1CONbits.QEIM = 0b111;           // 4x ,no index
    
    /*QEI2 Mode Select*/
    QEI2CONbits.QEIM = 0b000;           // QEI2 Mode disable
    QEI2CONbits.PCDOUT = 0;             // no direction pin out
    QEI2CONbits.QEIM = 0b111;           // 4x ,no index
    
    /*Digital Filter QEI1 Config */
    DFLT1CONbits.QECK = 0b111;          // clock divider Fcy/256
    DFLT1CONbits.QEOUT = 1;             // enable filter
    
    /*Digital Filter QEI2 Config */
    DFLT2CONbits.QECK = 0b111;          // clock divider Fcy/256
    DFLT2CONbits.QEOUT = 1;             // enable filter
    
}
void PWM1(char ch, char dir, char POW)
{
    if(ch == 0x01)
    {
        if(dir == 0x00)
        {
            _LATA2 = 0;
            _LATA3 = 0;
        }
        else if(dir == 0x01)
        {
            _LATA2 = 1;
            _LATA3 = 0;
        }
        P1DC1 = (2UL+2*P1TPER)*POW/100;
        P1DC2 = 0;
        P1DC3 = 0;
    }
    else if(ch == 0x02)
    {
        if(dir == 0x00)
        {
            _LATA2 = 0;
            _LATA3 = 0;
        }
        else if(dir == 0x01)
        {
            _LATA2 = 0;
            _LATA3 = 1;
        }
        P1DC1 = 0;
        P1DC2 = (2UL+2*P1TPER)*POW/100;
        P1DC3 = 0;
    }
    else if(ch == 0x66)
    {
        if(dir == 0x00)
        {
            _LATA2 = 0;
            _LATA3 = 0;

        }
        else if(dir == 0x01)
        {
            _LATA2 = 1;
            _LATA3 = 1;
        }
        P1DC1 = (2UL+2*P1TPER)*POW/100;
        P1DC2 = (2UL+2*P1TPER)*POW/100;
        P1DC3 = 0;
    }
}
void PWM2(char dir, char POW)
{
    if(dir == 0x00)
    {
        _LATB13 = 0;
    }
    else if(dir == 0x01)
    {
        _LATB13 = 1;
    }
    P2DC1 = (2UL+2*P2TPER)*POW/100;
}
void PWM1Stop(char ch)
{
    if(ch == 0x01)
    {
        _LATA2 = 0;
        P1DC1 = 0;
    }
    else if(ch == 0x02)
    {
        _LATA3 = 0;
        P1DC2 = 0;
    }
    else if(ch == 0x66){
        _LATA2 = 0;
        _LATA3 = 0;
        P1DC1 = 0;
        P1DC2 = 0;
    }
}
void PWM2Stop()
{
    P2DC1 = 0;
}
void initTimer1() //Timer1
{
    TIMER1_OFF;
    TIMER1_PRESCALE(0b10); //set timer prescaler to 1:64
    PR1 = 6250;
}
void initOCServo() // Timer23
{
    TIMER2_OFF;
    TIMER3_OFF;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    TIMER2_PRESCALE(0b10); //set timer prescaler to 1:64
    TIMER3_PRESCALE(0b10); //set timer prescaler to 1:64
    
    PR2 = 12500;            //set period to 12,500 tick per cycle (Frequency of Servo = 50 Hz)
    OC1CONbits.OCTSEL = 0;  //OC1 use timer2 as counter source
    
    PR3 = 12500;            //set period to 12,500 tick per cycle (Frequency of Servo = 50 Hz)
    OC2CONbits.OCTSEL = 1;  //OC2 use timer3 as counter source

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP10R = 0b10010;                 //remap RP11 connect to OC1
    _RP9R = 0b10011;                 //remap RP11 connect to OC2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
    
}
void servo_angle(char ch, signed int angle) //set velocity
{
    if(ch == 0x01)
    {
         //set to pwm without fault pin mode
        if(angle == -1)
        {
            OC1RS = 0;
        }
        else if(angle >= 0 && angle <= 180)
        {
            long value = map(angle,0,180,315,1560); // Angle 0-180 = OCxRS 315-1560
            OC1RS = value;
        }
    }
    else if(ch == 0x02)
    {
        //OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
        if(angle == -1)
        {
            OC2RS = 0;
        }
        else if(angle >= 0 && angle <= 180)
        {
            long value = map(angle,0,180,315,1560); // Angle 0-180 = OCxRS 315-1560
            OC2RS = value;
        }
    }
    else if(ch == 0x66)
    {
        if(angle == -1)
        {
            OC1RS = 0;
            OC2RS = 0;
        }
        else if(angle > 0 && angle <= 180)
        {
            long value = map(angle,0,180,315,1560); // Angle 0-180 = OCxRS 315-1560
            OC1RS = value;
            OC2RS = value;
        }
    }
}
void servo_stop(char ch)
{
    switch(ch)
    {
        case 0x01:
        {
            OC1RS = 0;
        }
        case 0x02:
        {
            OC2RS = 0;
        }
        case 0x66:
        {
            OC1RS = 0;
            OC2RS = 0;
        }
    }
}
void updatePositionX(long _posx)
{
    set_position_x = _posx;
}
void updatePositionY(long _posy)
{
    set_position_y = _posy;
}
void updatePositionZ(long _posz)
{
    set_position_z = _posz;
}
void updatePositionXYZ(long _posx, long _posy, long _posz)
{
    set_position_x = _posx;
    set_position_y = _posy;
    set_position_z = _posz;
}
void ResetPulse()
{
    POS1CNT = 0;
    POS2CNT = 0;
}
void initALL()
{
        // Set Port
    // Argument 1 Set all ports A/D I/O;    Analog = 0, Digital = 1
    // Argument 2 Set all ports A I/O       Output = 0, Input = 1
    // Argument 3 Set all ports B I/O       Output = 0, Input = 1
    setADIO(0xFFFC, 0x0003, 0x004F);
    
    // Disable Global Interrupt
    GLOBAL_INT_DISABLE;
    
    // Initialize
    initPLL();  // Initialize PLL at FCY = 40 MHz
    initTimer1();
    initPWM();  // Initialize PWM
    initADC();  // Initialize ADC
    
    // External Interrupt Configuration
    XINT(1,4);     // Remap External INT1 on RP4
    XINT(2,10);     // Remap External INT2 on RP10
    
    initQEI(); // Initialize QEI1 and QEI2
    initOCServo();

    // Enable External Interrupt
    INT0_TRIG_EDGE(POSITIVE_EDGE);              // Set INT0 Positive Edge
    INT1_TRIG_EDGE(POSITIVE_EDGE);              // Set INT1 Positive Edge
    INT2_TRIG_EDGE(POSITIVE_EDGE);              // Set INT2 Positive Edge
    INT0_PRIORITY(5);               // Set INT0 Interrupt Priority by 7
    INT1_PRIORITY(4);               // Set INT1 Interrupt Priority by 6
    INT2_PRIORITY(3);               // Set INT2 Interrupt Priority by 5
    INT0_ENABLE;                 // Enable INT0
    INT1_ENABLE;                 // Enable INT1
    INT2_ENABLE;                 // Enable INT2
    
    // UART1 Initialize Communication at Baud Rate 115200 bps
    UART1_Initialize(86, 347);
    _U1RXIP = 2;
    // Update Frequency of PWM1 and PWM2
    PWM1Freq(500);
    PWM1_ENABLE;
    PWM2Freq(10000);
    PWM2_ENABLE;
    
    TIMER2_ON;
    TIMER3_ON;
    // Enable Global Interrupt
    TIMER1INT_ENABLE;
    TIMER1_INT_PRIORITY(1);
    
    _LATB15 = 0;
    ResetPulse();
    updatePositionXYZ(0,0,0);
    Init_PID(1,0,0,-30,30); // Ki, Kp, Kd, MinDCPWM, MaxDCPWM
    
    GLOBAL_INT_ENABLE;
}
/*long step_pluse2mm()
{
    return 0;
}*/
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    if(Running == TRUE)
    {
        if(abs(set_position_x - POS1CNT) < MARGIN) //&& abs(set_position_y - POS2CNT) < MARGIN && abs(set_position_z - ???) < MARGIN)
        {
            Running = FALSE;
            PWM1Stop(ALL2);
            PWM2Stop();
        }
        else
        {
            Running = TRUE;
            PWM1(M1, !DIR, (int)PID_Calculate(set_position_x,POS1CNT));
            //PWM1(M2, !DIR, (int)PID_Calculate(set_position_y,POS2CNT));
            //PWM2(!DIR, (int)PID_Calculate(set_position_z,???));
        }
        AnPos = (2.0 * PI * POS1CNT) / PPR;
        Distance = (2.0 * PI * SHAFT_MOTOR_RADIUS) * (AnPos / 360.0);
        printf("AnPos = %.2f, Distance = %.2f\n",AnPos,Distance);
    }  
    else
    {
        PWM1Stop(ALL2);
    }
    _T1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{
    
    _T3IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void)
{
    
    _T5IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{

    _INT0IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
{

    _INT1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
{
    
    _INT2IF = 0;
}
int main(void) 
{
    initALL();
    while(1)
    {
        numByte = UART1_ReadBuffer(dataArray, 1);
        if (numByte != 0) {
            Running == TRUE;
         }
    }
    return 0;
}