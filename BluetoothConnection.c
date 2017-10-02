#include <msp430.h>

#define MCU_CLOCK           1048576             // default SMC Frequency
#define PWM_FREQUENCY       46                  // hertz

#define SERVO_STEPS         180                 // max amount of steps (degrees)
#define SERVO_MIN           650                 // min duty cycle
#define SERVO_MAX           2700                // max duty cycle

#define RightDcMotor    TA0CCR3
#define LeftDcMotor     TA0CCR4
#define RightServoMotor TA2CCR1
#define LeftServoMotor  TA2CCR2

#define BUTTON BIT1
#define BUTTON_PRESSED (!(BUTTON & P1IN))

volatile int servo = 0;
volatile int DcMotor = 0;
volatile int Roll   = 0;


/*double throttle[101] = {0 , 0.01 , 0.02 , 0.03 , 0.04 , 0.05 ,
                        0.06 , 0.07 , 0.08 , 0.09 , 0.10 , 0.11 ,
                        0.12 , 0.13 , 0.14 , 0.15 , 0.16 , 0.17 ,
                        0.18 , 0.19 , 0.20 , 0.21 , 0.22 , 0.23 ,
                        0.24 , 0.25 , 0.26 , 0.27 , 0.28 , 0.29 ,
                        0.30 , 0.31 , 0.32 , 0.33 , 0.34 , 0.35 ,
                        0.36 , 0.37 , 0.38 , 0.39 , 0.40 , 0.41 ,
                        0.42 , 0.43 , 0.44 , 0.45 , 0.46 , 0.47 ,
                        0.48 , 0.49 , 0.50 , 0.51 , 0.52 , 0.53 ,
                        0.54 , 0.55 , 0.56 , 0.57 , 0.58 , 0.59 ,
                        0.60 , 0.61 , 0.62 , 0.63 , 0.64 , 0.65 ,
                        0.66 , 0.67 , 0.68 , 0.69 , 0.70 , 0.71 ,
                        0.72 , 0.73 , 0.74 , 0.75 , 0.76 , 0.77 ,
                        0.78 , 0.79 , 0.80 , 0.81 , 0.82 , 0.83 ,
                        0.84 , 0.85 , 0.86 , 0.87 , 0.88 , 0.89 ,
                        0.90 , 0.91 , 0.92 , 0.93 , 0.94 , 0.95 ,
                        0.96 , 0.97 , 0.98 , 0.99 , 1
};*/

volatile int throttle = 0;
volatile int Rollright = 0;
volatile int Rollleft = 0;

unsigned int PWM_Period = (MCU_CLOCK / PWM_FREQUENCY);  // PWM Period 20971.52
unsigned int PWM_Duty = 0;                              // %

#define TXLED BIT0
#define RXLED BIT7
#define TXD BIT3
#define RXD BIT4
#define RXBUFFER UCA0RXBUF
#define TXBUFFER UCA0TXBUF
#define MAX_BUFFER_LENGTH 256
#define END_BYTE 'X'

unsigned int servo_stepval, servo_stepnow;
unsigned int servo_LUT[SERVO_STEPS + 1];
unsigned int i;


void RollSetup(void)
{
    if (Roll == 0)
    {
        Rollright = 1;
        Rollleft = 0;
    }
    if (Roll == 1)
    {
        Rollright = 1;
        Rollleft = 1;
    }
    if (Roll == 2)
    {
        Rollright = 0;
        Rollleft = 1;
    }
}

void InitializeUART(void)
{
    P3SEL = TXD + RXD;                        // P3.3,4 = USCI_A1 TXD/RXD
    P1DIR |= TXLED;
    P4DIR |= RXLED;
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 6;                              // 1MHz 9600 (see User's Guide)
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
                                              // over sampling
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void InitializePWM(void)
{
    servo_stepval = ((SERVO_MAX - SERVO_MIN) / SERVO_STEPS);    //11.38
    servo_stepnow = SERVO_MIN;                  // 650

    for (i = 0; i < SERVO_STEPS; i++)
    {                                           //fill up look-up table values
        servo_stepnow += servo_stepval;
        servo_LUT[i] = servo_stepnow;
    }

    TA1CCTL0 = CCIE;                            // enable A1 interrupts
    TA1CCR0 = 5000;                             // debounce time
    TA1CTL = TASSEL_2 + MC_0;                   // initialize timerA1 off
    P1REN |= BUTTON;                            // pull-up resistor
    P1OUT |= BUTTON;                            // button as output
    P1IE |= BUTTON;                             // button interrupt flag
    P1IFG &= ~BUTTON;                           // clear button flag

    // dc motor timer
    TA0CCTL3 = OUTMOD_7;                        // RightDcMotor reset/set
    TA0CCTL4 = OUTMOD_7;                        // LeftDcMotor reset/set
    TA0CTL = TASSEL_2 + MC_1;                   // SMCLK, upmode
    TA0CCR0 = PWM_Period - 1;                   // PWM Period
    RightDcMotor = PWM_Duty;                    // TA0CCR3 PWM Duty Cycle
    LeftDcMotor = PWM_Duty;                     // TA0CCR4 PWM Duty Cycle

    P1DIR |= BIT4 + BIT5;                       // P1.4 and 1.5 = output
    P1SEL |= BIT4 + BIT5;                       // P1.4 and 1.5 = TA0 output

    // servo motor timer
    TA2CCTL1 = OUTMOD_7;                        // RightServoMotor reset/set
    TA2CCTL2 = OUTMOD_7;                        // LeftServoMotor reset/set
    TA2CTL = TASSEL_2 + MC_1;                   // SMCLK, upmode
    TA2CCR0 = PWM_Period - 1;                   // PWM Period
    RightServoMotor = PWM_Duty;                 // TA2CCR1 PWM Duty Cycle
    LeftServoMotor = PWM_Duty;                         // TA2CCR2 PWM Duty Cycle

    P2DIR |= BIT4 + BIT5;                       // P2.4 = output
    P2SEL |= BIT4 + BIT5;                       // P2.4 = TA2 output
}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                   // Stop WDT

    InitializeUART();
    InitializePWM();

    __enable_interrupt();

    while (1)                                   // MAIN LOOP
    {
        RollSetup();
        switch (servo)
        {
        case 0:
            RightServoMotor = servo_LUT[165];             //  degrees
            LeftServoMotor = servo_LUT[0];
            LPM0;
            break;
        case 1:
            RightServoMotor = servo_LUT[159];            // 90 degrees
            LeftServoMotor = servo_LUT[6];
            LPM0;
            break;
        case 2:
            RightServoMotor = servo_LUT[153];            // 90 degrees
            LeftServoMotor = servo_LUT[12];
            LPM0;
            break;
        case 3:
            RightServoMotor = servo_LUT[146];            // 90 degrees
            LeftServoMotor = servo_LUT[19];
            LPM0;
            break;
        case 4:
            RightServoMotor = servo_LUT[140];            // 90 degrees
            LeftServoMotor = servo_LUT[25];
            LPM0;
            break;
        case 5:
            RightServoMotor = servo_LUT[134];            // 90 degrees
            LeftServoMotor = servo_LUT[31];
            LPM0;
            break;
        case 6:
            RightServoMotor = servo_LUT[127];            // 90 degrees
            LeftServoMotor = servo_LUT[38];
            LPM0;
            break;
        case 7:
            RightServoMotor = servo_LUT[121];            // 90 degrees
            LeftServoMotor = servo_LUT[46];
            LPM0;
            break;
        case 8:
            RightServoMotor = servo_LUT[115];            // 90 degrees
            LeftServoMotor = servo_LUT[52];
            LPM0;
            break;
        case 9:
            RightServoMotor = servo_LUT[108];            // 90 degrees
            LeftServoMotor = servo_LUT[59];
            LPM0;
            break;
        case 10:
            RightServoMotor = servo_LUT[102];            // 90 degrees
            LeftServoMotor = servo_LUT[65];
            LPM0;
            break;
        case 11:
            RightServoMotor = servo_LUT[96];            // 90 degrees
            LeftServoMotor = servo_LUT[71];
            LPM0;
            break;
        case 12:
            RightServoMotor = servo_LUT[89];            // 90 degrees
            LeftServoMotor = servo_LUT[78];
            LPM0;
            break;
        case 13:
            RightServoMotor = servo_LUT[83];            // 90 degrees
            LeftServoMotor = servo_LUT[84];
            LPM0;
            break;
        case 14:
            RightServoMotor = servo_LUT[77];            // 90 degrees
            LeftServoMotor = servo_LUT[90];
            LPM0;
            break;
        case 15:
            RightServoMotor = servo_LUT[70];            // 90 degrees
            LeftServoMotor = servo_LUT[97];
            LPM0;
            break;
        case 16:
            RightServoMotor = servo_LUT[64];            // 90 degrees
            LeftServoMotor = servo_LUT[103];
            LPM0;
            break;
        case 17:
            RightServoMotor = servo_LUT[58];            // 90 degrees
            LeftServoMotor = servo_LUT[109];
            LPM0;
            break;
        case 18:
            RightServoMotor = servo_LUT[51];            // 90 degrees
            LeftServoMotor = servo_LUT[116];
            LPM0;
            break;
        case 19:
            RightServoMotor = servo_LUT[45];            // 90 degrees
            LeftServoMotor = servo_LUT[122];
            LPM0;
            break;
        case 20:
            RightServoMotor = servo_LUT[39];            // 90 degrees
            LeftServoMotor = servo_LUT[128];
            LPM0;
            break;
        case 21:
            RightServoMotor = servo_LUT[32];            // 90 degrees
            LeftServoMotor = servo_LUT[135];
            LPM0;
            break;
        case 22:
            RightServoMotor = servo_LUT[26];            // 90 degrees
            LeftServoMotor = servo_LUT[141];
            LPM0;
            break;
        case 23:
            RightServoMotor = servo_LUT[20];            // 90 degrees
            LeftServoMotor = servo_LUT[147];
            LPM0;
            break;
        case 24:
            RightServoMotor = servo_LUT[13];            // 90 degrees
            LeftServoMotor = servo_LUT[154];
            LPM0;
            break;
        case 25:
            RightServoMotor = servo_LUT[0];            // 90 degrees
            LeftServoMotor = servo_LUT[165];
            LPM0;
            break;
        default:                                // default 0 degree case
            RightServoMotor = servo_LUT[120];
            LeftServoMotor = servo_LUT[40];
            LPM0;
        }

        switch (DcMotor)
        {
        case 0:
            throttle = 0;
            RightDcMotor = servo_LUT[0*Rollright];
            LeftDcMotor = servo_LUT[0*Rollleft];
            LPM0;
            break;
        case 1:
            throttle = 6;
            RightDcMotor = servo_LUT[55*Rollright];       //55
            LeftDcMotor = servo_LUT[5*Rollleft];
            LPM0;
            break;
        case 2:
            throttle = 13;
            RightDcMotor = servo_LUT[57*Rollright];       //60
            LeftDcMotor = servo_LUT[8*Rollleft];
            LPM0;
            break;
        case 3:
            throttle = 19;
            RightDcMotor = servo_LUT[60*Rollright];       //64
            LeftDcMotor = servo_LUT[12*Rollleft];
            LPM0;
            break;
        case 4:
            throttle = 26;
            RightDcMotor = servo_LUT[62*Rollright];       //68
            LeftDcMotor = servo_LUT[15*Rollleft];
            LPM0;
            break;
        case 5:
            throttle = 32;
            RightDcMotor = servo_LUT[65*Rollright];       //73
            LeftDcMotor = servo_LUT[19*Rollleft];
            LPM0;
            break;
        case 6:
            throttle = 39;
            RightDcMotor = servo_LUT[67*Rollright];
            LeftDcMotor = servo_LUT[22*Rollleft];
            LPM0;
            break;
        case 7:
            throttle = 45;
            RightDcMotor = servo_LUT[69*Rollright];
            LeftDcMotor = servo_LUT[24*Rollleft];
            LPM0;
            break;
        case 8:
            throttle = 52;
            RightDcMotor = servo_LUT[72*Rollright];
            LeftDcMotor = servo_LUT[27*Rollleft];
            LPM0;
            break;
        case 9:
            throttle = 58;
            RightDcMotor = servo_LUT[75*Rollright];
            LeftDcMotor = servo_LUT[30*Rollleft];
            LPM0;
            break;

        }
        LPM0;
    }
}




#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if (TA1CTL != TASSEL_2 + MC_1)          // if timer is not on
    {
        TA1CTL = TASSEL_2 + MC_1;           // turn on timer
    }

    P1IFG = 0;                              // clears port 1 flag
}

#pragma vector = TIMER1_A0_VECTOR           // timer a interrupt
__interrupt void TA1_ISR(void)
{
    if (!(BUTTON_PRESSED))                  // if button is still pressed
    {
        if (servo == 0)
        {
            servo = 1;
        }
        else if (servo == 1)
        {
            servo = 0;
        }
        else
        {
            servo = 0;
        }
    }
    TA1CTL = TASSEL_2 + MC_0;
    TA1CCTL0 &= ~(CCIFG);                   // clear flag
    LPM0_EXIT;
}

/*#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{

}*/

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    if (UCA0RXBUF == 'A')           // 'a' received?
    {
        servo = 0;
    }
    if (UCA0RXBUF == 'B')           // 'b' received?
    {
        servo = 1;
    }
    if (UCA0RXBUF == 'C')           // 't' received?
    {
        servo = 2;
    }
    if (UCA0RXBUF == 'D')           // 'f' received?
    {
        servo = 3;
    }
    if (UCA0RXBUF == 'E')           // 'r' received?
    {
        servo = 4;
    }
    if (UCA0RXBUF == 'F')           // 'g' received?
    {
        servo = 5;
    }
    if (UCA0RXBUF == 'G')           // 't' received?
    {
        servo = 6;
    }
    if (UCA0RXBUF == 'H')           // 'f' received?
    {
        servo = 7;
    }
    if (UCA0RXBUF == 'I')           // 'r' received?
    {
        servo = 8;
    }
    if (UCA0RXBUF == 'J')           // 'g' received?
    {
        servo = 9;
    }
    if (UCA0RXBUF == 'K')           // 't' received?
    {
        servo = 10;
    }
    if (UCA0RXBUF == 'L')           // 'f' received?
    {
        servo = 11;
    }
    if (UCA0RXBUF == 'M')           // 'r' received?
    {
        servo = 12;
    }
    if (UCA0RXBUF == 'N')           // 'g' received?
    {
        servo = 13;
    }
    if (UCA0RXBUF == 'O')           // 't' received?
    {
        servo = 14;
    }
    if (UCA0RXBUF == 'P')           // 'f' received?
    {
        servo = 15;
    }
    if (UCA0RXBUF == 'Q')           // 'r' received?
    {
        servo = 16;
    }
    if (UCA0RXBUF == 'R')           // 'g' received?
    {
        servo = 17;
    }
    if (UCA0RXBUF == 'S')           // 't' received?
    {
        servo = 18;
    }
    if (UCA0RXBUF == 'T')           // 'f' received?
    {
        servo = 19;
    }
    if (UCA0RXBUF == 'U')           // 'r' received?
    {
        servo = 20;
    }
    if (UCA0RXBUF == 'V')           // 'g' received?
    {
        servo = 21;
    }
    if (UCA0RXBUF == 'W')           // 't' received?
    {
        servo = 22;
    }
    if (UCA0RXBUF == 'X')           // 'f' received?
    {
        servo = 23;
    }
    if (UCA0RXBUF == 'Y')           // 'r' received?
    {
        servo = 24;
    }
    if (UCA0RXBUF == 'Z')           // 'g' received?
    {
        servo = 25;
    }
    if (UCA0RXBUF == 'a')
    {
        Roll = 0;
    }
    if (UCA0RXBUF == 'm')
    {
        Roll = 1;
    }
    if (UCA0RXBUF == 'z')
    {
        Roll = 2;
    }
    if (UCA0RXBUF == '0')
    {
        DcMotor = 0;
    }
    if (UCA0RXBUF == '1')
    {
        DcMotor = 1;
    }
    if (UCA0RXBUF == '2')
    {
        DcMotor = 2;
    }
    if (UCA0RXBUF == '3')
    {
        DcMotor = 3;
    }
    if (UCA0RXBUF == '4')
    {
        DcMotor = 4;
    }
    if (UCA0RXBUF == '5')
    {
        DcMotor = 5;
    }
    if (UCA0RXBUF == '6')
    {
        DcMotor = 6;
    }
    if (UCA0RXBUF == '7')
    {
        DcMotor = 7;
    }
    if (UCA0RXBUF == '8')
    {
        DcMotor = 8;
    }
    if (UCA0RXBUF == '9')
    {
        DcMotor = 9;
    }



UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
LPM0_EXIT;
}