#include <msp430.h>
#include <Servo.h> 

#define BUTTON BIT1
#define BUTTON_PRESSED (!(BUTTON & P1IN))
volatile int count = 0;

Servo motor1;                    // create servo object to control a servo
Servo motor2;                          // a maximum of eight servo objects can be created

int pos = 0;                    // variable to store the servo position

void setup()
{
    motor1.attach(2);            // attaches the servo on pin 2 to the servo object
    motor2.attach(6);
    TA1CCTL0 = CCIE;            // enable timer_a1 interrupt
    TA1CCR0 = 5000;             // debounce time
    TA1CTL = TASSEL_2 + MC_0;   // initialize timerA1 off
    P1REN |= BUTTON;            // pull-up resistor
    P1OUT |= BUTTON;            // button as output
    P1IE |= BUTTON;             // button interrupt flag
    P1IFG &= ~BUTTON;           // clear button flag
    __enable_interrupt();       // enable interrupts
}

void loop()
{
    switch (count)
    {
    case 0:
        motor1.write(5);
        motor2.write(5);
        delay(15);
        break;
    case 1:
        motor1.write(90);
        motor2.write(90);
        delay(15);
        break;
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
        if (count == 0)
        {
            count = 1;
        }
        else if (count == 1)
        {
            count = 0;
        }
        else
        {
            count = 0;
        }
    }
    TA1CTL = TASSEL_2 + MC_0;
    TA1CCTL0 &= ~(CCIFG);                   // clear flag
    LPM0_EXIT;
}