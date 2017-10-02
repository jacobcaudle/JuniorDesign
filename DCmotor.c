#include <msp430.h>

#define BUTTON BIT3
#define BUTTON_PRESSED (!(BUTTON & P1IN))
#define OUTPUT BIT6
volatile int count = 0;
enum StateType {
	SLOW, FAST, OFF
};

void main(void) {
	WDTCTL   = WDTPW + WDTHOLD;     		// Stop watchdog timer
	enum StateType state = SLOW;
	TA1CCTL0 = CCIE;					// enable timer_a1 interrupt
	TA1CCR0 = 5000;					// debounce time
	TA1CTL = TASSEL_2 + MC_0;			// initialize timerA1 off
	P1REN |= BUTTON;					// pull-up resistor
	P1OUT |= BUTTON;					// button as output
	P1IE |= BUTTON;					// button interrupt flag
	P1IFG &= ~BUTTON;					// clear button flag
	P1DIR |= OUTPUT;
	P1SEL |= OUTPUT;
	__enable_interrupt();

	while (1) {

		switch (state) {
		case SLOW:
			if (count == 0) {
				TACCR0 = 5000 - 1;
				TACCR1 = 5000 * .32;
				TACCTL1 = OUTMOD_7;	// reset/set
				TA0CTL = TASSEL_2 + MC_1;// SMCLK UP MODE
				LPM0;
			}
			if (count == 1)
			{
				state = FAST;
			}
			break;
		case FAST:
			if (count == 1) {
				TACCR0 = 5000 - 1;
				TACCR1 = 5000 * .1;
				TACCTL1 = OUTMOD_7;	// reset/set
				TA0CTL = TASSEL_2 + MC_1;// SMCLK UP MODE
				LPM0;
			}
			if (count == 2)
			{
				state = OFF;
			}
			break;
		case OFF:
			if (count == 2) {
				TACCR0 = 5000 - 1;
				TACCR1 = 5000 * 1;
				TACCTL1 = OUTMOD_7;	// reset/set
				TA0CTL = TASSEL_2 + MC_1;// SMCLK UP MODE
				LPM0;
			}
			else
			{
				TACCR0 = 5000 - 1;
				TACCR1 = 5000 * 1;
				TACCTL1 = OUTMOD_7;	// reset/set
				TA0CTL = TASSEL_2 + MC_1;// SMCLK UP MODE
				LPM0;
			}
			if (count == 0)	{
				state = SLOW;
			}
			break;
		default:
			state = SLOW;
		}
	}
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
	if (TA1CTL != TASSEL_2 + MC_1)		// if timer is not on
	{
		TA1CTL = TASSEL_2 + MC_1;		// turn on timer
	}

	P1IFG = 0;                          	// clears port 1 flag
}

#pragma vector = TIMER1_A0_VECTOR 			// timer a interrupt
__interrupt void TA1_ISR(void) {
	if (BUTTON_PRESSED)     			// if button is still pressed
	{
		if (count < 2) {
			count++;
		} else {
			count = 0;
		}

		TA1CTL = MC_0;         			// stop debouncing clock
		TA1CCTL0 &= ~(CCIFG); 			// clear flag
		LPM1_EXIT;
	}
}