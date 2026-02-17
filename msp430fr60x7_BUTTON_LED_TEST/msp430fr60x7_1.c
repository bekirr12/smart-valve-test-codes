/*
This code tests button and led. 
led: p1.5
button: p5.4, p5.5, p5.6, p5.7
*/

#include <msp430.h>

void init_buttons_internal_pullup(void) {
    // configure led
    P1OUT &= ~BIT0;
    P1DIR |= BIT0;
    
    // set buttons pin to input
    P5DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7);

    // enable internal resistor
    P5REN |= (BIT4 | BIT5 | BIT6 | BIT7);

    // set resistor pull up
    P5OUT |= (BIT4 | BIT5 | BIT6 | BIT7);
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT

    init_buttons_internal_pullup();

    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings

    while(1)
    {
        if ( ((P5IN & BIT4) == 0) || 
             ((P5IN & BIT5) == 0) || 
             ((P5IN & BIT6) == 0) ||
             ((P5IN & BIT7) == 0) ) 
        {
            P1OUT |= BIT0; // high
        } 
        else 
        {
            P1OUT &= ~BIT0; // low
        }
    }
}
