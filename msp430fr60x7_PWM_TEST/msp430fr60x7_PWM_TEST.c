// This code optimized for pwm test

// PWH1 p3.2/tb0.2
// PWL1 p3.1/tb0.1
// PWH2 p2.6/ta4.1
// PWL2 p3.5/tb0.4 (lehimle taşınacak)
// PW_L2 p2.5/ta4.0 (eski)

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#define PWM_PERIOD 40   // 8MHz / 200kHz = 40 Tick
#define DEAD_TIME  3    // ~187ns

void delay_ms(uint16_t ms) {
    while (ms--) {
        __delay_cycles(8000); // approximately 1ms in 8MHz
    }
}

void init_clock_external_8MHz(void){
    FRCTL0 = FRCTLPW | NWAITS_1;
    // Set cyrsyal pins PJ.6 ve PJ.7 -> HFXT
    PJSEL0 |= BIT6 | BIT7;
    PJSEL1 &= ~(BIT6 | BIT7);
    CSCTL0_H = CSKEY_H; // open clock system lock
      // set drive and freq
    CSCTL4 &= ~HFXTOFF; // open hfxt

    // stabilization cycle
    do {
        CSCTL5 &= ~HFXTOFFG;       // clear HFXT flag
        SFRIFG1 &= ~OFIFG;         // clear oscillator flag
    } while (SFRIFG1 & OFIFG);

    CSCTL2 = SELS__HFXTCLK | SELM__HFXTCLK; // smclk = mclk = hfxt = 8mhz

    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // set divider 

    CSCTL0_H = 0;   // lock
}

void init_pwm_pins(void) {
    // P3.1, P3.2, P3.5
    P3DIR |= (BIT1 | BIT2 | BIT5);
    P3SEL1 |= (BIT1 | BIT2 | BIT5);
    P3SEL0 &= ~(BIT1 | BIT2 | BIT5);

    // p2.5, P2.6
    P2DIR |= BIT5 | BIT6;
    P2SEL1 &= ~(BIT5 | BIT6);
    P2SEL0 |= BIT5 | BIT6;
}

void test_basic_connection(uint16_t duty) {
    // Timer B0
    TB0CCR0 = PWM_PERIOD - 1;
    TB0CCTL1 = OUTMOD_7; TB0CCR1 = duty; // PWL1
    TB0CCTL2 = OUTMOD_7; TB0CCR2 = duty; // PWH1
    TB0CCTL4 = OUTMOD_7; TB0CCR4 = duty; // PWL2
    
    // Timer A4
    TA4CCR0 = PWM_PERIOD - 1;
    TA4CCTL0 = OUTMOD_7; TA4CCR0 = duty;
    TA4CCTL1 = OUTMOD_7; TA4CCR1 = duty; // PWH2

    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
    TA4CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

void test_asynchronous_mode(uint16_t duty) {
    if(duty > PWM_PERIOD) duty = PWM_PERIOD;

    // --- SETUP ---
    TB0CCR0 = PWM_PERIOD - 1;
    TA4CCR0 = PWM_PERIOD - 1;

    // phase 1
    // PWH1 (TB0.2): mode 7
    TB0CCTL2 = OUTMOD_7; 
    TB0CCR2 = duty;
    
    // PWL1 (TB0.1): close
    TB0CCTL1 = OUTMOD_0; TB0CCR1 = 0;


    // phase 2
    // PWH2 (TA4.1): mode 3 (inverse)
    // formula: Period - Duty
    TA4CCTL1 = OUTMOD_3;
    if(duty == 0) TA4CCR1 = 0;
    else TA4CCR1 = PWM_PERIOD - duty;

    // PWL2 (TB0.4): close
    TB0CCTL4 = OUTMOD_0; TB0CCR4 = 0;

    // start
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
    TA4CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

// Senkron mod with dead time
void test_synchronous_mode(uint16_t duty) {
    // dead time
    if(duty > (PWM_PERIOD - DEAD_TIME - 2)) duty = PWM_PERIOD - DEAD_TIME - 2;

    // --- SETUP ---
    TB0CCR0 = PWM_PERIOD - 1;
    TA4CCR0 = PWM_PERIOD - 1;

    // phase 1
    // PWH1 (TB0.2): mode 7
    TB0CCTL2 = OUTMOD_7; // Reset/Set
    TB0CCR2 = duty;

    // PWL1 (TB0.1): complement (Duty+DeadTime -> Period)
    TB0CCTL1 = OUTMOD_3; // Set/Reset
    TB0CCR1 = duty + DEAD_TIME;


    // phase 2 
    // PWH2 (TA4.1): inverse pwm mod 3 (Period-Duty -> Period)
    // Mode 3: Set/Reset
    TA4CCTL1 = OUTMOD_3;
    TA4CCR1 = PWM_PERIOD - duty;

    // PWL2 (TB0.4): complement (0 -> Period-Duty-DeadTime)
    // Mode 7: Reset/Set
    TB0CCTL4 = OUTMOD_7;
    TB0CCR4 = PWM_PERIOD - duty - DEAD_TIME;

    // start
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
    TA4CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT

    init_clock_external_8MHz();
    init_pwm_pins();

    PM5CTL0 &= ~LOCKLPM5;

    test_basic_connection(20);

    // test_asynchronous_mode(40);

    // test_synchronous_mode(40);

    while(1) {
        // infinite loop
        __no_operation();
    }
}
