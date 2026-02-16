#include <msp430.h>

// All pins have to make low initally
// pin definitions

// P_EN p5.3
// P_G p4.6

// panel voltage p1.5/A3
// battery voltage p1.6/A4
// panel current p1.7/A5
// battery current p2.0/A6

// PWH1 p3.2/tb0.2
// PWL1 p3.1/tb0.1
// PWH2 p2.6/ta4.1
// PWL2 p3.5/tb0.4 (lehimle taşınacak)

#define PWM_PERIOD 40


void delay_ms(uint16_t ms) {
    while (ms--) {
        __delay_cycles(8000); // approximately 1ms in 8MHz
    }
}

void init_clock_external_8MHz(void){
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

void init_adc(void){
    P1SEL1 |= BIT5 | BIT6 | BIT7; // Configure p1.5, p1.6, p1.7 for ADC
    P1SEL0 |= BIT5 | BIT6 | BIT7;

    P2SEL1 |= BIT0; // Configure p2.0 for ADC
    P2SEL0 |= BIT0;

    ADC12CTL0 = ADC12SHT0_2 | ADC12ON | ADC12MSC; // MSC: multiple sample
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;
    ADC12CTL2 |= ADC12RES_2;

    // vref 3.3v
    ADC12MCTL0 = ADC12INCH_3 | ADC12VRSEL_0; // MEM0 = P Voltage
    ADC12MCTL1 = ADC12INCH_4 | ADC12VRSEL_0; // MEM1 = B Voltage
    ADC12MCTL2 = ADC12INCH_5 | ADC12VRSEL_0; // MEM2 = P Current
    ADC12MCTL3 = ADC12INCH_6 | ADC12VRSEL_0 | ADC12EOS; // MEM3 = B current

    ADC12CTL1 &= ~ADC12CSTARTADD_MASK; // Clear start address

    ADC12CTL0 |= ADC12ENC; // adc enable
}



void init_pwm(void){
    P3DIR |= BIT1 | BIT2 | BIT5;
    P3SEL0 &= ~BIT1 | BIT2 | BIT5;
    P3SEL1 |= BIT1 | BIT2 | BIT5;
    
    P2DIR |= BIT6;
    P2SEL0 &= ~BIT6;
    P2SEL1 |= BIT6;


    TB0CCR0 = PWM_PERIOD-1; // set period
    TB0CCTL3 = OUTMOD_7; // CCR1 Reset/Set mode
    TB0CCR3 = 0; // init speed 0
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR; // SMCLK, Up Mode, Clear
}

void init_system(void){
    WDTCTL = WDTPW | WDTHOLD; // watchdog timer closed

    init_clock_external_8MHz(); // init clock config


    // gpio settings outputs
    DIR_L_EN |= PIN_L_EN;
    DIR_ENABLE |= PIN_ENABLE;
    DIR_DIR |= PIN_DIR;
    DIR_BRAKE |= PIN_BRAKE;

    // pwm pin
    DIR_PWM |= PIN_PWM;
    SEL0_PWM &= ~PIN_PWM;
    SEL1_PWM |= PIN_PWM;

    // input
    DIR_FAULT &= ~PIN_FAULT;

    // init state
    PORT_L_EN &= ~PIN_L_EN;     // power closed
    PORT_ENABLE &= ~PIN_ENABLE; // driver closed
    PORT_BRAKE |= PIN_BRAKE;    // brake active
    PORT_DIR &= ~PIN_DIR;       // direction 0

    PM5CTL0 &= ~LOCKLPM5; // unlock gpio

    init_pwm();
    init_adc();  
}

void read_adc(void){
    ADC12CTL0 |= ADC12SC; // start conversion

    while (ADC12CTL1 & ADC12BUSY); // wait finish

    // results
    adc_result[0] = ADC12MEM0;
    adc_result[1] = ADC12MEM1;
    adc_result[2] = ADC12MEM2;
    adc_result[3] = ADC12MEM3;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT

    // GPIO Setup
    P1OUT &= ~BIT0;                         // Clear LED to start
    P1DIR |= BIT0;                          // Set P1.0/LED to output
    P1SEL1 |= BIT1;                         // Configure P1.1 for ADC
    P1SEL0 |= BIT1;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure ADC12
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;      // Sampling time, S&H=16, ADC12 on
    ADC12CTL1 = ADC12SHP;                   // Use sampling timer
    ADC12CTL2 |= ADC12RES_2;                // 12-bit conversion results
    ADC12MCTL0 |= ADC12INCH_1;              // A1 ADC input select; Vref=AVCC

    while (1)
    {
    }
}
