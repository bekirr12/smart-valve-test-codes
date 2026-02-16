#include <msp430.h>
#include <stdint.h>

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

volatile uint16_t adc_results[4];

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

void init_adc(void){
    P1SEL1 |= BIT5 | BIT6 | BIT7; // Configure p1.5, p1.6, p1.7 for ADC
    P1SEL0 |= BIT5 | BIT6 | BIT7;

    P2SEL1 |= BIT0; // Configure p2.0 for ADC
    P2SEL0 |= BIT0;

    ADC12CTL0 = ADC12ON | ADC12MSC | ADC12SHT0_2; // MSC: multiple sample
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;
    ADC12CTL2 |= ADC12RES_2;

    // vref 3.3v
    ADC12MCTL0 = ADC12INCH_3 | ADC12VRSEL_0; // MEM0 = P Voltage
    ADC12MCTL1 = ADC12INCH_4 | ADC12VRSEL_0; // MEM1 = B Voltage
    ADC12MCTL2 = ADC12INCH_5 | ADC12VRSEL_0; // MEM2 = P Current
    ADC12MCTL3 = ADC12INCH_6 | ADC12VRSEL_0 | ADC12EOS; // MEM3 = B current

    ADC12CTL0 |= ADC12ENC; // adc enable
}


void read_adc(void){
    ADC12CTL0 |= ADC12SC; // start conversion

    while (ADC12CTL1 & ADC12BUSY); // wait finish

    // results
    adc_results[0] = ADC12MEM0;
    adc_results[1] = ADC12MEM1;
    adc_results[2] = ADC12MEM2;
    adc_results[3] = ADC12MEM3;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT
    init_clock_external_8MHz();
    init_adc();
    PM5CTL0 &= ~LOCKLPM5;
    while(1) {
        read_adc();

        // delay
        __delay_cycles(100000); 
    }
}
