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

// Calculate currents, voltages, and power

/*
Panel voltage,
Battery voltage,
Panel current,
battery current,
panel power,
battery power

Voltage Constant
resistor values: voltage divider --> 178k, 10k = rate 18.8.
adc step: 3300mV/4095= 0.8058mV.
constant value: 0.8058 x 18.8 = 15.15mV.
Every adc step equals to 15.15mV.
code constant: 15.15 x 4096 = 62054.

Current Constant
resistor values: 10k / 402 = (gain = 24.8750).
shunt= 2mohm.
Sense: 24.875x0.002 = 0.04975 V/A (49.75 mV/A).
adc step: 3300mV/4095= 0.8058mV.
current constant: (0.8058mV) / (0.04975 V/A) x 1000 = 16.197
Every adc step equals to 16.2mA
code constant: 16.2 x 4096 = 66343.

*/



#include <msp430.h>
#include <stdint.h>

#define V_MULTIPLIER    62054UL // voltage constant
#define I_MULTIPLIER    66343UL // current constant

#define SHIFT_AMOUNT    12  // X >> 12 means X / 4096.

volatile uint16_t adc_results[4];

// physical values
typedef struct {
    uint16_t Panel_Voltage_mV;
    uint16_t Battery_Voltage_mV;
    uint16_t Panel_Current_mA;
    uint16_t Battery_Current_mA;
    uint32_t Panel_Power_mW;
    uint32_t Battery_Power_mW;
} SystemData;

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

void init_gpio_default_low(void) {
    // GPIO Configuration
    P1OUT = 0;
    P1DIR = 0xFF;

    P2OUT = 0;
    P2DIR = 0xFF;

    P3OUT = 0;
    P3DIR = 0xFF;

    P4OUT = 0;
    P4DIR = 0xFF;

    P5OUT = 0;
    P5DIR = 0xFF;

    P6OUT = 0;
    P6DIR = 0xFF;

    P7OUT = 0;
    P7DIR = 0xFF;

    P8OUT = 0;
    P8DIR = 0xFF;

    P9OUT = 0;
    P9DIR = 0xFF;
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
/*
// This function is for debugging purposes only. 
// We don't need to use it in the firmware. 
// We can perform our operations directly through the ADC values.
void update_display_data(void){
    // 1. voltage calculate: (ADC value * 62054) / 4096
    uint32_t temp_calc;

    temp_calc = (uint32_t)adc_results[0] * V_MULTIPLIER;
    solar_data.Panel_Voltage_mV = (uint16_t)(temp_calc >> SHIFT_AMOUNT);

    temp_calc = (uint32_t)adc_results[1] * V_MULTIPLIER;
    solar_data.Battery_Voltage_mV = (uint16_t)(temp_calc >> SHIFT_AMOUNT);

    // 2. current calculate: (ADC value * 66343) / 4096

    // basic noise filter
    if(adc_results[2] > 5) {
        temp_calc = (uint32_t)adc_results[2] * I_MULTIPLIER;
        solar_data.Panel_Current_mA = (uint16_t)(temp_calc >> SHIFT_AMOUNT);
    } else {
        solar_data.Panel_Current_mA = 0;
    }

    if(adc_results[3] > 5) {
        temp_calc = (uint32_t)adc_results[3] * I_MULTIPLIER;
        solar_data.Battery_Current_mA = (uint16_t)(temp_calc >> SHIFT_AMOUNT);
    } else {
        solar_data.Battery_Current_mA = 0;
    }

    // 3. power calculate: P(mW) = V(mV) * I(mA) / 1000
    solar_data.Panel_Power_mW = ((uint32_t)solar_data.Panel_Voltage_mV * solar_data.Panel_Current_mA) / 1000;
    solar_data.Battery_Power_mW = ((uint32_t)solar_data.Battery_Voltage_mV * solar_data.Battery_Current_mA) / 1000;

}
*/

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop WDT
    init_gpio_default_low(); // all pins low
    init_clock_external_8MHz();
    init_adc();
    PM5CTL0 &= ~LOCKLPM5;
    while(1) {
        read_adc();
        // update_display_data();
        // delay
        __delay_cycles(100000); 
    }
}
