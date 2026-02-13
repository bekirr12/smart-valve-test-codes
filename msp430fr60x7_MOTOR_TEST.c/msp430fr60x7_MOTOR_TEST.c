/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//*******************************************************************************
//  MSP430FR60xx Demo - TimerB, PWM TB0.1-2, Up Mode, DCO SMCLK
//
//  Description: This program generates two PWM outputs on P3.1,P3.2 using
//  TimerB configured for up mode. The value in CCR0, 1000-1, defines the PWM
//  period and the values in CCR1 and CCR2 the PWM duty cycles. Using ~1MHz
//  SMCLK as TACLK, the timer period is ~1ms with a 75% duty cycle on P3.1
//  and 25% on P3.2.
//  ACLK = n/a, SMCLK = MCLK = TACLK = 1MHz
//
//           MSP430FR6047
//         ---------------
//     /|\|               |
//      | |               |
//      --|RST            |
//        |               |
//        |     P3.1/TB0.1|--> CCR1 - 75% PWM
//        |     P3.2/TB0.2|--> CCR2 - 25% PWM
//
//   Evan Wakefield
//   Texas Instruments Inc.
//   October 2016
//   Built with IAR Embedded Workbench V6.50 & Code Composer Studio V6.2
//******************************************************************************


// This code has been changed for bldc motor test by Nuvo


#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// L_EN p5.2
#define PORT_L_EN       P5OUT
#define DIR_L_EN        P5DIR
#define PIN_L_EN        BIT2

// enable p9.2
#define PORT_ENABLE     P9OUT
#define DIR_ENABLE      P9DIR
#define PIN_ENABLE      BIT2

// direction p9.0
#define PORT_DIR        P9OUT
#define DIR_DIR         P9DIR
#define PIN_DIR         BIT0

// brake p9.1
#define PORT_BRAKE      P9OUT
#define DIR_BRAKE       P9DIR
#define PIN_BRAKE       BIT1

// pwm output speed control p3.3, tb0.3
#define PORT_PWM        P3OUT
#define DIR_PWM         P3DIR
#define SEL0_PWM        P3SEL0
#define SEL1_PWM        P3SEL1
#define PIN_PWM         BIT3

// fault p3.7
#define PORT_FAULT      P3IN
#define DIR_FAULT       P3DIR
#define PIN_FAULT       BIT7

// motor current L_I
#define PORT_ADC        P2OUT
#define DIR_ADC         P2DIR
#define SEL0_ADC        P2SEL0
#define SEL1_ADC        P2SEL1
#define PIN_ADC         BIT1

#define CURRENT_LIMIT   2000

// pwm settings 
#define PWM_PERIOD      40      // 200 khz     
#define PWM_MAX_DUTY    40     // %100 (10V out)

volatile uint16_t current_val = 0; // motor current value

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
    P2SEL1 |= PIN_ADC;
    P2SEL0 |= PIN_ADC;

    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
    ADC12CTL1 = ADC12SHP;
    ADC12CTL2 |= ADC12RES_2;
    ADC12MCTL0 = ADC12INCH_7 | ADC12VRSEL_0;
    ADC12CTL0 |= ADC12ENC;
}

void init_pwm(void){
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

void motor_set_speed(uint8_t duty) {
    if(duty > PWM_PERIOD) duty = PWM_PERIOD; // safety limit
    TB0CCR3 = duty; // update duty cycle
}

uint16_t read_motor_current(void){
    ADC12CTL0 |= ADC12SC; // start reading
    while (!(ADC12IFG0 & ADC12IFG0)); // Wait for completion
    return ADC12MEM0; // return result
}

int main(void)
{
    int i;
    init_system();
    delay_ms(1000); // wait 1sn

    PORT_L_EN |= PIN_L_EN; // high L_EN
    delay_ms(100);

    PORT_ENABLE |= PIN_ENABLE; // high driver's enable
    PORT_BRAKE &= ~PIN_BRAKE; // low brake control
    delay_ms(500);

    while (1) {
        // test1 forward moving
        PORT_DIR &= ~PIN_DIR; // set direction 0

        // soft start from 0 to 100 2sn
        for (i=0; i<=40; i++){
            motor_set_speed(i);
            current_val = read_motor_current();
            // safety check
            /*
            
            if (current_val > CURRENT_LIMIT){
                // STOP everything
                motor_set_speed(0);
                PORT_ENABLE &= ~PIN_ENABLE;
                PORT_L_EN &= ~PIN_L_EN;
                delay(1000);
            }
            */
            delay_ms(50);
        }

        delay_ms(2000);

        // soft stop 2sn
        for (i=40; i>=0; i--){
            motor_set_speed(i);
            delay_ms(50);
        }

        // stop completely
        motor_set_speed(0);
        delay_ms(1000);

        // test 2 backward moving
        PORT_DIR |= PIN_DIR;

        for (i=0; i<=40; i++){
            motor_set_speed(i);
            delay_ms(50);
        }

        delay_ms(2000);

        // soft stop 2sn
        for (i=40; i>=0; i--){
            motor_set_speed(i);
            delay_ms(50);
        }

        // stop completely
        motor_set_speed(0);
        delay_ms(1000);
    }
}
