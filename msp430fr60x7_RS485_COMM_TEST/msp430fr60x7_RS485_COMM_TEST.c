//******************************************************************************
//   MSP430FR60xx - RS485 Controlled BLDC Motor Driver with Dynamic Power Scaling
//
//   Description:
//   This application controls a BLDC motor via RS485 commands. It features a 
//   dynamic clock system to maximize power efficiency:
//
//   1. IDLE STATE (Motor Stopped):
//      - System enters LPM3 (Deep Sleep).
//      - CPU/MCLK runs at minimal DCO frequency or turns off.
//      - UART remains active using ACLK (32.768kHz LFXT) to listen for commands.
//      - High-Frequency Crystal (HFXT) is DISABLED to save power.
//
//   2. ACTIVE STATE (Motor Running):
//      - System wakes up to LPM0.
//      - HFXT (8MHz) is enabled dynamically.
//      - SMCLK switches to 8MHz to generate high-resolution PWM (200kHz).
//      - CPU runs at 8MHz for fast processing.
//
//   RS485 Protocol Format:
//   ----------------------
//   [HEADER] [DATA] [FINISH]
//   - HEADER: 0xAA
//   - FINISH: 0xFF
//   - DATA Byte Structure: [DIR (Bit 7)] | [SPEED (Bits 6-0)]
//       * Bit 7 (MSB): Direction (0 = CW, 1 = CCW)
//       * Bits 0-6   : Speed Duty Cycle (0 - 100%)
//
//   Pin Configuration:
//   ------------------
//   RS485 Transceiver:
//     P4.3 -> UART TXD (DI)
//     P4.4 -> UART RXD (RO)
//     P4.5 -> RS485 Enable (DE/RE) - Active High for TX, Low for RX
//
//   Motor Driver Interface:
//     P5.2 -> L_EN (Power Enable)
//     P9.2 -> ENABLE (Driver Enable)
//     P9.0 -> DIR (Direction)
//     P9.1 -> BRAKE (Active Low/High based on driver)
//     P3.3 -> PWM Output (TB0.3)
//
//   Clocks:
//     ACLK  = 32.768 kHz (LFXT) -> Used for UART (Always On)
//     SMCLK = 8 MHz (HFXT)      -> Used for PWM (On Demand)
//
//******************************************************************************

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// rs485 pins P4.3 TX, P4.4 RX, P4.5 EN
#define RS485_PORT_SEL0     P4SEL0
#define RS485_PORT_SEL1     P4SEL1
#define RS485_PORT_DIR      P4DIR
#define RS485_PORT_OUT      P4OUT
#define PIN_TX              BIT3
#define PIN_RX              BIT4
#define PIN_RS485_EN        BIT5

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

// Protocol
#define PKT_HEADER      0xAA
#define PKT_FINISH      0xFF

#define PWM_PERIOD      40      // 200 kHz (8MHz / 40)
#define MAX_SPEED_VAL   40

// UART Initialization
#define SMCLK_115200     0
#define SMCLK_9600      1
#define ACLK_9600       2

#define UART_MODE       ACLK_9600//SMCLK_115200//

// Global Variables
volatile uint8_t rx_buffer[3];  // receiving buffer
volatile uint8_t rx_index = 0;
volatile uint8_t motor_update_req = 0; // new package flag
volatile uint8_t target_data = 0;      // Parsed data


void initSystem(void)
{
  // gpio settings
  DIR_L_EN |= PIN_L_EN;       PORT_L_EN &= ~PIN_L_EN;
  DIR_ENABLE |= PIN_ENABLE;   PORT_ENABLE &= ~PIN_ENABLE;
  DIR_DIR |= PIN_DIR;         PORT_DIR &= ~PIN_DIR;
  DIR_BRAKE |= PIN_BRAKE;     PORT_BRAKE |= PIN_BRAKE;
  
  // pwm pin
  DIR_PWM |= PIN_PWM;
  SEL0_PWM &= ~PIN_PWM;
  SEL1_PWM |= PIN_PWM;

  // RS485 Enable Pin
  RS485_PORT_DIR |= PIN_RS485_EN;
  RS485_PORT_OUT &= ~PIN_RS485_EN; // low rx mode

  // UART Pins(P4.3 TX, P4.4 RX)
  RS485_PORT_SEL0 &= ~(PIN_TX | PIN_RX);
  RS485_PORT_SEL1 |= (PIN_TX | PIN_RX);

  // clock settings
  PJSEL0 |= BIT4 | BIT5; // LFXT Pinleri

  CSCTL0_H = CSKEY_H;
  CSCTL4 &= ~LFXTOFF; // LFXT open
  do {
      CSCTL5 &= ~LFXTOFFG;
      SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG);
  
  // ACLK = LFXT (32kHz), MCLK/SMCLK = DCO
  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
  CSCTL0_H = 0;

  // 3. UART settinga (ACLK @ 9600 Baud)
  UCA0CTLW0 = UCSWRST;
  UCA0CTLW0 |= UCSSEL__ACLK;

  // 32768 / 9600 = 3.41
  UCA0BR0 = 3;
  UCA0BR1 = 0;
  UCA0MCTLW |= 0x9200; // modulation

  UCA0CTLW0 &= ~UCSWRST;
  UCA0IE |= UCRXIE; // interrupt open

  TB0CCR0 = PWM_PERIOD - 1;
  TB0CCTL3 = OUTMOD_7;
  TB0CCR3 = 0;

  PM5CTL0 &= ~LOCKLPM5;
}

void setMotor(uint8_t data_byte)
{
  bool direction_ccw = (data_byte & 0x80) ? 1 : 0; // 7. bit direciton
  uint8_t speed_raw = (data_byte & 0x7F);          // other 7 bit speed

  // scale pwm to period
  uint16_t pwm_val = 0;
  if(speed_raw > 100) speed_raw = 100;
  pwm_val = (speed_raw * PWM_PERIOD) / 100;

  // state control 
  if (pwm_val == 0)
  {
      // stop motor
      TB0CCR3 = 0;
      PORT_ENABLE &= ~PIN_ENABLE;
      PORT_L_EN &= ~PIN_L_EN;
      PORT_BRAKE |= PIN_BRAKE;

      // close high freq clock
      disableHighPerfClock();
  }
  else
  {
    // open high freq clock
    enableHighPerfClock();

    if (direction_ccw) PORT_DIR |= PIN_DIR;
    else               PORT_DIR &= ~PIN_DIR;

    // enable
    PORT_L_EN |= PIN_L_EN;
    PORT_ENABLE |= PIN_ENABLE;
    PORT_BRAKE &= ~PIN_BRAKE;

    // set speed
    TB0CCR3 = pwm_val;
  }
}

// activate 8mhz smclk
void enableHighPerfClock(void)
{
  if (CSCTL2 & SELS__HFXTCLK) return; // not open again

  PJSEL0 |= BIT6 | BIT7;  // HFXT Pins
  PJSEL1 &= ~(BIT6 | BIT7);

  CSCTL0_H = CSKEY_H; // open lock
  CSCTL4 &= ~HFXTOFF; // open HFXT

  // wait stabilize crystal
  do {
      CSCTL5 &= ~HFXTOFFG;
      SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG);

 
  CSCTL2 = SELA__LFXTCLK | SELS__HFXTCLK | SELM__HFXTCLK; 
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;  // no divider
  CSCTL0_H = 0; // lock
  
  // Timer'ı başlat (SMCLK geldiği için saymaya başlar)
  TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR; 
}

// Sistemi yavaşlatır, 8MHz kristali kapatır
void disableHighPerfClock(void)
{
  TB0CTL = MC__STOP; // stop timer

  CSCTL0_H = CSKEY_H;  // open lock
  // MCLK ve SMCLK default value 1mhz
  CSCTL1 = DCOFSEL_0;  // 1mhz dco
  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; 
  
  CSCTL4 |= HFXTOFF;  // close external cyrstal
  CSCTL0_H = 0;  // lock
}


int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop Watchdog

  initSystem();

  __bis_SR_register(LPM3_bits + GIE);

  while (1)
  {
    if (motor_update_req)
    {
        motor_update_req = 0;
        setMotor(target_data);
    }
    if (TB0CCR3 > 0) 
    {
        __bis_SR_register(LPM0_bits + GIE); 
    }
    else 
    {
        __bis_SR_register(LPM3_bits + GIE);
    }
  }
}

// interrupt function (ISR)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      uint8_t rx_byte = UCA0RXBUF;

      if (rx_index == 0){
        if (rx_byte = PKT_HEADER)
        {
          rx_buffer[0] = rx_byte;
          rx_index++;
        }
      } else if (rx_index == 1){
          rx_buffer[1] = rx_byte;
          rx_index++;
      } else if(rx_index == 2) {
          if(rx_byte = PKT_FINISH){
            target_data = rx_buffer[1];
            motor_update_req = 1;

            __bic_SR_register_on_exit(LPM3_bits);
          }
          rx_index = 0;
      }
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}
