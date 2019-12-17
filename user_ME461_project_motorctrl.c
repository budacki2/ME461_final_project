/******************************************************************************
MSP430F2272 Project Creator 4.0

ME 461 - S. R. Platt
Fall 2010

Updated for CCSv4.2 Rick Rekoske 8/10/2011

Written by: Steve Keres
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430x22x2.h"
#include "UART.h"

// Create your global variables here:

char newprint = 0;
unsigned int timecnt = 0;
unsigned int fastcnt = 0;
unsigned int fastcnt2 = 0;
int freq = 100;
int photoresist[4] = {0,0,0,0}; //two element integer array for sampled DTC channels
long A1_mv=0,A0_mv=0,A2_mv=0,A3_mv; // stores V reading from A1, A0
char P2_sw = 0;
unsigned int counter = 0;
unsigned int dutycounter = 0;

// motor selection variables
int A0_dutylimit = 0;
int A1_dutylimit = 0;
int A2_dutylimit = 0;
int A3_dutylimit = 0;

// motor selection and duty cycle
int A0_motorstate = 20;
int A1_motorstate = 20;
int A2_motorstate = 20;
int A3_motorstate = 20;

// color thresholds
int A0_white = 900;
int A0_black = 500;
int A1_white = 900;
int A1_black = 700;
int A2_white = 2100;
int A2_black = 1950;
int A3_white = 1100;
int A3_black = 500;

void main(void) {

    WDTCTL = WDTPW + WDTHOLD; // Stop WDT

    if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

    DCOCTL  = CALDCO_16MHZ; // Set uC to run at approximately 16 Mhz
    BCSCTL1 = CALBC1_16MHZ;

    //P1IN          Port 1 register used to read Port 1 pins setup as Inputs
    P1SEL &= ~0xFF; // Set all Port 1 pins to Port I/O function
    P1REN &= ~0xFF; // Disable internal resistor for all Port 1 pins
    P1DIR |= 0xFF;   // Set all Port 1 pins to outputs
    P1OUT &= ~0xFF; // Initially set all Port 1 pins set as Outputs to zero

    // Timer A Config
    TACCTL0 = CCIE;              // Enable Timer A interrupt
    TACCR0  = 8000;             // period = 1ms
    TACTL   = TASSEL_2 + MC_1;   // source SMCLK, up mode

    P4SEL &= ~0x1E; // Sets P4.1 to TB1
    P4DIR |= 0x1E; // Sets P4.1 to TB1

    ADC10CTL0 |= SREF_0 + ADC10ON + ADC10IE + MSC; // Period = 51.2 us - Sets reference to GND and 3.3V, turns on ADC10, enables ADC10 interrupt
    ADC10CTL1 |= INCH_3 + CONSEQ_1 + ADC10SSEL_0 + SHS_0; // Uses function generator as sampling channel and clock source to ADC10OSC, sets input channel to A3
    ADC10AE0 |= 0x0F; // Enables channels we want to sample (A0,A1,A2,A3 = 1111 = hex 0F)
    ADC10DTC1 |= 0x04; // number of channels we want to sample
    ADC10SA = (int)&photoresist[0];

    Init_UART(115200, 1);   // Initialize UART for 9600 baud serial communication

    _BIS_SR(GIE);       // Enable global interrupt

    while(1) {

        if(newmsg) {
            //my_scanf(rxbuff,&var1,&var2,&var3,&var4);
            newmsg = 0;
        }

        if (newprint)  {
            P1OUT ^= 0x1; // Blink LED
            UART_printf("%d %d %d %d\n\r",(int)A0_mv, (int)A1_mv, (int)A2_mv, (int)A3_mv); //  %d int, %ld long, %c char, %x hex form, %.3f float 3 decimal place, %s null terminated character array
            // UART_send(1,(float)timecnt);

            timecnt++;  // Just incrementing this integer for default print out.
            newprint = 0;
        }

    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
    fastcnt++; // Keep track of time for main while loop.
    if (fastcnt == 50) {
        fastcnt = 0;
        newprint = 1;  // flag main while loop that .5 seconds have gone by.
    }

    ADC10CTL0 |= ENC + ADC10SC; // Enables ADC10 and starts conversion once every ms

    // Put your Timer_A code here:

    // MOTOR YEAH

    if (counter == 0) { // starts counting from 0, runs one time

        P4OUT |= 0x1E; // pulls all motors high
        A0_dutylimit = A0_motorstate; // sets duty cycle based on photoresistor
        A1_dutylimit = A1_motorstate;
        A2_dutylimit = A2_motorstate;
        A3_dutylimit = A3_motorstate;

        dutycounter = 0; // resets duty counter
    }

    if (dutycounter == A0_dutylimit) { // has motor 1 hit duty cycle limit? (X/200)
        P4OUT &= ~0x02; // if yea, turn off
    }

    if (dutycounter == A1_dutylimit) {
        P4OUT &= ~0x04;
    }

    if (dutycounter == A2_dutylimit) {
        P4OUT &= ~0x08;
    }

    if (dutycounter == A3_dutylimit) {
        P4OUT &= ~0x10;
    }

    counter++; // increments counter every loop (once every 0.1 ms)
    dutycounter++; // increments duty cycle counter every loop

    if (counter >= 40) { // if counter hits 200
        counter = 0;
    }
}

// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    A3_mv = (photoresist[0]*3300L)/1023; // converts ADC10 memory to mV
    A2_mv = (photoresist[1]*3300L)/1023; // converts ADC10 memory to mV
    A1_mv = (photoresist[2]*3300L)/1023; // converts ADC10 memory to mV
    A0_mv = (photoresist[3]*3300L)/1023; // converts ADC10 memory to mV

    if (A0_mv < A0_black) {
        A0_motorstate = 2;
    }

    if (A0_mv > A0_white) {
        A0_motorstate = 3;
    }

    if (A1_mv < A1_black) {
        A1_motorstate = 3;
    }

    if (A1_mv > A1_white) {
        A1_motorstate = 2;
    }

    if (A2_mv < A2_black) {
        A2_motorstate = 2;
    }

    if (A2_mv > A2_white) {
        A2_motorstate = 3;
    }

    if (A3_mv < A3_black) {
        A3_motorstate = 3;
    }

    if (A3_mv > A3_white) {
        A3_motorstate = 2;
    }

    /*if ((A1_mv > 1300) && (A1_mv < 1400)) {
        fastcnt2++;
        if (fastcnt2 == freq*1) {
            P1OUT &= 0x00;
            P1OUT |= 0x01;
        }
        if (fastcnt2 == freq*2) {
            P1OUT &= 0x00;
            P1OUT |= 0x02;
        }
        if (fastcnt2 == freq*3) {
            P1OUT &= 0x00;
            P1OUT |= 0x04;
        }
        if (fastcnt2 == freq*4) {
            P1OUT &= 0x00;
            P1OUT |= 0x08;
        }
        if (fastcnt2 == freq*5) {
            P1OUT &= 0x00;
            P1OUT |= 0x10;
        }
        if (fastcnt2 == freq*6) {
            P1OUT &= 0x00;
            P1OUT |= 0x20;
        }
        if (fastcnt2 == freq*7) {
            P1OUT &= 0x00;
            P1OUT |= 0x40;
        }
        if (fastcnt2 >= freq*8) {
            P1OUT &= 0x00;
            P1OUT |= 0x80;
            fastcnt2 = 0;
        }
    }*/

    ADC10SA = (int)&photoresist[0];
}



// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {
  
    if((IFG2&UCA0TXIFG) && (IE2&UCA0TXIE)) { // USCI_A0 requested TX interrupt
        if(printf_flag) {
            if (currentindex == txcount) {
                senddone = 1;
                printf_flag = 0;
                IFG2 &= ~UCA0TXIFG;
            } else {
            UCA0TXBUF = printbuff[currentindex];
            currentindex++;
            }
        } else if(UART_flag) {
            if(!donesending) {
                UCA0TXBUF = txbuff[txindex];
                if(txbuff[txindex] == 255) {
                    donesending = 1;
                    txindex = 0;
                } else {
                    txindex++;
                }
            }
        }

        IFG2 &= ~UCA0TXIFG;
    }

    if((IFG2&UCB0TXIFG) && (IE2&UCB0TXIE)) { // USCI_B0 requested TX interrupt (UCB0TXBUF is empty)

        IFG2 &= ~UCB0TXIFG;   // clear IFG
    }
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
  
    if((IFG2&UCA0RXIFG) && (IE2&UCA0RXIE)) { // USCI_A0 requested RX interrupt (UCA0RXBUF is full)

        if(!started) {  // Haven't started a message yet
            if(UCA0RXBUF == 253) {
                started = 1;
                newmsg = 0;
            }
        } else { // In process of receiving a message
            if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
                rxbuff[msgindex] = UCA0RXBUF;

                msgindex++;
            } else { // Stop char received or too much data received
                if(UCA0RXBUF == 255) { // Message completed
                    newmsg = 1;
                    rxbuff[msgindex] = 255; // "Null"-terminate the array
                }
                started = 0;
                msgindex = 0;
            }
        }
        IFG2 &= ~UCA0RXIFG;
    }

    if((IFG2&UCB0RXIFG) && (IE2&UCB0RXIE)) { // USCI_B0 requested RX interrupt (UCB0RXBUF is full)

        IFG2 &= ~UCB0RXIFG; // clear IFG
    }
  
}



