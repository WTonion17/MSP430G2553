#include "msp430.h"
#include "lcd.h"
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h>

#define TRIG BIT0                               //P1.0
#define ECHO BIT1                               //P1.1

#define IN1 BIT3                                //P1.2

#define MOTOR_FORWARD BIT1                      //P2.1
#define MOTOR_REVERSE BIT0                      //P2.0

#define RAIN_SENSOR BIT2                        //P2.2

#define LIMIT_SWITCH_PIN1 BIT3                  //P2.3
#define LIMIT_SWITCH_PIN2 BIT4                  //P2.4


unsigned int distance;
unsigned int Maindistance;
unsigned int Maxdistance = 205;

unsigned char customChar[8] = {
  0x00,
  0x00,
  0x15,
  0x0E,
  0x1F,
  0x0E,
  0x15,
  0x00
};

unsigned char customChar2[8] = {
  0x04,
  0x04,
  0x0E,
  0x0E,
  0x1F,
  0x1F,
  0x1F,
  0x0E
};

bool isBlindOpen() {
    if (P2IN & RAIN_SENSOR) {
        return true;
    } else {
        return false;
    }
}

bool digital1() {
    if (P2IN &  RAIN_SENSOR) {
        return true;
    } else {
        return false;
    }
}

//Hàm hiển thị thông báo mưa lên LCD
void getRain(){
      bool cambienmua = digital1();
          if(cambienmua){
              LCD_SetCursor(0,2);
              LCD_Print("Sunny");
              LCD_writeChar(8);
          }else{
              LCD_SetCursor(0,2);
              LCD_Print("Rain ");
              LCD_writeChar(6);
          }
          __delay_cycles(100000);

          bool isOpen = isBlindOpen();
          if (isOpen){
              LCD_SetCursor(0,3);
              LCD_Print("Rem mo ra   ");
          }else{
              LCD_SetCursor(0,3);
              LCD_Print("Rem dong lai");
          }
}

void delayMicroseconds(unsigned int us) {
    while (us--) {
        __delay_cycles(1);                  // Assuming 1MHz clock, 1 cycle = 1 microsecond
    }
}

//Hàm tính toán mực nước
unsigned int getDistance() {
    unsigned int duration;
    P1OUT |= TRIG;                          // Set TRIG_PIN high
    delayMicroseconds(10);                  // Wait for 10 microseconds
    P1OUT &= ~TRIG;                         // Set TRIG_PIN low

    while (!(P1IN & ECHO));                 // Wait for ECHO_PIN to go high
    TA0R = 0;                               // Reset timer
    TA0CTL |= MC_2;                         // Start timer in continuous mode

    while (P1IN & ECHO);                    // Wait for ECHO_PIN to go
    TA0CTL &= ~MC_2;                        // Stop timer
    duration = TA0R;                        // Read timer value

    distance = (duration /58);
    Maindistance = Maxdistance - distance;
    return Maindistance;
}

//Hàm điều khiển motor bơm nước
void controlMotor(){
    unsigned int distance = getDistance();

    LCD_SetCursor(0,0);
    LCD_Print("Distance: ");
    lcd_put_num(distance,0,0);
    LCD_Print("cm     ");

    if (distance < 200) {
        P1OUT |= IN1;
        LCD_SetCursor(0,1);
        LCD_Print("Bom Nuoc Vao       ");
        } else {
            P1OUT &= ~IN1;
            LCD_SetCursor(0,1);
            LCD_Print("Khong Bom, Khong Hut");
            }
}

//Hàm điều khiển mở, đóng rèm
void openCurtain() {
    P2OUT |= MOTOR_FORWARD;
    P2OUT &= ~MOTOR_REVERSE;
}
void closeCurtain() {
    P2OUT |= MOTOR_REVERSE;
    P2OUT &= ~MOTOR_FORWARD;
}

//Hàm setup chân
void setup_dc() {
    P1SEL = ECHO + TRIG;
    P1SEL2 = ECHO ;
    P1DIR |= TRIG;
    P1OUT &= ~TRIG;

    P1DIR &= ~ECHO;
    P1SEL &= ~ECHO;

    P1DIR |= IN1 ;
    P1OUT &= ~IN1;

    P2IFG = 0x00;

    P2DIR |= MOTOR_FORWARD + MOTOR_REVERSE;
    P2OUT &= ~(MOTOR_FORWARD + MOTOR_REVERSE);

    P2DIR &= ~(RAIN_SENSOR + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2);
    P2REN |= RAIN_SENSOR + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;
    P2OUT |= RAIN_SENSOR + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;


    P2IE |= RAIN_SENSOR + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;
    P2IES |= RAIN_SENSOR + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;
    P2IFG &= ~(RAIN_SENSOR + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2);

    P2DIR &= ~RAIN_SENSOR;
    P2REN |= RAIN_SENSOR;
    P2OUT |= RAIN_SENSOR;
}

void main(void){

    WDTCTL = WDTPW + WDTHOLD;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
    TA0CCR0 = 0;
    TA0CTL = TASSEL_2 + MC_0;

    setup_dc();

    LCD_Init(0X27, 4, 20);
    LCD_backlightOn();
    LCD_Clear();

    LCD_createChar(8, customChar);
    LCD_createChar(6, customChar2);
    _BIS_SR(GIE);
    while(1){
        controlMotor();
        getRain();
      __delay_cycles(50000);
  }
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void){
    if (P2IFG & RAIN_SENSOR) {
        P2IFG &= ~RAIN_SENSOR;
        if (P2IN & RAIN_SENSOR) {
            openCurtain();
        } else {
            closeCurtain();
        }
    }
    if(P2IFG & LIMIT_SWITCH_PIN1) {
        P2IFG &= ~LIMIT_SWITCH_PIN1;
        if (P2IN & LIMIT_SWITCH_PIN1) {
            P2OUT &= ~(MOTOR_FORWARD + MOTOR_REVERSE);
        }
    }
    if (P2IFG & LIMIT_SWITCH_PIN2) {
        P2IFG &= ~LIMIT_SWITCH_PIN2;
        if (P2IN & LIMIT_SWITCH_PIN2) {
            P2OUT &= ~(MOTOR_FORWARD + MOTOR_REVERSE);
        }
    }
}
