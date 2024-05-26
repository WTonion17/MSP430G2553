#include "msp430.h"
//#include "lcd.h"
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h>

#define TRIG BIT0 //P1.0
#define ECHO BIT1 //P1.1

#define IN1 BIT2 //P1.2

#define IN3 BIT4 //P1.4


#define MOTOR_FORWARD BIT0 //P2.0
#define MOTOR_REVERSE BIT1 //P2.1

#define RAIN_SENSOR BIT2 //P2.2

#define LIMIT_SWITCH_PIN1 BIT3 //P2.3
#define LIMIT_SWITCH_PIN2 BIT4 //P2.4

unsigned int milisecond;
unsigned int distance;
unsigned int sensor;

enum MotorState { STOPPED, FORWARD, REVERSE };
volatile enum MotorState motorState = STOPPED;

bool trangthairem = false;
bool digital = false;

volatile unsigned int timer_count = 0;
void show();

void configureTimer();
void delayMicroseconds(unsigned int us);
unsigned int measureDistance();

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

// Hàm kiem tra trang thái cua rèm
bool isBlindOpen() {
    // Neu chan cam bien rem dong, tra ve false (rèm dóng)
    if (P2IN & RAIN_SENSOR) {
        return true;
    } else {
        return false;
    }
}

// Hàm kiem tra trang thái cua cambien
bool digital1() {
    // Neu chan cam bien co nuoc, tra ve false (mua)
    if (P2IN &  RAIN_SENSOR) {
        return true;
    } else {
        return false;
    }
}

/*void getRain(){
      bool cambienmua = digital1();
          if(cambienmua){
              LCD_SetCursor(0,2);//set vi tri lcd(cot, hang)
              LCD_Print("Sunny");//in chuoi ra mang hinh
              LCD_writeChar(8); //in char ra man hinh
          }else{
              LCD_SetCursor(0,2);//set vi tri lcd(cot, hang)
              LCD_Print("Rain ");
              LCD_writeChar(6); //in char ra man hinh
          }
          __delay_cycles(100000);

          // dieu khien rem
          bool isOpen = isBlindOpen();
          if (isOpen){
              LCD_SetCursor(0,3);//set vi tri lcd(cot, hang)
              LCD_Print("Rem mo ra");//in chuoi ra mang hinh
          }else{
              LCD_SetCursor(0,3);//set vi tri lcd(cot, hang)
              LCD_Print("Rem dong lai");
          }
}*/


void delayMicroseconds(unsigned int us) {
    while (us--) {
        __delay_cycles(1); // Assuming 1MHz clock, 1 cycle = 1 microsecond
    }
}

void configureTimer() {
    TA0CCTL0 = CCIE;          // Enable interrupt for CCR0
    TA0CCR0 = 0;              // Initialize CCR0
    TA0CTL = TASSEL_2 + MC_0; // SMCLK, Stop mode
}

unsigned int getDistance() {
    unsigned int duration = 0;

        P1OUT |= TRIG;   // Set TRIG_PIN high
        delayMicroseconds(10); // Wait for 10 microseconds
        P1OUT &= ~TRIG;  // Set TRIG_PIN low

        while (!(P1IN & ECHO)); // Wait for ECHO_PIN to go high

        TA0R = 0;                 // Reset timer
        TA0CTL |= MC_2;           // Start timer in continuous mode

        while (P1IN & ECHO);  // Wait for ECHO_PIN to go low
TA0CTL &= ~MC_2;          // Stop timer
        duration = TA0R;          // Read timer value

        // Calculate distance in cm (speed of sound is 343 m/s)
        unsigned int distance = (duration /58);

        return distance;
}



void setup() {

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



    //__bis_SR_register(GIE);
}

void main(void)
{
 WDTCTL = WDTPW + WDTHOLD;

  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
  /*CCTL0 = CCIE;
  CCR0 = 1000;
  TACTL = TASSEL_2 +MC_1;*/
  P2IFG = 0x00;

  P1DIR |= TRIG;          // Set TRIG_PIN as output
      P1OUT &= ~TRIG;         // Set TRIG_PIN to low

      P1DIR &= ~ECHO;         // Set ECHO_PIN as input
      P1SEL &= ~ECHO;         // Select GPIO function for ECHO_PIN

      P1DIR |= (IN1 + IN3); //cấu hình các chân động cơ là OUTPUT
      P1OUT &= ~(IN1 + IN3); // Khởi tạo các chân điều khiển động cơ ở mức thấp
  setup();

      configureTimer();
  /*LCD_Init(0X27, 4, 20);//khoi tao LCD voi giao thuc i2c
  LCD_backlightOn();//cho phep bat den nen
  LCD_Clear();//clear mang hinh de xoa ky tu vo dinh

  LCD_createChar(8, customChar);
  LCD_createChar(6, customChar2);*/

  _BIS_SR(GIE);
  while(1){
      unsigned int distance = getDistance();

          /*LCD_SetCursor(0,0);//set vi tri lcd(cot, hang)
          LCD_Print("Distance: ");//in chuoi ra mang hinh
          lcd_put_num(distance,0,0);
          LCD_Print("cm ");*/

          // Điều khiển động cơ bơm nước
          if (distance < 10) {
                     P1OUT |= IN1;    // Turn on motor 1
                     P1OUT &= ~IN3;   // Turn off motor 2
                 } else
                   if (distance > 15) {
                     P1OUT |= IN3;    // Turn on motor 2
                     P1OUT &= ~IN1;   // Turn off motor 1
                 } else {
                     P1OUT &= ~(IN1 + IN3);  // Turn off both motors
                 }

      //getRain();

      __delay_cycles(50000);
  }
}

/*#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void){
    if(P1IFG & ECHO){
        if(!(P1IES & ECHO)){
            TACTL |= TACLR;
            milisecond = 0;
            P1IES |= ECHO;
        }else {
            sensor = (long)milisecond*1000 + (long)TAR;
        }
        P1IFG &= ~ECHO;
    }
}*/

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void){
    if (P2IFG & RAIN_SENSOR) {
        if (motorState == FORWARD) {
            motorState = REVERSE;
P2OUT &= ~MOTOR_FORWARD;
            P2OUT |= MOTOR_REVERSE;
        } else if (motorState == REVERSE) {
            motorState = FORWARD;
            P2OUT &= ~MOTOR_REVERSE;
            P2OUT |= MOTOR_FORWARD;
        } else {
            motorState = FORWARD;
            P2OUT |= MOTOR_FORWARD;
        }
        P2IFG &= ~RAIN_SENSOR;
    }

    if(P2IFG & LIMIT_SWITCH_PIN1) {
        P2OUT &= ~(MOTOR_FORWARD + MOTOR_REVERSE);
        motorState = STOPPED;
        P2IFG &= ~LIMIT_SWITCH_PIN1;
    }
    if (P2IFG & LIMIT_SWITCH_PIN2) {
        P2OUT &= ~(MOTOR_FORWARD + MOTOR_REVERSE);
        motorState = STOPPED;
        P2IFG &= ~LIMIT_SWITCH_PIN2;
    }
}
