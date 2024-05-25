#include "msp430.h"
#include "lcd.h"
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h>

#define MOTOR_FORWARD BIT0
#define MOTOR_REVERSE BIT1
#define SENSOR_PIN BIT2
#define LIMIT_SWITCH_PIN1 BIT3
#define LIMIT_SWITCH_PIN2 BIT4

enum MotorState { STOPPED, FORWARD, REVERSE };
volatile enum MotorState motorState = STOPPED;

int milisecond;
int distance;
int sensor;

bool trangthairem = false;
bool digital = false;

volatile unsigned int timer_count = 0;
void show();

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
/*bool isBlindOpen() {
    // Neu chan cam bien rem dong, tra ve false (rèm dóng)
    if (P1IN & REM_SENSOR_PIN) {
        return false;
    } else {
        return true;
    }
}*/

// Hàm kiem tra trang thái cua cambien
bool digital1() {
    // Neu chan cam bien co nuoc, tra ve false (mua)
    if (P2IN &  SENSOR_PIN) {
        return true;
    } else {
        return false;
    }
}

void getRain(){
      P2DIR &= ~SENSOR_PIN;
      P2REN |= SENSOR_PIN;
      P2OUT |= SENSOR_PIN;
      P2SEL &=~ (BIT6 + BIT7); //Tat dao dong ngoai o chan P2.6 va P2.7 de su dung chan GPIO cho LCD

      //Cau hình chân GPIO ket noi thiet bi dieu khien rèm
      /*P1DIR &= ~REM_SENSOR_PIN; // Ðat chân là input
      P1REN |= REM_SENSOR_PIN;  // Bat resistor pull-up
      P1OUT |= REM_SENSOR_PIN;  // Ðat resistor pull-up*/

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
          /*bool isOpen = isBlindOpen();
          if (isOpen){
              LCD_SetCursor(0,3);//set vi tri lcd(cot, hang)
              LCD_Print("Rem mo ra");//in chuoi ra mang hinh
          }else{
              LCD_SetCursor(0,3);//set vi tri lcd(cot, hang)
              LCD_Print("Rem dong lai");
          }*/
}

void getDistance(){
    P1IE &= ~BIT0;
    P1DIR |= BIT0;
    P1OUT |= BIT0;
    __delay_cycles(10);
    P1OUT &= ~BIT0;
    P1DIR &= ~BIT1;
    P1IFG = 0x00;
    P1IE |= BIT1;
    P1IES &= ~BIT1;
    __delay_cycles(30000);
    distance = sensor/58;
    if(distance < 50 && distance != 0){
        P1OUT &= ~(BIT2 + BIT3 + BIT4);
        P1OUT |= BIT5;
        LCD_SetCursor(0,1);//set vi tri lcd (cot, hang)
        LCD_Print("bom nuoc vao");
    }else if(distance > 60){
        P1OUT &= ~(BIT2 + BIT4 + BIT5);
        P1OUT |= BIT3;
        LCD_SetCursor(0,1);//set vi tri lcd (cot, hang)
        LCD_Print("hut nuoc ra ");
    }
    else P1OUT &= ~(BIT2 + BIT3 + BIT4 + BIT5);
    LCD_SetCursor(0,0);//set vi tri lcd(cot, hang)
    LCD_Print("Distance: ");//in chuoi ra mang hinh
    lcd_put_num(distance,0,0);
    LCD_Print("cm ");
}

void setup() {


    P2DIR |= MOTOR_FORWARD + MOTOR_REVERSE;
    P2OUT &= ~(MOTOR_FORWARD + MOTOR_REVERSE);

    P2DIR &= ~(SENSOR_PIN + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2);
    P2REN |= SENSOR_PIN + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;
    P2OUT |= SENSOR_PIN + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;


    P2IE |= SENSOR_PIN + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;
    P2IES |= SENSOR_PIN + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2;
    P2IFG &= ~(SENSOR_PIN + LIMIT_SWITCH_PIN1 + LIMIT_SWITCH_PIN2);

    //__bis_SR_register(GIE);
}

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;

  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
  CCTL0 = CCIE;
  CCR0 = 1000;
  TACTL = TASSEL_2 +MC_1;
  P1IFG = 0x00;
  P1DIR |= (BIT2 + BIT3 + BIT4 + BIT5);
  P1OUT &= ~(BIT2 + BIT3 + BIT4 + BIT5);

  LCD_Init(0X27, 4, 20);//khoi tao LCD voi giao thuc i2c
  LCD_backlightOn();//cho phep bat den nen
  LCD_Clear();//clear mang hinh de xoa ky tu vo dinh

  LCD_createChar(8, customChar);
  LCD_createChar(6, customChar2);

  setup();

  _BIS_SR(GIE);
  while(1){
      getDistance();
      getRain();
      __no_operation();
      __delay_cycles(10000);
  }
}

#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void){
    if(P1IFG & BIT1){
        if(!(P1IES & BIT1)){
            TACTL |= TACLR;
            milisecond = 0;
            P1IES |= BIT1;
        }else {
            sensor = (long)milisecond*1000 + (long)TAR;
        }
        P1IFG &= ~BIT1;
    }

}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void){
    if (P2IFG & SENSOR_PIN) {

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
                P2IFG &= ~SENSOR_PIN;
            }

            if (P2IFG & LIMIT_SWITCH_PIN1) {

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

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void){
    milisecond++;
}











