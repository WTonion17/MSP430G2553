#include "msp430.h"
#include "lcd.h"
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h>

#define TRIG BIT0                               //P1.0
#define ECHO BIT1                               //P1.1

#define IN1 BIT3                                //P1.2

#define MOTOR_FORWARD BIT0                      //P2.0
#define MOTOR_REVERSE BIT1                      //P2.1

#define RAIN_SENSOR BIT2                        //P2.2

#define LIMIT_SWITCH_PIN1 BIT3                  //P2.3
#define LIMIT_SWITCH_PIN2 BIT4                  //P2.4


unsigned int distance;
unsigned int Maindistance;
unsigned int Maxdistance = 205;

enum MotorState { STOPPED, FORWARD, REVERSE };
volatile enum MotorState motorState = STOPPED;

bool trangthairem = false;
bool digital = false;

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

bool isBlindOpen() {                        // Hàm kiem tra trang thái cua rèm
    if (P2IN & RAIN_SENSOR) {               // Neu chan cam bien rem dong, tra ve false (rèm dóng)
        return true;
    } else {
        return false;
    }
}

bool digital1() {                           // Hàm kiem tra trang thái cua cambien
    if (P2IN &  RAIN_SENSOR) {              // Neu chan cam bien co nuoc, tra ve false (mua)
        return true;
    } else {
        return false;
    }
}

void getRain(){
      bool cambienmua = digital1();
          if(cambienmua){
              LCD_SetCursor(0,2);           //set vi tri lcd(cot, hang)
              LCD_Print("Sunny");           //in chuoi ra mang hinh
              LCD_writeChar(8);             //in char ra man hinh
          }else{
              LCD_SetCursor(0,2);           //set vi tri lcd(cot, hang)
              LCD_Print("Rain ");
              LCD_writeChar(6);             //in char ra man hinh
          }
          __delay_cycles(100000);

          bool isOpen = isBlindOpen();      // dieu khien rem
          if (isOpen){
              LCD_SetCursor(0,3);           //set vi tri lcd(cot, hang)
              LCD_Print("Rem mo ra   ");    //in chuoi ra mang hinh
          }else{
              LCD_SetCursor(0,3);           //set vi tri lcd(cot, hang)
              LCD_Print("Rem dong lai");
          }
}


void delayMicroseconds(unsigned int us) {
    while (us--) {
        __delay_cycles(1);                  // Assuming 1MHz clock, 1 cycle = 1 microsecond
    }
}


unsigned int getDistance() {
    unsigned int duration;
    P1OUT |= TRIG;                          // Set TRIG_PIN high
    delayMicroseconds(10);                  // Wait for 10 microseconds
    P1OUT &= ~TRIG;                         // Set TRIG_PIN low

    while (!(P1IN & ECHO));                 // Wait for ECHO_PIN to go high
    TA0R = 0;                               // Reset timer
    TA0CTL |= MC_2;                         // Start timer in continuous mode

    while (P1IN & ECHO);                    // Wait for ECHO_PIN to go lo
    TA0CTL &= ~MC_2;                        // Stop timer
    duration = TA0R;                        // Read timer value
  
    distance = (duration /58);
    Maindistance = Maxdistance - distance;
    return Maindistance;
}

void controlMotol(){
    unsigned int distance = getDistance();

    LCD_SetCursor(0,0);                      //set vi tri lcd(cot, hang)
    LCD_Print("Distance: ");                 //in chuoi ra mang hinh
    lcd_put_num(distance,0,0);
    LCD_Print("cm ");

      // Điều khiển động cơ bơm nước
    if (distance < 200) {
        P1OUT |= IN1;                        // Turn on motor
        LCD_SetCursor(0,1);                  //set vi tri lcd(cot, hang)
        LCD_Print("Bom Nuoc Vao       ");    //in chuoi ra mang hinh
        } else {
            P1OUT &= ~IN1;                   // Turn off motor
            LCD_SetCursor(0,1);              //set vi tri lcd(cot, hang)
            LCD_Print("Khong Bom,Khong Hut");//in chuoi ra mang hinh
            }
}


void setup() {
    P1SEL = ECHO + TRIG;
    P1SEL2 = ECHO ;
    P1DIR |= TRIG;                            // Set TRIG_PIN là output
    P1OUT &= ~TRIG;                           // Set TRIG_PIN thành mức thấp

    P1DIR &= ~ECHO;                           // Set ECHO_PIN là input
    P1SEL &= ~ECHO;                           // Set chân GPIO cho chân ECHO_PIN

    P1DIR |= IN1 ;                            //cấu hình các chân động cơ là OUTPUT
    P1OUT &= ~IN1;                            // Khởi tạo các chân điều khiển động cơ ở mức thấp

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
    TA0CCR0 = 0;                              // Initialize CCR0
    TA0CTL = TASSEL_2 + MC_0;                 // SMCLK, Stop mode
    setup();
    LCD_Init(0X27, 4, 20);                    //khoi tao LCD voi giao thuc i2c
    LCD_backlightOn();                        //cho phep bat den nen
    LCD_Clear();                              //clear mang hinh de xoa ky tu vo dinh

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
