#include "msp430.h"
#include "lcd.h"
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h>

#define TRIG BIT0 //P1.0
#define ECHO BIT1 //P1.1

#define IN1 BIT2 //P1.2
#define IN2 BIT3 //P1.3
#define IN3 BIT4 //P1.4
#define IN4 BIT5 //P1.5

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

void getRain(){
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
}

unsigned int getDistance(){

    // Gửi xung TRIG
    P1IE &= ~TRIG;
    P1DIR |= TRIG;
    P1OUT |= TRIG;
    __delay_cycles(10);
    P1OUT &= ~TRIG;
    P1DIR &= ~ECHO;
    P1IFG = 0x00;
    P1IE |= ECHO;
    P1IES &= ~ECHO;
    __delay_cycles(30000);
    distance = sensor/58;
    return distance;
}

void controlMotor(){
    unsigned int distance = getDistance();

    LCD_SetCursor(0,0);//set vi tri lcd(cot, hang)
    LCD_Print("Distance: ");//in chuoi ra mang hinh
    lcd_put_num(distance,0,0);
    LCD_Print("cm ");

    // Điều khiển động cơ bơm nước
    if (distance < 10) { // Nếu khoảng cách đo được nhỏ hơn 10 cm
        P1OUT |= IN1;  // Bật động cơ bơm nước
        P1OUT &= ~IN2; // Đảm bảo động cơ không quay ngược
        LCD_SetCursor(0,1);//set vi tri lcd(cot, hang)
        LCD_Print("Bom Nuoc Vao");//in chuoi ra mang hinh
        } else {
            P1OUT &= ~IN1; // Tắt động cơ bơm nước
            }

    // Điều khiển động cơ hút nước
    if (distance  > 13) { // Nếu khoảng cách đo được lớn hơn 13 cm
        P1OUT |= IN3;  // Bật động cơ hút nước
        P1OUT &= ~IN4; // Đảm bảo động cơ không quay ngược
        LCD_SetCursor(0,1);//set vi tri lcd(cot, hang)
        LCD_Print("Hut Nuoc Ra");//in chuoi ra mang hinh
        } else {
            P1OUT &= ~IN3; // Tắt động cơ hút nước
            }

    // Động cơ không hoạt động
    if (distance >= 10 && distance <= 13){ // Nếu khoảng cách đo được từ 10 - 13 cm
        P1OUT &= ~(IN1 + IN2 + IN3 + IN4); //Tắt hết động cơ
        LCD_SetCursor(0,1);//set vi tri lcd(cot, hang)
        LCD_Print("Khong Bom,Khong Hut");//in chuoi ra mang hinh
        }
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

    P1DIR |= (IN1 + IN2 + IN3 + IN4); //cấu hình các chân động cơ là OUTPUT
    P1OUT &= ~(IN1 + IN2 + IN3 + IN4); // Khởi tạo các chân điều khiển động cơ ở mức thấp

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

  setup();

  LCD_Init(0X27, 4, 20);//khoi tao LCD voi giao thuc i2c
  LCD_backlightOn();//cho phep bat den nen
  LCD_Clear();//clear mang hinh de xoa ky tu vo dinh

  LCD_createChar(8, customChar);
  LCD_createChar(6, customChar2);

  _BIS_SR(GIE);
  while(1){
      controlMotor();
      getRain();
      __no_operation();
      __delay_cycles(100000);
  }
}

#pragma vector = PORT1_VECTOR
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

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void){
    milisecond++;
}











