#line 1 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Examples/MyProject.c"
#line 25 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Examples/MyProject.c"
unsigned int distance_front, distance_left, distance_right;
void setupPWM() {
 TRISC.F2 = 0;
 CCP1CON = 0x0C;
 T2CON = 0x04;
 PR2 = 249;
 CCPR1L = 0;

}
void Initialize() {

 setupPWM();

 TRISB = 0x3E;


 TRISC = 0x00;
 PORTC = 0x00;


 TRISD.F0 = 0;


 TRISD.F1 = 0;
 TRISD.F2 = 0;


 TRISD.F3 = 1;
 TRISD.F4 = 1;

 OPTION_REG = 0x07;


 T1CON = 0x10;

}

void Delay_us(unsigned int us) {
 while (us--) {
 TMR1H = 0;
 TMR1L = 0;
 T1CON.F0 = 1;
 while (TMR1L < 4);
 T1CON.F0 = 0;
 }
}

void Delay_ms(unsigned int ms) {
 while (ms--) {
 Delay_us(1000);
 }
}

unsigned int ReadUltrasonicSensor() {
 unsigned int time;


  PORTB.F0  = 1;
 Delay_us(10);
  PORTB.F0  = 0;


 while (! PORTB.F1 );
 TMR1H = 0;
 TMR1L = 0;
 T1CON.F0 = 1;
 while ( PORTB.F1 );
 time = (TMR1H << 8) | TMR1L;
 T1CON.F0 = 0;


 return (time /13.51);
}

void MoveForward() {
  PORTC.F4  = 1;
  PORTC.F5  = 0;
  PORTC.F6  = 1;
  PORTC.F7  = 0;
}

void MoveBackward() {
  PORTC.F4  = 0;
  PORTC.F5  = 1;
  PORTC.F6  = 0;
  PORTC.F7  = 1;
}

void TurnLeft() {
  PORTC.F4  = 0;
  PORTC.F5  = 1;
  PORTC.F6  = 1;
  PORTC.F7  = 0;
}

void TurnRight() {
  PORTC.F4  = 1;
  PORTC.F5  = 0;
  PORTC.F6  = 0;
  PORTC.F7  = 1;
}

void Stop() {
  PORTC.F4  = 0;
  PORTC.F5  = 0;
  PORTC.F6  = 0;
  PORTC.F7  = 0;

}
void moveServo(int angle) {

 unsigned int duty;




 duty = ((1000 + (angle * 1000 / 180)) * 1024) / 20000;

 CCPR1L = duty >> 2;
 CCP1CON &= 0xCF;
 CCP1CON |= (duty & 0x03) << 4;
}

void RaiseFlag() {

 moveServo(90);
 Delay_ms(1000);
 moveServo(0);
 Delay_ms(1000);}
#line 164 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Examples/MyProject.c"
void ControlFan() {
 if ( PORTD.F3 ) {
  PORTD.F1  = 1;
  PORTD.F2  = 0;
 } else {
  PORTD.F1  = 0;
  PORTD.F2  = 0;
 }
}

void main() {
 Initialize();

 RaiseFlag();

 while( PORTD.F4 ){
 ControlFan();
 distance_front = ReadUltrasonicSensor();

 if (distance_front < 20) {
 Stop();
 Delay_ms(500);
 MoveBackward();
 Delay_ms(200);
 Stop();
 Delay_ms(500);
 TurnLeft();
 Delay_ms(350);
 Stop();
 Delay_ms(1000);
 }
 else {
 MoveForward();
 Delay_ms(500);

 }
#line 205 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Examples/MyProject.c"
 }
 Stop();
  PORTD.F1  = 0;
  PORTD.F2  = 0;

}
