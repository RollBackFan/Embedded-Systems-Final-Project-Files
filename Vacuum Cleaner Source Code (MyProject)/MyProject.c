// Ultrasonic Sensor Pins
#define TRIG_FRONT PORTB.F0
#define ECHO_FRONT PORTB.F1
#define TRIG_LEFT PORTB.F2
#define ECHO_LEFT PORTB.F3
#define TRIG_RIGHT PORTB.F4
#define ECHO_RIGHT PORTB.F5

// Motor Control Pins
#define LEFT_MOTOR_FORWARD PORTC.F4
#define LEFT_MOTOR_BACKWARD PORTC.F5
#define RIGHT_MOTOR_FORWARD PORTC.F6
#define RIGHT_MOTOR_BACKWARD PORTC.F7

// Servo Motor Pin
#define SERVO_PIN PORTC.F2

// H-Bridge Control Pins for Fan
#define FAN_FORWARD PORTD.F1
#define FAN_BACKWARD PORTD.F2

// Switch Pin
#define SWITCH_PIN PORTD.F3
 #define IR_PIN PORTD.F4
unsigned int distance_front, distance_left, distance_right;
void setupPWM() {
    TRISC.F2 = 0;           // Set RC2 as output
    CCP1CON = 0x0C;         // Configure CCP1 module in PWM mode
    T2CON = 0x04;           // Enable Timer2 with prescaler = 1
    PR2 = 249;              // Set PWM frequency to 50 Hz (20 ms period)
    CCPR1L = 0;             // Initialize duty cycle to 0
 //   TMR2ON_bit = 1;         // Start Timer2
}
void Initialize() {

 setupPWM();
    // Initialize Ultrasonic Sensor Pins
    TRISB = 0x3E; // Set RB0, RB2, RB4 as outputs (TRIG), and RB1, RB3, RB5 as inputs (ECHO)

    // Initialize Motor Control Pins
    TRISC = 0x00; // Set PORTC as output
    PORTC = 0x00; // Initialize PORTC to 0

    // Initialize Servo Motor Pin
    TRISD.F0 = 0; // Set RD0 as output

    // Initialize H-Bridge Control Pins for Fan
    TRISD.F1 = 0; // Set RD1 as output
    TRISD.F2 = 0; // Set RD2 as output

    // Initialize Switch Pin
    TRISD.F3 = 1; // Set RD3 as input
    TRISD.F4 = 1; // Set RD3 as input
    // Initialize Timer0 for PWM
    OPTION_REG = 0x07; // Timer0 with 1:256 prescaler

    // Initialize Timer1 for delay functions
    T1CON = 0x10; // Timer1 with 1:2 prescaler, T1OSCEN = 1 (oscillator enabled), and the timer is off

}

void Delay_us(unsigned int us) {
    while (us--) {
        TMR1H = 0;  // Clear high byte of Timer1
        TMR1L = 0;  // Clear low byte of Timer1
        T1CON.F0 = 1;  // Start Timer1
        while (TMR1L < 4);  // Wait until the timer overflows (1 us delay for 4 MHz clock)
        T1CON.F0 = 0;  // Stop Timer1
    }
}

void Delay_ms(unsigned int ms) {
    while (ms--) {
        Delay_us(1000); // 1 ms delay
    }
}

unsigned int ReadUltrasonicSensor() {
    unsigned int time;

    // Send trigger pulse
    TRIG_FRONT = 1;
    Delay_us(10);
    TRIG_FRONT = 0;

    // Wait for echo pulse
    while (!ECHO_FRONT);
    TMR1H = 0;  // Clear high byte of Timer1
    TMR1L = 0;  // Clear low byte of Timer1
    T1CON.F0 = 1;  // Start Timer1
    while (ECHO_FRONT);
    time = (TMR1H << 8) | TMR1L;  // Read Timer1 value
    T1CON.F0 = 0;  // Stop Timer1

    // Calculate distance in cm
    return (time /13.51);
}

void MoveForward() {
    LEFT_MOTOR_FORWARD = 1;
    LEFT_MOTOR_BACKWARD = 0;
    RIGHT_MOTOR_FORWARD = 1;
    RIGHT_MOTOR_BACKWARD = 0;
}

void MoveBackward() {
    LEFT_MOTOR_FORWARD = 0;
    LEFT_MOTOR_BACKWARD = 1;
    RIGHT_MOTOR_FORWARD = 0;
    RIGHT_MOTOR_BACKWARD = 1;
}

void TurnLeft() {
    LEFT_MOTOR_FORWARD = 0;
    LEFT_MOTOR_BACKWARD = 1;
    RIGHT_MOTOR_FORWARD = 1;
    RIGHT_MOTOR_BACKWARD = 0;
}

void TurnRight() {
    LEFT_MOTOR_FORWARD = 1;
    LEFT_MOTOR_BACKWARD = 0;
    RIGHT_MOTOR_FORWARD = 0;
    RIGHT_MOTOR_BACKWARD = 1;
}

void Stop() {
    LEFT_MOTOR_FORWARD = 0;
    LEFT_MOTOR_BACKWARD = 0;
    RIGHT_MOTOR_FORWARD = 0;
    RIGHT_MOTOR_BACKWARD = 0;

}
void moveServo(int angle) {

    unsigned int duty;

    // Convert angle to duty cycle
    // 0 degrees = 1 ms pulse (5% duty), 90 degrees = 1.5 ms (7.5% duty)
    // Duty cycle = (PulseWidth / Period) * 1024
    duty = ((1000 + (angle * 1000 / 180)) * 1024) / 20000;

    CCPR1L = duty >> 2;      // Set upper 8 bits of duty cycle
    CCP1CON &= 0xCF;         // Clear lower 2 bits
    CCP1CON |= (duty & 0x03) << 4;  // Set lower 2 bits of duty cycle
}

void RaiseFlag() {
       //   moveServoRC2(90);    // Move servo to 0 degrees
   moveServo(90);
   Delay_ms(1000);
  moveServo(0);
  Delay_ms(1000);}
   /*void RaiseFlag() { // PWM signal generation for 1ms to 2ms pulse width
unsigned int i; for (i = 0; i < 50; i++) {
// 50 cycles for 20ms period (50Hz)
SERVO_PIN = 1; // Start pulse
Delay_us(1500); // Adjust the pulse width (1500us for ~90 degrees)
 SERVO_PIN = 0; // End pulse
  Delay_us(18500); // Complete 20ms period
   }
   } */

void ControlFan() {
    if (SWITCH_PIN) {
        FAN_FORWARD = 1;
        FAN_BACKWARD = 0;
    } else {
        FAN_FORWARD = 0;
        FAN_BACKWARD = 0;
    }
}

void main() {
    Initialize();

   RaiseFlag(); // Raise the flag before starting

      while(IR_PIN){
       ControlFan();
       distance_front = ReadUltrasonicSensor();

        if (distance_front < 20) { // Distance threshold in cm
           Stop();
            Delay_ms(500); // Pause for 500 ms
            MoveBackward();
            Delay_ms(200); // Move backward for 500 ms
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
              /*    FAN_FORWARD = 1;
                Delay_ms(500);
              FAN_FORWARD = 0;
                Delay_ms(500);          */

            }
           Stop();
        FAN_FORWARD = 0;
        FAN_BACKWARD = 0;

}
