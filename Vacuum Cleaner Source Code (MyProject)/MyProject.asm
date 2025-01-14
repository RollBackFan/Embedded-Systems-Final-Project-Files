
_setupPWM:

;MyProject.c,26 :: 		void setupPWM() {
;MyProject.c,27 :: 		TRISC.F2 = 0;           // Set RC2 as output
	BCF        TRISC+0, 2
;MyProject.c,28 :: 		CCP1CON = 0x0C;         // Configure CCP1 module in PWM mode
	MOVLW      12
	MOVWF      CCP1CON+0
;MyProject.c,29 :: 		T2CON = 0x04;           // Enable Timer2 with prescaler = 1
	MOVLW      4
	MOVWF      T2CON+0
;MyProject.c,30 :: 		PR2 = 249;              // Set PWM frequency to 50 Hz (20 ms period)
	MOVLW      249
	MOVWF      PR2+0
;MyProject.c,31 :: 		CCPR1L = 0;             // Initialize duty cycle to 0
	CLRF       CCPR1L+0
;MyProject.c,33 :: 		}
L_end_setupPWM:
	RETURN
; end of _setupPWM

_Initialize:

;MyProject.c,34 :: 		void Initialize() {
;MyProject.c,36 :: 		setupPWM();
	CALL       _setupPWM+0
;MyProject.c,38 :: 		TRISB = 0x3E; // Set RB0, RB2, RB4 as outputs (TRIG), and RB1, RB3, RB5 as inputs (ECHO)
	MOVLW      62
	MOVWF      TRISB+0
;MyProject.c,41 :: 		TRISC = 0x00; // Set PORTC as output
	CLRF       TRISC+0
;MyProject.c,42 :: 		PORTC = 0x00; // Initialize PORTC to 0
	CLRF       PORTC+0
;MyProject.c,45 :: 		TRISD.F0 = 0; // Set RD0 as output
	BCF        TRISD+0, 0
;MyProject.c,48 :: 		TRISD.F1 = 0; // Set RD1 as output
	BCF        TRISD+0, 1
;MyProject.c,49 :: 		TRISD.F2 = 0; // Set RD2 as output
	BCF        TRISD+0, 2
;MyProject.c,52 :: 		TRISD.F3 = 1; // Set RD3 as input
	BSF        TRISD+0, 3
;MyProject.c,53 :: 		TRISD.F4 = 1; // Set RD3 as input
	BSF        TRISD+0, 4
;MyProject.c,55 :: 		OPTION_REG = 0x07; // Timer0 with 1:256 prescaler
	MOVLW      7
	MOVWF      OPTION_REG+0
;MyProject.c,58 :: 		T1CON = 0x10; // Timer1 with 1:2 prescaler, T1OSCEN = 1 (oscillator enabled), and the timer is off
	MOVLW      16
	MOVWF      T1CON+0
;MyProject.c,60 :: 		}
L_end_Initialize:
	RETURN
; end of _Initialize

_Delay_us:

;MyProject.c,62 :: 		void Delay_us(unsigned int us) {
;MyProject.c,63 :: 		while (us--) {
L_Delay_us0:
	MOVF       FARG_Delay_us_us+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay_us_us+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay_us_us+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay_us_us+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay_us1
;MyProject.c,64 :: 		TMR1H = 0;  // Clear high byte of Timer1
	CLRF       TMR1H+0
;MyProject.c,65 :: 		TMR1L = 0;  // Clear low byte of Timer1
	CLRF       TMR1L+0
;MyProject.c,66 :: 		T1CON.F0 = 1;  // Start Timer1
	BSF        T1CON+0, 0
;MyProject.c,67 :: 		while (TMR1L < 4);  // Wait until the timer overflows (1 us delay for 4 MHz clock)
L_Delay_us2:
	MOVLW      4
	SUBWF      TMR1L+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_Delay_us3
	GOTO       L_Delay_us2
L_Delay_us3:
;MyProject.c,68 :: 		T1CON.F0 = 0;  // Stop Timer1
	BCF        T1CON+0, 0
;MyProject.c,69 :: 		}
	GOTO       L_Delay_us0
L_Delay_us1:
;MyProject.c,70 :: 		}
L_end_Delay_us:
	RETURN
; end of _Delay_us

_Delay_ms:

;MyProject.c,72 :: 		void Delay_ms(unsigned int ms) {
;MyProject.c,73 :: 		while (ms--) {
L_Delay_ms4:
	MOVF       FARG_Delay_ms_ms+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay_ms_ms+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay_ms_ms+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay_ms_ms+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay_ms5
;MyProject.c,74 :: 		Delay_us(1000); // 1 ms delay
	MOVLW      2
	MOVWF      R12+0
	MOVLW      75
	MOVWF      R13+0
L_Delay_ms6:
	DECFSZ     R13+0, 1
	GOTO       L_Delay_ms6
	DECFSZ     R12+0, 1
	GOTO       L_Delay_ms6
;MyProject.c,75 :: 		}
	GOTO       L_Delay_ms4
L_Delay_ms5:
;MyProject.c,76 :: 		}
L_end_Delay_ms:
	RETURN
; end of _Delay_ms

_ReadUltrasonicSensor:

;MyProject.c,78 :: 		unsigned int ReadUltrasonicSensor() {
;MyProject.c,82 :: 		TRIG_FRONT = 1;
	BSF        PORTB+0, 0
;MyProject.c,83 :: 		Delay_us(10);
	MOVLW      3
	MOVWF      R13+0
L_ReadUltrasonicSensor7:
	DECFSZ     R13+0, 1
	GOTO       L_ReadUltrasonicSensor7
;MyProject.c,84 :: 		TRIG_FRONT = 0;
	BCF        PORTB+0, 0
;MyProject.c,87 :: 		while (!ECHO_FRONT);
L_ReadUltrasonicSensor8:
	BTFSC      PORTB+0, 1
	GOTO       L_ReadUltrasonicSensor9
	GOTO       L_ReadUltrasonicSensor8
L_ReadUltrasonicSensor9:
;MyProject.c,88 :: 		TMR1H = 0;  // Clear high byte of Timer1
	CLRF       TMR1H+0
;MyProject.c,89 :: 		TMR1L = 0;  // Clear low byte of Timer1
	CLRF       TMR1L+0
;MyProject.c,90 :: 		T1CON.F0 = 1;  // Start Timer1
	BSF        T1CON+0, 0
;MyProject.c,91 :: 		while (ECHO_FRONT);
L_ReadUltrasonicSensor10:
	BTFSS      PORTB+0, 1
	GOTO       L_ReadUltrasonicSensor11
	GOTO       L_ReadUltrasonicSensor10
L_ReadUltrasonicSensor11:
;MyProject.c,92 :: 		time = (TMR1H << 8) | TMR1L;  // Read Timer1 value
	MOVF       TMR1H+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       TMR1L+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;MyProject.c,93 :: 		T1CON.F0 = 0;  // Stop Timer1
	BCF        T1CON+0, 0
;MyProject.c,96 :: 		return (time /13.51);
	CALL       _word2double+0
	MOVLW      246
	MOVWF      R4+0
	MOVLW      40
	MOVWF      R4+1
	MOVLW      88
	MOVWF      R4+2
	MOVLW      130
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	CALL       _double2word+0
;MyProject.c,97 :: 		}
L_end_ReadUltrasonicSensor:
	RETURN
; end of _ReadUltrasonicSensor

_MoveForward:

;MyProject.c,99 :: 		void MoveForward() {
;MyProject.c,100 :: 		LEFT_MOTOR_FORWARD = 1;
	BSF        PORTC+0, 4
;MyProject.c,101 :: 		LEFT_MOTOR_BACKWARD = 0;
	BCF        PORTC+0, 5
;MyProject.c,102 :: 		RIGHT_MOTOR_FORWARD = 1;
	BSF        PORTC+0, 6
;MyProject.c,103 :: 		RIGHT_MOTOR_BACKWARD = 0;
	BCF        PORTC+0, 7
;MyProject.c,104 :: 		}
L_end_MoveForward:
	RETURN
; end of _MoveForward

_MoveBackward:

;MyProject.c,106 :: 		void MoveBackward() {
;MyProject.c,107 :: 		LEFT_MOTOR_FORWARD = 0;
	BCF        PORTC+0, 4
;MyProject.c,108 :: 		LEFT_MOTOR_BACKWARD = 1;
	BSF        PORTC+0, 5
;MyProject.c,109 :: 		RIGHT_MOTOR_FORWARD = 0;
	BCF        PORTC+0, 6
;MyProject.c,110 :: 		RIGHT_MOTOR_BACKWARD = 1;
	BSF        PORTC+0, 7
;MyProject.c,111 :: 		}
L_end_MoveBackward:
	RETURN
; end of _MoveBackward

_TurnLeft:

;MyProject.c,113 :: 		void TurnLeft() {
;MyProject.c,114 :: 		LEFT_MOTOR_FORWARD = 0;
	BCF        PORTC+0, 4
;MyProject.c,115 :: 		LEFT_MOTOR_BACKWARD = 1;
	BSF        PORTC+0, 5
;MyProject.c,116 :: 		RIGHT_MOTOR_FORWARD = 1;
	BSF        PORTC+0, 6
;MyProject.c,117 :: 		RIGHT_MOTOR_BACKWARD = 0;
	BCF        PORTC+0, 7
;MyProject.c,118 :: 		}
L_end_TurnLeft:
	RETURN
; end of _TurnLeft

_TurnRight:

;MyProject.c,120 :: 		void TurnRight() {
;MyProject.c,121 :: 		LEFT_MOTOR_FORWARD = 1;
	BSF        PORTC+0, 4
;MyProject.c,122 :: 		LEFT_MOTOR_BACKWARD = 0;
	BCF        PORTC+0, 5
;MyProject.c,123 :: 		RIGHT_MOTOR_FORWARD = 0;
	BCF        PORTC+0, 6
;MyProject.c,124 :: 		RIGHT_MOTOR_BACKWARD = 1;
	BSF        PORTC+0, 7
;MyProject.c,125 :: 		}
L_end_TurnRight:
	RETURN
; end of _TurnRight

_Stop:

;MyProject.c,127 :: 		void Stop() {
;MyProject.c,128 :: 		LEFT_MOTOR_FORWARD = 0;
	BCF        PORTC+0, 4
;MyProject.c,129 :: 		LEFT_MOTOR_BACKWARD = 0;
	BCF        PORTC+0, 5
;MyProject.c,130 :: 		RIGHT_MOTOR_FORWARD = 0;
	BCF        PORTC+0, 6
;MyProject.c,131 :: 		RIGHT_MOTOR_BACKWARD = 0;
	BCF        PORTC+0, 7
;MyProject.c,133 :: 		}
L_end_Stop:
	RETURN
; end of _Stop

_moveServo:

;MyProject.c,134 :: 		void moveServo(int angle) {
;MyProject.c,141 :: 		duty = ((1000 + (angle * 1000 / 180)) * 1024) / 20000;
	MOVF       FARG_moveServo_angle+0, 0
	MOVWF      R0+0
	MOVF       FARG_moveServo_angle+1, 0
	MOVWF      R0+1
	MOVLW      232
	MOVWF      R4+0
	MOVLW      3
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVLW      180
	MOVWF      R4+0
	CLRF       R4+1
	CALL       _Div_16x16_S+0
	MOVF       R0+0, 0
	ADDLW      232
	MOVWF      R3+0
	MOVLW      3
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      R0+1, 0
	MOVWF      R3+1
	MOVLW      10
	MOVWF      R2+0
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__moveServo37:
	BTFSC      STATUS+0, 2
	GOTO       L__moveServo38
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__moveServo37
L__moveServo38:
	MOVLW      32
	MOVWF      R4+0
	MOVLW      78
	MOVWF      R4+1
	CALL       _Div_16x16_S+0
;MyProject.c,143 :: 		CCPR1L = duty >> 2;      // Set upper 8 bits of duty cycle
	MOVF       R0+0, 0
	MOVWF      R2+0
	MOVF       R0+1, 0
	MOVWF      R2+1
	RRF        R2+1, 1
	RRF        R2+0, 1
	BCF        R2+1, 7
	RRF        R2+1, 1
	RRF        R2+0, 1
	BCF        R2+1, 7
	MOVF       R2+0, 0
	MOVWF      CCPR1L+0
;MyProject.c,144 :: 		CCP1CON &= 0xCF;         // Clear lower 2 bits
	MOVLW      207
	ANDWF      CCP1CON+0, 1
;MyProject.c,145 :: 		CCP1CON |= (duty & 0x03) << 4;  // Set lower 2 bits of duty cycle
	MOVLW      3
	ANDWF      R0+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVF       R0+0, 0
	IORWF      CCP1CON+0, 1
;MyProject.c,146 :: 		}
L_end_moveServo:
	RETURN
; end of _moveServo

_RaiseFlag:

;MyProject.c,148 :: 		void RaiseFlag() {
;MyProject.c,150 :: 		moveServo(90);
	MOVLW      90
	MOVWF      FARG_moveServo_angle+0
	MOVLW      0
	MOVWF      FARG_moveServo_angle+1
	CALL       _moveServo+0
;MyProject.c,151 :: 		Delay_ms(1000);
	MOVLW      6
	MOVWF      R11+0
	MOVLW      19
	MOVWF      R12+0
	MOVLW      173
	MOVWF      R13+0
L_RaiseFlag12:
	DECFSZ     R13+0, 1
	GOTO       L_RaiseFlag12
	DECFSZ     R12+0, 1
	GOTO       L_RaiseFlag12
	DECFSZ     R11+0, 1
	GOTO       L_RaiseFlag12
	NOP
	NOP
;MyProject.c,152 :: 		moveServo(0);
	CLRF       FARG_moveServo_angle+0
	CLRF       FARG_moveServo_angle+1
	CALL       _moveServo+0
;MyProject.c,153 :: 		Delay_ms(1000);}
	MOVLW      6
	MOVWF      R11+0
	MOVLW      19
	MOVWF      R12+0
	MOVLW      173
	MOVWF      R13+0
L_RaiseFlag13:
	DECFSZ     R13+0, 1
	GOTO       L_RaiseFlag13
	DECFSZ     R12+0, 1
	GOTO       L_RaiseFlag13
	DECFSZ     R11+0, 1
	GOTO       L_RaiseFlag13
	NOP
	NOP
L_end_RaiseFlag:
	RETURN
; end of _RaiseFlag

_ControlFan:

;MyProject.c,164 :: 		void ControlFan() {
;MyProject.c,165 :: 		if (SWITCH_PIN) {
	BTFSS      PORTD+0, 3
	GOTO       L_ControlFan14
;MyProject.c,166 :: 		FAN_FORWARD = 1;
	BSF        PORTD+0, 1
;MyProject.c,167 :: 		FAN_BACKWARD = 0;
	BCF        PORTD+0, 2
;MyProject.c,168 :: 		} else {
	GOTO       L_ControlFan15
L_ControlFan14:
;MyProject.c,169 :: 		FAN_FORWARD = 0;
	BCF        PORTD+0, 1
;MyProject.c,170 :: 		FAN_BACKWARD = 0;
	BCF        PORTD+0, 2
;MyProject.c,171 :: 		}
L_ControlFan15:
;MyProject.c,172 :: 		}
L_end_ControlFan:
	RETURN
; end of _ControlFan

_main:

;MyProject.c,174 :: 		void main() {
;MyProject.c,175 :: 		Initialize();
	CALL       _Initialize+0
;MyProject.c,177 :: 		RaiseFlag(); // Raise the flag before starting
	CALL       _RaiseFlag+0
;MyProject.c,179 :: 		while(IR_PIN){
L_main16:
	BTFSS      PORTD+0, 4
	GOTO       L_main17
;MyProject.c,180 :: 		ControlFan();
	CALL       _ControlFan+0
;MyProject.c,181 :: 		distance_front = ReadUltrasonicSensor();
	CALL       _ReadUltrasonicSensor+0
	MOVF       R0+0, 0
	MOVWF      _distance_front+0
	MOVF       R0+1, 0
	MOVWF      _distance_front+1
;MyProject.c,183 :: 		if (distance_front < 20) { // Distance threshold in cm
	MOVLW      0
	SUBWF      R0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main42
	MOVLW      20
	SUBWF      R0+0, 0
L__main42:
	BTFSC      STATUS+0, 0
	GOTO       L_main18
;MyProject.c,184 :: 		Stop();
	CALL       _Stop+0
;MyProject.c,185 :: 		Delay_ms(500); // Pause for 500 ms
	MOVLW      3
	MOVWF      R11+0
	MOVLW      138
	MOVWF      R12+0
	MOVLW      85
	MOVWF      R13+0
L_main19:
	DECFSZ     R13+0, 1
	GOTO       L_main19
	DECFSZ     R12+0, 1
	GOTO       L_main19
	DECFSZ     R11+0, 1
	GOTO       L_main19
	NOP
	NOP
;MyProject.c,186 :: 		MoveBackward();
	CALL       _MoveBackward+0
;MyProject.c,187 :: 		Delay_ms(200); // Move backward for 500 ms
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_main20:
	DECFSZ     R13+0, 1
	GOTO       L_main20
	DECFSZ     R12+0, 1
	GOTO       L_main20
	DECFSZ     R11+0, 1
	GOTO       L_main20
	NOP
;MyProject.c,188 :: 		Stop();
	CALL       _Stop+0
;MyProject.c,189 :: 		Delay_ms(500);
	MOVLW      3
	MOVWF      R11+0
	MOVLW      138
	MOVWF      R12+0
	MOVLW      85
	MOVWF      R13+0
L_main21:
	DECFSZ     R13+0, 1
	GOTO       L_main21
	DECFSZ     R12+0, 1
	GOTO       L_main21
	DECFSZ     R11+0, 1
	GOTO       L_main21
	NOP
	NOP
;MyProject.c,190 :: 		TurnLeft();
	CALL       _TurnLeft+0
;MyProject.c,191 :: 		Delay_ms(350);
	MOVLW      2
	MOVWF      R11+0
	MOVLW      199
	MOVWF      R12+0
	MOVLW      136
	MOVWF      R13+0
L_main22:
	DECFSZ     R13+0, 1
	GOTO       L_main22
	DECFSZ     R12+0, 1
	GOTO       L_main22
	DECFSZ     R11+0, 1
	GOTO       L_main22
	NOP
;MyProject.c,192 :: 		Stop();
	CALL       _Stop+0
;MyProject.c,193 :: 		Delay_ms(1000);
	MOVLW      6
	MOVWF      R11+0
	MOVLW      19
	MOVWF      R12+0
	MOVLW      173
	MOVWF      R13+0
L_main23:
	DECFSZ     R13+0, 1
	GOTO       L_main23
	DECFSZ     R12+0, 1
	GOTO       L_main23
	DECFSZ     R11+0, 1
	GOTO       L_main23
	NOP
	NOP
;MyProject.c,194 :: 		}
	GOTO       L_main24
L_main18:
;MyProject.c,196 :: 		MoveForward();
	CALL       _MoveForward+0
;MyProject.c,197 :: 		Delay_ms(500);
	MOVLW      3
	MOVWF      R11+0
	MOVLW      138
	MOVWF      R12+0
	MOVLW      85
	MOVWF      R13+0
L_main25:
	DECFSZ     R13+0, 1
	GOTO       L_main25
	DECFSZ     R12+0, 1
	GOTO       L_main25
	DECFSZ     R11+0, 1
	GOTO       L_main25
	NOP
	NOP
;MyProject.c,199 :: 		}
L_main24:
;MyProject.c,205 :: 		}
	GOTO       L_main16
L_main17:
;MyProject.c,206 :: 		Stop();
	CALL       _Stop+0
;MyProject.c,207 :: 		FAN_FORWARD = 0;
	BCF        PORTD+0, 1
;MyProject.c,208 :: 		FAN_BACKWARD = 0;
	BCF        PORTD+0, 2
;MyProject.c,210 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
