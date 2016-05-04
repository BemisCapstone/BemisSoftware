//If the motor falls below the setPoint by this percentage, the slowing down LED will turn on
#define SLOWING_DOWN_THRESHOLD_PERCENT 5
#define ERROR_LED_BRIGHTNESS_PERCENT 50
#define SEVEN_SEG_BRIGHTNESS_PERCENT 100

#define ENCODER_A_MASK 0b00000001
#define ENCODER_B_MASK 0b00000010
#define CLOCK_DIVIDER 8
#define dt 0.01

#define SEVEN_SEG_PORT PORTB
#define ERROR_LED_PORT PORTF
#define ERROR_LED_BRIGHTNESS_PORT PORTC
#define SEVEN_SEG_BRIGHTNESS_PORT PORTC
#define ENCODER_A_PORT PORTD
#define ENCODER_B_PORT PORTD
#define CALIBRATION_PORT PORTD
#define SEVEN_SEG_DIGIT_PORT PORTD

#define ERROR_LED_BRIGHTNESS_PIN 7
#define SEVEN_SEG_BRIGHTNESS_PIN 6
#define ENCODER_A_PIN 0
#define ENCODER_B_PIN 1
#define CALIBRATION_PIN 2
#define SEVEN_SEG_D1_PIN 4
#define SEVEN_SEG_D2_PIN 5
#define SEVEN_SEG_D3_PIN 6
#define SEVEN_SEG_D4_PIN 7
#define ERROR_SIGNAL_LOSS_PIN 0
#define ERROR_EXCEEDS_9999_PIN 4
#define ERROR_REVERSE_PIN 5
#define ERROR_SLOWING_DOWN_PIN 1

/* GPIO Controls
 * Used for controlling the direction, state and reading of GPIO pins. Uses the PORT register to find the DDR and PIN 
 * addresses. This can be done because the register pointers are always sequentially ordered (In every AVR I've ever used).
 * e.g. setPORT(PORTB,4,HIGH);
 * e.g. setDDR(PORTB,4,INPUT);
 * e.g. x = getPIN(PORTB,4); 
 */
#define setPORT(_port,_bit,_state) (_state > 0 ? (_port |= (1 << _bit)) : (_port &= ~(1 << _bit)))
#define setDDR(_port,_bit,_direction) (_direction > 0 ? (*(&_port - 1) |= (1 << _bit)) : (*(&_port - 1) &= ~(1 << _bit)))
#define getPIN(_port,_bit) ((*(&_port - 2) & (1 << _bit)) > 0 ? HIGH : LOW)

const int digit[] = {0b11100111, 0b01000100, 0b10101101, 0b01101101, 0b01001110, 0b01101011, 0b11101011, 0b01000101, 0b11101111, 0b01101111};



//Encoder variables:
volatile uint32_t timerMeasurement = 0;
volatile uint32_t timer1OverflowCounter = 0;
volatile uint32_t counterCapture = 0;
volatile uint64_t timer1TotalClocks = 0; // clock cycles between two rising edges
volatile bool direction = true;
volatile bool stopped = false;
volatile float motorRPM = 0;
volatile float setPoint = 0;

//Seven Segment Display Variable:
uint8_t currentDigit = 0;

unsigned long timestamp = 0;


void setupEncoders(){
  EICRA = (0x0f); // trigger on rising edge
  EIMSK = (1 << INT0); // enable interrupt INT0
}

//divide by 8 clock
//set up 16 bit timer
void setupTimer1(){
  TCCR1A = 0; //sets timer1 control register A bits
  TCNT1 = 0;
  TIMSK1 |= (1<<TOIE1); //enable interrupt to fire on overflow
  TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10); //sets clock divider to divide by 8
} 

//timer for 7 seg brightness
void setupTimer3(){
  TCCR3A = 0; //sets timer3 control register A bits
  TCCR3B = 0; 
  TCCR3A |= (1<<COM3A1) | (0<<COM3A0);
  //waveform generation mode = Fast PWM and the TOP value is stored in ICR3
  TCCR3A |= (1<<WGM31) | (0<<WGM30);
  TCCR3B |= (1<< WGM33) | (1<<WGM32);
  TCNT3 = 0;
  ICR3 = 5000; //sets PWM frequency to 400 Hz
  OCR3A = SEVEN_SEG_BRIGHTNESS_PERCENT/100.0*ICR3;
  
  TCCR3B |= (0<<CS32) | (1<<CS31) | (0<<CS30); //sets clock divider to divide by 8
}

//timer for errror LED brightness
void setupTimer4(){
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  TCCR4D = 0;
  TCCR4E = 0;
  TCCR4A |= (1<<COM4A1) | (0<<COM4A0);
  TCCR4A |= (1<<PWM4A) | (0<<PWM4B);
  TCNT4  = 0;
  OCR4C = 156; //set TOP value, sets PWM frequency to 400 Hz
  OCR4A = ERROR_LED_BRIGHTNESS_PERCENT/100.0*OCR4C;

  TCCR4B |= (1<<CS43) | (0<<CS42) | (0<<CS41) | (1<<CS40); //sets clock divider by 256
}


/* Interupt Service Routines */

ISR(TIMER1_OVF_vect){
  timer1OverflowCounter++;
  // will overflow every 4.46 years if the motor is not moved
}

ISR(INT0_vect){
  timerMeasurement = TCNT1; // store the timer value
  TCNT1 = 0;

  // read the state of B
  direction = PIND & ENCODER_B_MASK;

  //If this interrupt executed exactly when the timer overflows, the overflow counter won't be incremented as it should 
  if(timer1TotalClocks < 2)
    timer1OverflowCounter++;
  
  if (timer1OverflowCounter > 0)
    timer1TotalClocks = CLOCK_DIVIDER * (timerMeasurement + (timer1OverflowCounter * 65536)) + 26; // compensate for code indeterminancy (± 22)
  else
    timer1TotalClocks = CLOCK_DIVIDER * timerMeasurement;
    
  counterCapture = timer1OverflowCounter;
  timer1OverflowCounter = 0;
  motorRPM = F_CPU * 60 / timer1TotalClocks;
}

void setup() {
  
  *(&SEVEN_SEG_PORT -1) =0xFF;
  
  setDDR(SEVEN_SEG_DIGIT_PORT, SEVEN_SEG_D1_PIN, OUTPUT);
  setDDR(SEVEN_SEG_DIGIT_PORT, SEVEN_SEG_D2_PIN, OUTPUT);
  setDDR(SEVEN_SEG_DIGIT_PORT, SEVEN_SEG_D3_PIN, OUTPUT);
  setDDR(SEVEN_SEG_DIGIT_PORT, SEVEN_SEG_D4_PIN, OUTPUT);

  setDDR(ERROR_LED_PORT, ERROR_SIGNAL_LOSS_PIN, OUTPUT);
  setDDR(ERROR_LED_PORT, ERROR_EXCEEDS_9999_PIN, OUTPUT);
  setDDR(ERROR_LED_PORT, ERROR_REVERSE_PIN, OUTPUT);
  setDDR(ERROR_LED_PORT, ERROR_SLOWING_DOWN_PIN, OUTPUT); 
  
  setDDR(ENCODER_A_PORT, ENCODER_A_PIN, INPUT);
  setDDR(ENCODER_B_PORT, ENCODER_B_PIN, INPUT);
  setDDR(CALIBRATION_PORT, CALIBRATION_PIN, INPUT); 

  setDDR(ERROR_LED_BRIGHTNESS_PORT, ERROR_LED_BRIGHTNESS_PIN, OUTPUT);
  setDDR(SEVEN_SEG_BRIGHTNESS_PORT, SEVEN_SEG_BRIGHTNESS_PIN, OUTPUT); 


  setPORT(ERROR_LED_BRIGHTNESS_PORT, ERROR_LED_BRIGHTNESS_PIN, HIGH);
  setPORT(SEVEN_SEG_BRIGHTNESS_PORT, SEVEN_SEG_BRIGHTNESS_PIN, HIGH); 
  
  setupEncoders();
  setupTimer1();
  setupTimer3();
  setupTimer4();
  Serial.begin(9600); 
}


void loop() {

  while(1)
  {
  //Clear all error LEDs
  ERROR_LED_PORT=0;
  
  //signal loss defined as motor RPM < 60
  if(timer1OverflowCounter > 31)
  {
    setPORT(ERROR_LED_PORT, ERROR_SIGNAL_LOSS_PIN, HIGH);
    motorRPM = 0;
  }
  if(motorRPM > 9999)
    setPORT(ERROR_LED_PORT, ERROR_EXCEEDS_9999_PIN, HIGH);

  if(!direction && motorRPM !=0)
    setPORT(ERROR_LED_PORT, ERROR_REVERSE_PIN, HIGH);

  if(setPoint - motorRPM*(1+ SLOWING_DOWN_THRESHOLD_PERCENT/100.0) > 0 && motorRPM != 0)
    setPORT(ERROR_LED_PORT, ERROR_SLOWING_DOWN_PIN, HIGH);

  if(!getPIN(CALIBRATION_PORT, CALIBRATION_PIN))
    setPoint = motorRPM;

  // Serial print statements for debugging
/*  Serial.print("RPM: ");
  Serial.println(motorRPM);
  Serial.print("Set Point: ");
  Serial.println(setPoint);
  Serial.print("Overflow: ");
  Serial.println(counterCapture);
  Serial.print("Timer Measurement: ");
  Serial.println(timerMeasurement);
  Serial.print("Clocks: ");
  Serial.println((uint32_t)timer1TotalClocks);
  Serial.print("Frequency (Hz): ");
  Serial.println(motorRPM / 60.0); */


  // display the RPM digits

  setPORT(SEVEN_SEG_DIGIT_PORT, SEVEN_SEG_D1_PIN + currentDigit++, LOW);

//If current digit counter points to the 5th nonexistant digit, set it back to zero
  if(currentDigit > 3)
    currentDigit = 0;

  //Change the A, B, C, D, E, F, G< DP pints to the next digit
  SEVEN_SEG_PORT = digit[uint8_t(motorRPM/pow(10,3-currentDigit))%10];

  //Turn on current digit
  setPORT(SEVEN_SEG_DIGIT_PORT, SEVEN_SEG_D1_PIN + currentDigit, HIGH);

//make each loop last 2ms
  while(millis() - timestamp < 2);
  timestamp = millis();
  }
}



