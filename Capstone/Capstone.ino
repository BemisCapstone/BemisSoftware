#define ENCODER_A_MASK 0b00000001
#define ENCODER_B_MASK 0b00000010
#define CLOCK_DIVIDER 8
#define dt 0.01

//Encoder variables:
volatile uint32_t timerMeasurement = 0;
volatile uint32_t timer1OverflowCounter = 0;
volatile uint32_t counterCapture = 0;
volatile uint64_t timer1TotalClocks = 0; // clock cycles between two rising edges
volatile bool direction = true;
volatile bool stopped = false;
volatile float motorRPM = 0;
volatile float setPoint = 0;

const int pinA = 1;
const int pinB = 4;
const int pinC = 5;
const int pinD = 6;
const int pinE = 7;
const int pinF = 8;
const int pinG = 9;
const int D1 = 15;
const int D2 = 14;
const int D3 = 16;
const int D4 = 10;
const int encoderA = 2;
const int encoderB = 3;
const int resetPin = 19;
/*const int calibratePin = 18;
const int signalLossLED = 15;
const int exceeds9999LED = 14;
const int reverseLED = 16;
const int slowingDownLED = 10; */


/*void setupEncoders(){
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

void setupPinChangeInterrupts() {
  PCICR  |= 0b00000001;    // turn on port b
  PCMSK0 |= 0b00110000;    // turn on physical pins 18, 19
}


/* Interupt Service Routines */

/*ISR(TIMER1_OVF_vect){
  timer1OverflowCounter++;
  // will overflow every 4.46 years if the motor is not moved
}

ISR(INT0_vect){
  timerMeasurement = TCNT1; // store the timer value
  TCNT1 = 0;

  // read the state of B
  direction = PIND & ENCODER_B_MASK;
  
  if (timer1OverflowCounter > 0)
    timer1TotalClocks = CLOCK_DIVIDER * (timerMeasurement + (timer1OverflowCounter * 65536)) + 26; // compensate for code indeterminancy (± 22)
  else
    timer1TotalClocks = CLOCK_DIVIDER * timerMeasurement;
    
  counterCapture = timer1OverflowCounter;
  timer1OverflowCounter = 0;
  motorRPM = F_CPU * 60 / timer1TotalClocks;
  
  // check for signal loss
  if (timer1TotalClocks < 2) 
    stopped = true;
  else
    stopped = false;
}

ISR(PCINT0_vect){
  if (digitalRead(resetPin)) {
    timerMeasurement = 0;
    timer1OverflowCounter = 0;
    counterCapture = 0;
    timer1TotalClocks = 0; // clock cycles between two rising edges
    direction = true;
    motorRPM = 0;
    setPoint = 0;

    //clearLEDs();
    digitalWrite(signalLossLED, LOW);
    digitalWrite(exceeds9999LED, LOW);
    digitalWrite(reverseLED, LOW);
    digitalWrite(slowingDownLED, LOW);
  }
  
  if (digitalRead(calibratePin))
    setPoint = motorRPM;
}
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
 /* pinMode(signalLossLED, OUTPUT);
  pinMode(exceeds9999LED, OUTPUT);
  pinMode(reverseLED, OUTPUT);
  pinMode(slowingDownLED, OUTPUT);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(resetPin, INPUT);
  pinMode(calibratePin, INPUT);

 /* setupEncoders();
  setupTimer1();
  setupPinChangeInterrupts();
  Serial.begin(9600); */
}


void loop() {
  // check for signal loss
 /* if (stopped) 
    digitalWrite(signalLossLED, HIGH);
  else
    digitalWrite(signalLossLED, LOW);

  // check to see if the RPM exceeds 9999
  if (motorRPM > 9999) 
    digitalWrite(exceeds9999LED, HIGH);
  else
    digitalWrite(exceeds9999LED, LOW);

  // check the polarity of the motor
  if (!direction && (motorRPM != 0)) 
    digitalWrite(reverseLED, HIGH);
  else
    digitalWrite(reverseLED, LOW);

  // check to see if the motor is slowing down
  if ((setPoint - motorRPM > 10) && (motorRPM != 0) && direction) 
    digitalWrite(slowingDownLED, HIGH);
  else
    digitalWrite(slowingDownLED, LOW);

  // Serial print statements for debugging
  Serial.print("RPM: ");
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
  Serial.println(motorRPM / 60.0);
  delay(500);

  // break motorRPM into 4 digits for displaying
  int num = motorRPM;
  int digit1 = num / 1000;
  int digit2 = (num - (digit1 * 1000)) / 100;
  int digit3 = (num - (digit1 * 1000) - (digit2 * 100)) / 10;
  int digit4 = (num - (digit1 * 1000) - (digit2 * 100) - (digit3 * 10));

  // display the RPM digits
/*  clearLEDs();
  delay(500);
  writeNum(digit1);
  delay(500);
  clearLEDs();
  delay(100);
  writeNum(digit2);
  delay(500);
  clearLEDs();
  delay(100);
  writeNum(digit3);
  delay(500);
  clearLEDs();
  delay(100);
  writeNum(digit4);
  delay(400); */

  pickDigit(1);
  writeNum(1);
  delay(500);
  clearLEDs();
  pickDigit(2);
  writeNum(2);
  /*
  pickDigit(3);
  writeNum(3);
  pickDigit(4);
  writeNum(4); */
}

void pickDigit(int n){
  digitalWrite(D1, HIGH);
  digitalWrite(D2, HIGH);
  digitalWrite(D3, HIGH);
  digitalWrite(D4, HIGH); 

  switch(n)
  {
    case 1:
      digitalWrite(D1, LOW);
      break;
    case 2:
      digitalWrite(D2, LOW);
      break;
    case 3:
       digitalWrite(D3, LOW);
       break;
    case 4:
      digitalWrite(D4, LOW);
      break;      
  }
}

void writeNum(int x){
  switch(x)
  {
    default: zero(); break;
    case 1: one(); break;
    case 2: two(); break;
    case 3: three(); break;
    case 4: four(); break;
    case 5: five(); break;
    case 6: six(); break;
    case 7: seven(); break;
    case 8: eight(); break;
    case 9: nine(); break;
  }
}

void zero()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, LOW);
}

void one()
{
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
}

void two()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, HIGH);
}

void three()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, HIGH);
}

void four()
{
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
}

void five()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
}

void six()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
}

void seven()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
}

void eight()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
}

void nine()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
}

void clearLEDs()
{
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
}
