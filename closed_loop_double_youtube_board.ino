//Program to run 2 brushless motors in closed loop mode as a Steer-by-Wire Demo
//By Juan Pablo Angulo
//email: juanpablocanguro@hotmail.com
//Use a Power Stage, such as AO4606 to drive the 3 coils of your motor.
//Can be run on a board such as https://www.ebay.com/itm/124127740844
//Arduino forum thread: https://forum.arduino.cc/index.php?topic=672887.0

const int K=7; // Proportional gain for position control, tune it to your liking 1-10 or more...

const int encoderA = A0;  // AMS AS5048 encoder
const int encoderB = A1;  // AMS AS5048 encoder 

const int motorPinAa =3; //Coil A
const int motorPinAb =5; //Coil B
const int motorPinAc =6; //Coil C

const int motorPinBa =9; //Coil A
const int motorPinBb =10; //Coil B
const int motorPinBc =11; //Coil C

const int PA = 7; //Motor's A number of Poles 
const int PB = 11; //Motor's B number of Poles 

// Variables
int pwmSin[] = {127,110,94,78,64,50,37,26,17,10,4,1,0,
                1,4,10,17,26,37,50,64,78,94,110,127,
                144,160,176,191,204,217,228,237,244,250,253,254,255,
                250,244,237,228,217,204,191,176,160,144}; // Lookup table array of 48 PWM duty values for 8-bit timer - sine function

int currentStepAa=0;  //index for lookup table Coil A 
int currentStepAb=16; //index for lookup table Coil B
int currentStepAc=32; //index for lookup table Coil C

int currentStepBa=0;  //index for lookup table Coil A 
int currentStepBb=16; //index for lookup table Coil B
int currentStepBc=32; //index for lookup table Coil C

int posA=0; //mechanical position of shaft
int posB=0; //mechanical position of shaft

int eposA=0; //electrical position of shaft
int eposB=0; //electrical position of shaft

int torqueA=0; //MAX output torque
int torqueB=0; //MAX output torque

int directnA=0; //vector for Magnetic field orientation
int directnB=0; //vector for Magnetic field orientation

int setpoint=0; //variable for storing desired position

int errorA=0; //variable for calculating the error for Proportional Control
int errorB=0; //variable for calculating the error for Proportional Control

void setup() {
  Serial.begin(9600);
  TCCR0B = TCCR0B & 0b11111000 | 0x03; // this one stays in 0x03 otherwise important timers are affected
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 9 and 10
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 11 and 3 
  ICR1 = 255 ; // 8 bit resolution
 
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  
  pinMode(motorPinAa, OUTPUT);
  pinMode(motorPinAb, OUTPUT);
  pinMode(motorPinAc, OUTPUT);
  
  pinMode(motorPinBa, OUTPUT);
  pinMode(motorPinBb, OUTPUT);
  pinMode(motorPinBc, OUTPUT);
}
 
void loop() {

  posA=pulseIn(encoderA,HIGH);//read encoder pulse
  eposA=map(posA,4,905,0,(48*PA)); //scale shaft's encoder position range to electrical positions multiple of 48*Number of poles in motor.
  eposA=constrain(eposA, 0, 48*PA); //constraint the values
  posA=map(eposA,0,(48*PA),0,360); //translate the electrical position to shaft position in degrees (0-360)
  
  posB=pulseIn(encoderB,HIGH);//read encoder pulse
  eposB=map(posB,4,890,0,(48*PB)); //scale shaft's encoder position range to electrical positions multiple of 48*Number of poles in motor.
  eposB=constrain(eposB, 0, 48*PB); //constraint the values
  posB=map(eposB,0,(48*PB),0,360); //translate the electrical position to shaft position in degrees (0-360)

  //MAGIC HAPPENS HERE
  setpoint = (posA+posB)/2; //setpoint is the average between both motor positions
   
 // Proportional control
  errorA=posA-setpoint; //calculate error
  if (errorA<0){directnA=36;} //orient magnetic field vector 90 degrees ahead
  if (errorA>0){directnA=12;} //orient magnetic field vector 90 degrees behind 

  torqueA=abs(errorA*K); //define the magnitude of the reaction of the motor, or the Proportional Gain. Small torque for small errors, large torque for large errors. Adjust the Value of K to better match the response you want from the motor, try 1,2,5,8,10.
  torqueA=constrain(torqueA,0,100);    //limit maximum torque 0-100 %, you can limit the maximum torque you want the motor to have i.e. instead of 100, use 60 for 60% of the maximum torque available, motor runs cooler ;)
  
  errorB=posB-setpoint; //calculate error
  if (errorB<0){directnB=36;} //orient magnetic field vector 90 degrees ahead
  if (errorB>0){directnB=12;} //orient magnetic field vector 90 degrees behind 

  torqueB=abs(errorB*K); //define the magnitude of the reaction of the motor, or the Proportional Gain. Small torque for small errors, large torque for large errors. Adjust the Value of K to better match the response you want from the motor, try 1,2,5,8,10.
  torqueB=constrain(torqueB,0,100);//limit maximum torque 0-100 %, you can limit the maximum torque you want the motor to have i.e. instead of 100, use 60 for 60% of the maximum torque available, motor runs cooler ;)
  
  move();
}
  
void move()
{
  const int offsetA=15; //OFFSET needed for Syncronization, unique to each motor as it depends on manufacturing.
  currentStepAa = eposA + directnA + offsetA;  //directn is used to define the rotation the motor sould turn.
  currentStepAb = currentStepAa + 16; //add 120 deg of phase to StepA position. 
  currentStepAc = currentStepAa + 32; //add 240 deg of phase to StepA position.
 
  currentStepAa = currentStepAa%48; //I used remainder operation or modulo to "wrap" the values between 0 and 47
  currentStepAb = currentStepAb%48;
  currentStepAc = currentStepAc%48;

  const int offsetB=11; //OFFSET needed for Syncronization, unique to each motor as it depends on manufacturing.
  currentStepBa = eposB + directnB + offsetB;  //directn is used to define the rotation the motor sould turn.
  currentStepBb = currentStepBa + 16; //add 120 deg of phase to StepA position. 
  currentStepBc = currentStepBa + 32; //add 240 deg of phase to StepA position.
 
  currentStepBa = currentStepBa%48; //I used remainder operation or modulo to "wrap" the values between 0 and 47
  currentStepBb = currentStepBb%48;
  currentStepBc = currentStepBc%48;

 //write PWM values to ports
   
  analogWrite(motorPinAa, pwmSin[currentStepAa]*torqueA/100);
  analogWrite(motorPinAb, pwmSin[currentStepAb]*torqueA/100);
  analogWrite(motorPinAc, pwmSin[currentStepAc]*torqueA/100);
  
  analogWrite(motorPinBa, pwmSin[currentStepBa]*torqueB/100);
  analogWrite(motorPinBb, pwmSin[currentStepBb]*torqueB/100);
  analogWrite(motorPinBc, pwmSin[currentStepBc]*torqueB/100); 
 /*
  Serial.print("DATA,");
  Serial.print(pwmSin[currentStepA]);
  Serial.print(","); 
  Serial.print(pwmSin[currentStepB]);
  Serial.print(","); 
  Serial.print(pwmSin[currentStepC]);
  Serial.print(","); 
  Serial.println(pos);
  */
    //Serial.println(error);
  
}
