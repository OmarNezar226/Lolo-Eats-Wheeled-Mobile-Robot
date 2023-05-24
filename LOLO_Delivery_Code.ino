#include <util/atomic.h>

#define ENCA 18
#define ENCB 19
#define PWM 6   // elly odam 3l shmal
#define IN2 32
#define IN1 30

/*MOTOR 2 pins*/

#define Mot2_ENCA 2
#define Mot2_ENCB 3
#define Mot2_PWM 8   // elly odam 3l ymeen
#define Mot2_IN2 34
#define Mot2_IN1 36


/*MOTOR 3 pins*/


#define Mot3_PWM 7
#define Mot3_IN1 26
#define Mot3_IN2 28

/*MOTOR 4 pins*/


#define Mot4_PWM 4
#define Mot4_IN1 22
#define Mot4_IN2 24

/*Ultrasonics pins*/

#define T1 40
#define T2 42
#define T3 44
#define E1 41
#define E2 43
#define E3 45

double t1;
double t2;
double t3;
double d1;
double d2;
double d3;

double x;
double y;
double z;

char command = '0';

/*MOTORs global Vars*/
long prevT = 0;
volatile float prevT_i = 0;

/*MOTOR 1 Vars*/
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float eintegral = 0;
float eprev = 0;

/*MOTOR 2 Vars*/
int Mot2_posPrev = 0;
volatile int Mot2_pos_i = 0;
volatile float Mot2_velocity_i = 0;
float Mot2_v1Filt = 0;
float Mot2_v1Prev = 0;
float Mot2_v2Filt = 0;
float Mot2_v2Prev = 0;
float Mot2_eintegral = 0;
float Mot2_eprev = 0;
int dir=1;
int Mot2_dir=1;
int pwr;
int Mot2_pwr;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder();
void setMotor2(int Mot2_dir, int Mot2_pwmVal, int Mot2_pwm, int Mot2_in1, int Mot2_in2);
void Mot2_readEncoder();
void pid1(int v_1);
void pid2(int v_2);


void setup() {
  // put your setup code here, to run once:
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(Mot2_ENCA, INPUT);
  pinMode(Mot2_ENCB, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(Mot2_IN1, OUTPUT);
  pinMode(Mot2_IN2, OUTPUT);
  pinMode(Mot2_PWM, OUTPUT);
  pinMode(Mot3_IN1, OUTPUT);
  pinMode(Mot3_IN2, OUTPUT);
  pinMode(Mot3_PWM, OUTPUT);
  pinMode(Mot4_IN1, OUTPUT);
  pinMode(Mot4_IN2, OUTPUT);
  pinMode(Mot4_PWM, OUTPUT);


  pinMode(T1, OUTPUT);
  pinMode(T2, OUTPUT);
  pinMode(T3, OUTPUT);
  pinMode(E1, INPUT);
  pinMode(E2, INPUT);
  pinMode(E3, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Mot2_ENCB), Mot2_readEncoder, RISING);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(T1, HIGH);
  delayMicroseconds(10);
  digitalWrite(T1, LOW);
  t1 = pulseIn(E1, HIGH);
  d1 = t1 * 0.017;

  digitalWrite(T2, HIGH);
  delayMicroseconds(10);
  digitalWrite(T2, LOW);
  t2 = pulseIn(E2, HIGH);
  d2 = t2 * 0.017;

  digitalWrite(T3, HIGH);
  delayMicroseconds(10);
  digitalWrite(T3, LOW);
  t3 = pulseIn(E3, HIGH);
  d3 = t3 * 0.017;

  if (d1 > 10 && d2 > 10 && d3 > 10) 
  {

    if (millis() > 2000) 
    {

      if (Serial.available() > 0) {
          command = Serial.read();
          if(command == '1')
          {

            pid1(250);
            pid2(50);
            setMotor(dir,pwr,PWM,IN1,IN2);
            setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
            setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
            setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
          }
          else if(command == '2')
          {

            pid1(50);
            pid2(250);
            setMotor(dir,pwr,PWM,IN1,IN2);
            setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
            setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
            setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
          }
          else if(command == '3')
          {

            pid1(250);
            pid2(250);
            setMotor(dir,pwr,PWM,IN1,IN2);
            setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
            setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
            setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
          }
          else if(command == 'A'){

            pid1(200);
            pid2(200);
            setMotor(dir,pwr,PWM,IN1,IN2);
            setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
            setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
            setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
          }
          else if(command == '6')
          {
              pid1(0);
              pid2(0);
              setMotor(dir,pwr,PWM,IN1,IN2);
              setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
              setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
              setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
          }
          else if(command == '7')
          {

          pid1(250);
          pid2(250);
          setMotor(dir,pwr,PWM,IN1,IN2);
          setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
          setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
          setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
          }
   
      
      }
    }
    else 
    {
      if (millis() > 2000) 
      {
        pid1(0);
        pid2(0);
        setMotor(dir,pwr,PWM,IN1,IN2);
        setMotor(dir,pwr*133/255,Mot3_PWM,Mot3_IN1,Mot3_IN2);
        setMotor(Mot2_dir,Mot2_pwr,Mot2_PWM,Mot2_IN1,Mot2_IN2);
        setMotor2(Mot2_dir,Mot2_pwr*133/255 ,Mot4_PWM ,Mot4_IN1,Mot4_IN2 );
  }
  }
  }
}

void pid1(int v_1)
{
    //read the position in an atomic block
  // to avoid potential misreads

  /********************MOTOR 1******************************/
  int pos = 0;
  float velocity2 =0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i; 
  }

  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  /********************MOTOR 1******************************/
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;

  /********************converts counts/s to RPM******************************/
  /********************MOTOR 1******************************/
  float v1 = velocity1/408.0*60.0;      //1/G.R * 1/Counts
  float v2 = velocity2/408.0*60.0;


  /********************Low Pass Filter (25 Hz cuttoff)******************************/
  /********************MOTOR 1******************************/
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  /******************** PID MOTOR 1******************************/
  //Compute the control signal 
  float kp = 5;
  float ki = 1.2;
  float kd = 0.9;
  float e = v_1 - v1Filt;
  eintegral = eintegral + e*deltaT;
  float dedt = (e-eprev)/(deltaT);
  float u = kp*e + ki*eintegral +  kd*dedt;
          
  /******************** set the motor Speed and Direction  MOTOR 1******************************/
  if(u<0){
    dir = -1;
  }
  pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
}


void pid2(int v_2)
{
  /********************MOTOR 2******************************/ 
  int Mot2_pos = 0;
  float Mot2_velocity2 =0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    Mot2_pos = Mot2_pos_i;
    Mot2_velocity2 = Mot2_velocity_i; 
  }
  /********************MOTOR 2******************************/
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float Mot2_velocity1 = (Mot2_pos - Mot2_posPrev)/deltaT;
  Mot2_posPrev = Mot2_pos;
  
  prevT = currT;
  /********************MOTOR 2******************************/  
  float Mot2_v1 = Mot2_velocity1/408.0*60.0;
  float Mot2_v2 = Mot2_velocity1/408.0*60.0;
  /********************Low Pass Filter (25 Hz cuttoff)******************************/
  /********************MOTOR 2******************************/  
  Mot2_v1Filt = 0.854*Mot2_v1Filt + 0.0728*Mot2_v1 + 0.0728*Mot2_v1Prev;
  Mot2_v1Prev = Mot2_v1;
  Mot2_v2Filt = 0.854*Mot2_v2Filt + 0.0728*Mot2_v2 + 0.0728*Mot2_v2Prev;
  Mot2_v2Prev = Mot2_v2;

 /******************** PID MOTOR 2******************************/  
  float Mot2_kp = 10;
  float Mot2_ki = 1.2;
  float Mot2_kd = 0.9;
  float Mot2_e = v_2 - Mot2_v1Filt;
  Mot2_eintegral = Mot2_eintegral + Mot2_e * deltaT;
  float Mot2_dedt = (Mot2_e - Mot2_eprev)/(deltaT);
  float Mot2_u = Mot2_kp * Mot2_e + Mot2_ki * Mot2_eintegral + Mot2_kd*Mot2_dedt; 
  /******************** set the motor Speed and Direction  MOTOR 2******************************/  
  if(Mot2_u<0){
    Mot2_dir = -1;
  }
  Mot2_pwr = (int) fabs(Mot2_u);
  if(Mot2_pwr > 255){
    Mot2_pwr = 255;
  }
}


/********************MOTOR 1******************************/

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // Motor speed

  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder(void) {
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  } else {
    increment = -1;
  }
  pos_i = pos_i + increment;

  //compute the velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}


/********************MOTOR 2******************************/

void setMotor2(int Mot2_dir, int Mot2_pwmVal, int Mot2_pwm, int Mot2_in1, int Mot2_in2) {
  analogWrite(Mot2_pwm, Mot2_pwmVal);  // Motor speed
  if (Mot2_dir == 1) {
    // Turn one way
    digitalWrite(Mot2_in1, HIGH);
    digitalWrite(Mot2_in2, LOW);
  } else if (Mot2_dir == -1) {
    // Turn the other way
    digitalWrite(Mot2_in1, LOW);
    digitalWrite(Mot2_in2, HIGH);
  } else {
    // Or dont turn
    digitalWrite(Mot2_in1, LOW);
    digitalWrite(Mot2_in2, LOW);
  }
}

void Mot2_readEncoder(void) {
  int Mot2_b = digitalRead(Mot2_ENCB);
  int Mot2_increment = 0;
  if (Mot2_b > 0) {
    Mot2_increment = 1;
  } else {
    Mot2_increment = -1;
  }
  Mot2_pos_i = Mot2_pos_i + Mot2_increment;

  //compute the velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  Mot2_velocity_i = Mot2_increment / deltaT;
  prevT_i = currT;
}