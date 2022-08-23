//Pins
const int led = 10;
const int ldrSensor = A0;
const int usrInput = A1;

//PID variables
float Err = 0; //Current error
float pErr = 0; //Previous error

float Sp = 0; //Setpoint
float PV = 0; //Process variable (current sensor value)

float dt = 0; //Delta time

float P = 0; //Proportional gain
float kP = 6.0; //P gain multiplier

float It = 0; //Integral total gain
float kI = 0.0001; //I gain multiplier

float D = 0; //Derivative gain
float kD = 8.0; //D gain multiplier

float gain = 0; //Total gain
float cOut = 0; //Current output
float cOutAnalog = 0; //Current output analog

//Time variables
float t1 = 0;
float t2 = 0;

void setup(){
  pinMode(led, OUTPUT);
  pinMode(ldrSensor, INPUT);
  pinMode(usrInput, INPUT);
  Serial.begin(9600);
}

void loop(){
  Sp = analogRead(usrInput);
  if(Sp < 25){
    Sp = 0;
  }
  
  t2 = millis();
  dt = t2 - t1;
  t1 = millis();
  PV = analogRead(ldrSensor);
  Err = Sp - PV;

  P = kP * Err;
  
  It = It + (Err * kI * dt);
  
  D = kD * (pErr - Err) / dt;

  pErr = Err;
  
  gain = P + It + D;
  cOut = cOut + gain;

  if(isnan(cOut)){
    cOut = 0;
  } else{
    if(cOut < 0){
      cOut = 0;
      cOutAnalog = 0;
    } else{
      if(cOut > 1022){
        cOut = 1023;
        cOutAnalog = 255;
      } else{
          cOutAnalog = map(cOut, 0, 1023, 0, 255);
      }
    }
  }

  Serial.print("Sp: " + (String)Sp + " ");

  Serial.print("PV: " + (String)PV + " ");

  Serial.print("cOutAnalog: " + (String)cOutAnalog + " ");
  
  Serial.println(" ");

  analogWrite(led, cOutAnalog);
  
  delay(10);
}
