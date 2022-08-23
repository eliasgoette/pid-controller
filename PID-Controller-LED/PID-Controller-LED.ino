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
float kP = 6.0; //P gain multiplier --- change value to get the best results

float It = 0; //Integral total gain
float kI = 0.0001; //I gain multiplier --- change value to get the best results

float D = 0; //Derivative gain
float kD = 8.0; //D gain multiplier --- change value to get the best results

float gain = 0; //Total gain
float cOut = 0; //Current output
float cOutAnalog = 0; //Current output analog

//Time variables
float t1 = 0;
float t2 = 0;

void setup(){
  //Setting pinmodes
  pinMode(led, OUTPUT);
  pinMode(ldrSensor, INPUT);
  pinMode(usrInput, INPUT);
  //Start USB communication
  Serial.begin(9600);
}

void loop(){
  //Read current potentiometer value
  Sp = analogRead(usrInput);
  //If its below 25 itll be considered as zero
  if(Sp < 25){
    Sp = 0;
  }
  
  
  //Get current time and calculate loop time difference
  t2 = millis();
  dt = t2 - t1;
  t1 = millis();
  
  //Read sensor value and calculate error
  PV = analogRead(ldrSensor);
  Err = Sp - PV;

  //Calculate proportional gain
  P = kP * Err;
  
  //Calculate integral gain
  It = It + (Err * kI * dt);
  
  //Calculate derivative gain
  D = kD * (pErr - Err) / dt;

  //Set the previous error for the next loop
  pErr = Err;
  
  //Add all gains together and add gain to current output
  gain = P + It + D;
  cOut = cOut + gain;

  //Set current output variable to zero if its no number
  //Make sure its between 0 and 1023 and set the analog output to the equivalent value
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

  //Send data via USB (used for graphs)
  Serial.print("Sp: " + (String)Sp + " ");
  Serial.print("PV: " + (String)PV + " ");
  Serial.print("cOutAnalog: " + (String)cOutAnalog + " ");
  Serial.println(" ");

  //Set the LEDs output to calculated value
  analogWrite(led, cOutAnalog);
  
  //Wait 10 milliseconds before restarting the loop
  delay(10);
}
