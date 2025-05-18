const int RCliff = A4;
const int FCliff = A5;
const int RtTrigPin = 11;
const int RtEchoPin = 10;
const int LtTrigPin = 9;
const int LtEchoPin = 8;
const int RrTrigPin = 7;
const int RrEchoPin = 6;
const int RtHall = 12;
const int LtHall = 13;
int32_t sensor[7];

//Ultrasonic distance sensors
int32_t distance(int trig_pin, int echo_pin){
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  int32_t dur = pulseIn(echo_pin, HIGH);
  //return (dur*.0343)/2/100; //in meters
  return dur;
  }


//IR Cliff sensors
int32_t cliff(int pin){
  //float v = analogRead(pin)*0.0048828125; //(5/1024)
  //return 13*pow(v,-1)/100;
  return analogRead(pin);
}


//Wheel speed hall sensors
int32_t speed(int hall){
  //int pw = pulseIn(hall, HIGH); //length of time in us
  //return pw; //in us converts to float
  return pulseIn(hall,HIGH,100000); //this is blocking adjust 3rd arg to speed up
}


void setup() {
  pinMode(LtTrigPin, OUTPUT);
  pinMode(LtEchoPin, INPUT);
  pinMode(RtTrigPin, OUTPUT);
  pinMode(RtEchoPin, INPUT);
  pinMode(RrTrigPin, OUTPUT);
  pinMode(RrEchoPin, INPUT);
  pinMode(RtHall, INPUT);
  pinMode(LtHall, INPUT);
  Serial.begin(115200);
}

void loop() {
  //Ultrasonic Distance
  sensor[0] = distance(LtTrigPin, LtEchoPin);
  sensor[1] = distance(RtTrigPin, RtEchoPin);
  sensor[2] = distance(RrTrigPin,RrEchoPin);

  //Cliff Sensors
  sensor[3] = cliff(FCliff);
  sensor[4] = cliff(RCliff);

  //Speed sensors
  sensor[5] = speed(RtHall);
  sensor[6] = speed(LtHall); 
  

  //Send sensor array over serial
  byte *p = (byte*)sensor;
  for(byte i =0; i < sizeof(sensor); i++) {
    Serial.write(p[i]);
  }
  

}

