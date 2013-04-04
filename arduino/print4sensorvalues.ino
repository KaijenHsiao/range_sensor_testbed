/*
  Read 2 IRs and 2 sonars 
 */

int sonar1SwitchPin = 13;
int sonar2SwitchPin = 12;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
  pinMode(sonar1SwitchPin, OUTPUT);
  pinMode(sonar2SwitchPin, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {

  // read the two IRs  
  int sensorValue0 = analogRead(A0);
  int sensorValue1 = analogRead(A1);

  // switch on the left sonar and read the left sonar
  digitalWrite(sonar1SwitchPin, HIGH);
  digitalWrite(sonar2SwitchPin, LOW);
  delay(10);
  int sensorValue2 = analogRead(A2);
  
  // switch on the right sonar and read the right sonar
  digitalWrite(sonar1SwitchPin, LOW);
  digitalWrite(sonar2SwitchPin, HIGH);
  delay(10);
  int sensorValue3 = analogRead(A3);
  
  // print out the values read:
  Serial.print(sensorValue0);
  Serial.print("\t");
  Serial.print(sensorValue1);
  Serial.print("\t");
  Serial.print(sensorValue2);
  Serial.print("\t");
  Serial.print(sensorValue3);
  Serial.print("\n");
  delay(80);        // delay in between reads for stability
}

