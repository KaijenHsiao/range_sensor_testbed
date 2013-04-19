
//Sonar types: SDM-IO (trigger low), HC-SRO4 (trigger high)

#define NUM_SONARS 3
 int trigger_high[NUM_SONARS] = {0, 1, 1};  //0 = triggers upon bringing pin low and then high; 1=high then low
 unsigned long timeout_lengths[NUM_SONARS] = {10000, 10000, 10000}; //in us
 int trigger_pins[NUM_SONARS] = {13, 8, 7};
 int echo_pins[NUM_SONARS] = {12, 2, 4};
 
 int sequence_length = 3;
 int sequence_order[NUM_SONARS] = {0, 1, 2};
 unsigned long phase_length = 60000;  //in us

/*
#define NUM_SONARS 2
 int trigger_high[NUM_SONARS] = {0, 1};  //0 = triggers upon bringing pin low and then high; 1=high then low
 unsigned long timeout_lengths[NUM_SONARS] = {10000, 10000}; //in us
 int trigger_pins[NUM_SONARS] = {13, 7};
 int echo_pins[NUM_SONARS] = {12, 4};
 
 int sequence_length = 2;
 int sequence_order[NUM_SONARS] = {0, 1};
 unsigned long phase_length = 100000;  //in us
 */
/*
#define NUM_SONARS 1
//int trigger_high[NUM_SONARS] = {0};
int trigger_high[NUM_SONARS] = {1};
unsigned long timeout_lengths[NUM_SONARS] = {10000};
//int trigger_pins[NUM_SONARS] = {13};
//int echo_pins[NUM_SONARS] = {12};
int trigger_pins[NUM_SONARS] = {8};
int echo_pins[NUM_SONARS] = {2};
int sequence_length = 1;
int sequence_order[NUM_SONARS] = {0};
unsigned long phase_length = 100000;
*/

void setup() {
  Serial.begin(9600);
  for (int i=0; i<NUM_SONARS; i++) {
    if (trigger_pins[i] != -1) { //some sonars are triggered with the same trigger pin
      pinMode(trigger_pins[i], OUTPUT);
    }
    pinMode(echo_pins[i], INPUT);
    if (trigger_high[i]) digitalWrite(trigger_pins[i], LOW);
    else digitalWrite(trigger_pins[i], HIGH);

  }
  delay(100);
}

void startPingTrigger(int index){
  if (trigger_high[index]) {
    digitalWrite(trigger_pins[index], HIGH);           
  }
  else{
    digitalWrite(trigger_pins[index], LOW);
  }
}

void finishPingTrigger(int index){
  if (trigger_high[index]) {
    digitalWrite(trigger_pins[index], LOW);
  }
  else {
    digitalWrite(trigger_pins[index], HIGH);
  }
}

//returns -1 for timeout, 0 for nothing yet, 1 if echo heard (duration goes in duration)
int listenForEcho(int index, unsigned long start_time, unsigned long *duration){

  //check for timeout
  unsigned long current_time = micros();
  if (current_time - start_time > timeout_lengths[index] || current_time - start_time < 0){
    Serial.print(index);
    Serial.print(" timeout\n");
    return -1;       
  }

  //check for echo
  bool pin = digitalRead(echo_pins[index]);
  if (!pin){
    *duration = current_time - start_time;
    return 1;
  }

  return 0;
}

int waitForPinReady(int index, unsigned long start_time){
  while(1){
    bool pin = digitalRead(echo_pins[index]);
    //Serial.print(pin);
    if(pin){
      //Serial.println();
      return 1;
    }
    //check for timeout
    unsigned long current_time = micros();
    if (current_time - start_time > timeout_lengths[index] || current_time - start_time < 0){
      //Serial.println();
      Serial.print(index);
      Serial.print(" timeout waiting for pin ready\n");
      return -1;       
    }
  }
}

void loop() {

  unsigned long durations[NUM_SONARS];
  unsigned long start_times[NUM_SONARS];

  // go through the sonar sequence 
  for (int sequence_step = 0; sequence_step < sequence_length; ++sequence_step){
    unsigned long step_start = micros();   

    // trigger all the sonars in this phase
    for (int i=0; i<NUM_SONARS; i++) {
      if (sequence_order[i] == sequence_step){
        
        /*Serial.print("triggering ");
         Serial.print(i);
         Serial.println();
        */
        durations[i] = 0;
        startPingTrigger(i);
      }
    }
    delayMicroseconds(10);
    for (int i=0; i<NUM_SONARS; i++) {
      if (sequence_order[i] == sequence_step){
        finishPingTrigger(i);
        start_times[i] = micros();
      }
    }     

    // read all the sonars in this phase
    bool done = false;
    for (int i=0; i<NUM_SONARS; i++) {    
      if (sequence_order[i] == sequence_step){
        int ready = waitForPinReady(i, start_times[i]);
        if (ready) start_times[i] = micros();
        else start_times[i] = 0;
      }
    }
    while (!done) {
      done = true;
      for (int i=0; i<NUM_SONARS; i++) {
        if (sequence_order[i] == sequence_step){
          if (durations[i] == 0 && start_times[i] != 0){
            
            //durations[i] = pulseIn(echo_pins[i], HIGH);
            //int response = 1;
           
            int response = listenForEcho(i, start_times[i], &durations[i]);
            if (response == 0) done = false; 
            
            else if (response != -1){

            }
          }
        }  
      }
    }

    for (int i=0; i<NUM_SONARS; i++){
      if (sequence_order[i] == sequence_step && durations[i]){
        Serial.print(i);
        for(int j=0; j<i+1; j++) Serial.print("\t\t\t");
        //Serial.print(durations[i]);
        //Serial.print(" us, \t");
        Serial.print(durations[i]*0.017, DEC);
        Serial.print(" cm");
        Serial.println();
      }
    }
    // wait for the next phase if not over-time already
    //Serial.print("finished phase in ");
    //Serial.print(micros()-step_start);
    //Serial.println();
    unsigned long current_time = micros();
    while ((current_time - step_start) < phase_length && current_time - step_start > 0){
      delay(1);
      current_time = micros();
    }
  }
}



