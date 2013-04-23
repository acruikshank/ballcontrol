int MOTOR_PIN[3][2] = { {3,5}, {6,9}, {10,11} };
int LED_STATES[13][3] = {
  {-1,-1,-1},
  {1,0,-1},
  {-1,0,1},
  {0,-1,1},
  {0,1,-1},
  {-1,1,0},
  {1,-1,0},
  {1,0,1},
  {0,0,1},
  {0,1,1},
  {0,1,0},
  {1,1,0},
  {1,0,0}
};
int LED_PINS[3] = { 2, 4, 7 };

void setup() {
  // initialize serial:
  Serial.begin(9600);
  for (int i=0; i<3; i++) {
    pinMode(MOTOR_PIN[i][0],OUTPUT);
    analogWrite(MOTOR_PIN[i][0],0);
    pinMode(MOTOR_PIN[i][1],OUTPUT);
    analogWrite(MOTOR_PIN[i][1],0);
  }
  setLEDState(0);
}

void loop() {
}

/*
 Control byte: 1MMDHCCC
 Value byte:   0PPPPPPP
 M = Which motor
 D = Motor direction
 H = High bit of power
 C = Checksum
 P = Power value
 
 Checksum = number of ones in value byte
 Power = H*127 + value byte
 */
boolean control = true;
int motor = 0;
int forward = 1;
int powerHigh = 0;
int checksum = 0;

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    byte in = Serial.read();
    
    if (control) {
      if (in & 0x80 == 0) {
        error();
        continue;
      }
      
      motor = (in & 0x60) >> 5;
      forward = (in & 0x10) >> 4;
      powerHigh = ((in & 0x8) << 4);
      checksum = in & 0x7;
      control = false;
    } else {
      if (in & 0x80 == 0) {
        error();
        continue;
      }
      
      int bits = 0;
      for (int i=0; i<8; i++)
        bits += (in >> i) & 0x1;
        
      if (bits != checksum) {
        error();
        continue;
      }
      
      if ( motor == 3 )
        setLEDState( in );
      else
        setPower( motor, forward, powerHigh + in );      
  
      control = true;
    }
  }
}

void setPower( int motor, int forward, int power ) {
  if ( power == 0 ) {
    digitalWrite(MOTOR_PIN[motor][0],LOW);
    digitalWrite(MOTOR_PIN[motor][1],LOW);
  } else if ( forward > 0 ) {
    analogWrite(MOTOR_PIN[motor][0],power);
    digitalWrite(MOTOR_PIN[motor][1],LOW);
  } else {
    digitalWrite(MOTOR_PIN[motor][0],LOW);
    analogWrite(MOTOR_PIN[motor][1],power);
  }
}

void setLEDState(int state) {
  Serial.println(state);
  for (int i=0; i<3; i++) {
    if (LED_STATES[state][i] >= 0) {
      pinMode(LED_PINS[i],OUTPUT);
      digitalWrite(LED_PINS[i], LED_STATES[state][i] == 0 ? LOW : HIGH);
    } else {
      pinMode(LED_PINS[i],INPUT);
    }
  }
}

void error() {
  control = true;
  for (int i=0; i<3; i++) {
    digitalWrite(MOTOR_PIN[i][0],LOW);
    digitalWrite(MOTOR_PIN[i][1],LOW); 
  }
  Serial.println("error");
}


