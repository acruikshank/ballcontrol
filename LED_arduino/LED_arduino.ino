int states[12][3] = {
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
int PINS[3] = { 2, 3, 4 };
int state = 0;

void setup() {
  Serial.begin(9600);
  setLEDstate(0);
}

void setLEDstate(int state) {
  for (int i=0; i<3; i++) {
    if (states[state][i] >= 0) {
      pinMode(PINS[i],OUTPUT);
      Serial.println("Setting pin " + String(PINS[i]) + " to " + (states[state][i] == 0 ? "LOW" : "HIGH"));
      digitalWrite(PINS[i], states[state][i] == 0 ? LOW : HIGH);
    } else {
      Serial.println("Setting pin " + String(PINS[i]) + " to INPUT");
      pinMode(PINS[i],INPUT);
    }
  }
  pinMode(1,INPUT);
}

void loop() {
  setLEDstate(state);
  delay(1000);
  state = (state + 1) % 12;
}
