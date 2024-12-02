
#define GREEN_LED 12
#define YELLOW_LED 13
#define BLUE_LED 11
#define RED_LED 10
#define START 2
#define RESET 3

volatile char state;
char previousState;

void start_button_ISR() {
  state = 'i';
}

void leds_off() {
  for (int i = 13; i >= 10; i--) {
    digitalWrite(i, LOW);
  }
}

void startSystem() {
  if (state == 'd') { 
    state = 'i';
  }
}

void setup() {
  state = 'd';
  previousState = ' ';

  // put your setup code here, to run once:
  for (int i = 13; i >= 10; i--) {
    pinMode(i, OUTPUT);
  }

  pinMode(START, INPUT);

  attachInterrupt(digitalPinToInterrupt(START), start_button_ISR, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (previousState != state) {
    leds_off();
    previousState = state;
    switch(state) {
      case 'd':
        digitalWrite(YELLOW_LED, HIGH);
        break;
      case 'r':
        digitalWrite(BLUE_LED, HIGH);
        break;
      case 'i':
        digitalWrite(GREEN_LED, HIGH);
        break;
      case 'e':
        digitalWrite(RED_LED, HIGH);
        break;

    }
  }


}
