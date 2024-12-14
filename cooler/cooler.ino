#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#define GREEN_LED 24
#define YELLOW_LED 22
#define BLUE_LED 26
#define RED_LED 28
#define START 2
#define RESET 3
#define STOP 18
#define DHTPIN A1 //water + hum sensor
#define LOWTEMP 20
#define HIGHTEMP 30
#define WATER_SENSOR_PIN A0
#define RS 23
#define EN 13
#define D4 4
#define D5 5
#define D6 6
#define D7 25
#define STEPPER_1N1 9
#define STEPPER_1N2 10
#define STEPPER_1N3 11
#define STEPPER_1N4 12
#define ENA_PIN 8  //  motor speed (PWM control)   all of this are form the link from modules
#define IN1_PIN 53  // direction 1
#define IN2_PIN 52  //  direction 2
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void myUARTBegin(unsigned long U0baud)  {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

void myUARTPrint(char* printedString) {
  int i = 0;
  while(printedString[i] != '\0') {
    U0putchar(printedString[i]);
    i++;
  }
}

void U0putchar(unsigned char U0pdata) {
  while(!(*myUCSR0A & TBE));
  *myUDR0 = U0pdata;
}

const int stepsPerRevolution = 2038;
Stepper stepper = Stepper(stepsPerRevolution, STEPPER_1N1, STEPPER_1N3, STEPPER_1N2, STEPPER_1N4);


volatile char state;
char previousState;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

dht DHT;
// DHT dhDHTPIN, DHTTYPE;
// dht dhDHTPIN, DHTTYPE;
void start_button_ISR() {
  state = 'i';
}

void motorStart(int speed) {
  digitalWrite(IN1_PIN, HIGH);  
  digitalWrite(IN2_PIN, LOW);   
  analogWrite(ENA_PIN, speed);  
}

void motorStop() {
  analogWrite(ENA_PIN, 0);  // stop 
}

void reset_button_ISR() {
  if (state == 'e') {
    state = 'i';
  }
}

void stop_button_ISR() {
  state = 'd';
}

void leds_off() {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}

void startSystem() {
  if (state == 'd') { 
    state = 'i';
  }
}

void setup() {

  myUARTBegin(9600);
  //Serial.begin(9600);
  // Test Code
  myUARTPrint("Print function works");
  U0putchar('\n');

  state = 'd';
  previousState = ' ';

  // put your setup code here, to run once:
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  pinMode(START, INPUT_PULLUP);
  pinMode(RESET, INPUT_PULLUP);
  pinMode(STOP, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RESET), reset_button_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(START), start_button_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP), stop_button_ISR, FALLING);

  pinMode(DHTPIN, INPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);
  
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Evaporation");
  lcd.setCursor(0, 1);
  lcd.print("Cooling System");

  // Test Code //
  // stepper.setSpeed(10);
  // stepper.step(stepsPerRevolution);

  motorStart(254);
  // Test Code //

}

void loop() {
  
  // Test code //
  int testValue = analogRead(WATER_SENSOR_PIN);
  //Serial.println(testValue);
  // Test code //


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

  if (state != 'd') {  
    int chk = DHT.read11(DHTPIN);

    float temperature = DHT.temperature;  // this should be in celsius
    float humidity = DHT.humidity;  // this reads humidity
    
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("dht is not reading right ");
      return;

     // https://www.circuitbasics.com/how-to-set-up-the-dht11-humidity-sensor-on-an-arduino/
    }
    /*Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println("%");*/

    if (state == 'r' && temperature < LOWTEMP) {
      state = 'i'; 
    } else if (state == 'i' && temperature > HIGHTEMP) {
      state = 'r';
    }

    /*lcd.setCursor(0, 0);
    lcd.print("Temperature: ");
    lcd.print(temperature);
    
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);*/
  }
}
