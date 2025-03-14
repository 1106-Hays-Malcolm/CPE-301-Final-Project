//Project: Water Evaporation Cooling System
//Authors: Aween Ali, Malcolm Hays, Derek Ngu, Thomas Vasile

#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#define GREEN_LED 24
#define YELLOW_LED 22
#define BLUE_LED 26
#define RED_LED 28
#define START 2
#define RESET 3
#define STOP 18
#define DHTPIN A1 //water + hum sensor
#define LOWTEMP 26
#define HIGHTEMP 27
#define WATER_SENSOR_PIN 0 // Pin A0
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
#define VENT_ADJUST_BUTTON 19
#define VENT_MAX_POS 500

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;


volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// this is for adc read ^^

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

#define DDRB_ADDR 0x24
#define DDRH_ADDR 0x101
#define DDRE_ADDR 0x2D
#define DDRG_ADDR 0x33
#define DDRJ_ADDR 0x107
#define DDRD_ADDR 0x2A
#define DDRA_ADDR 0x21

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Thursday", "Friday", "Saturday"};

const int stepsPerRevolution = 2038;
Stepper stepper = Stepper(stepsPerRevolution, STEPPER_1N1, STEPPER_1N3, STEPPER_1N2, STEPPER_1N4);


volatile char state;
char previousState;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

dht DHT;
void start_button_ISR() {
  if (state != 'e') {
    state = 'i';
  }
}



void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}



unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void myDigitalWrite(uint8_t pin, bool value) {
  volatile uint8_t* port;
  uint8_t bit;

if (pin == 0) {                // Pin D0
    port = &PORTE;
    bit = 0;                   // PE0
} else if (pin == 1) {         // Pin D1
    port = &PORTE;
    bit = 1;                   // PE1
} else if (pin == 2) {         // Pin D2
    port = &PORTE;
    bit = 4;                   // PE4
} else if (pin == 3) {         // Pin D3
    port = &PORTE;
    bit = 5;                   // PE5
} else if (pin == 4) {         // Pin D4
    port = &PORTG;
    bit = 5;                   // PG5
} else if (pin == 5) {         // Pin D5
    port = &PORTE;
    bit = 3;                   // PE3
} else if (pin == 6) {         // Pin D6
    port = &PORTH;
    bit = 3;                   // PH3
} else if (pin == 7) {         // Pin D7
    port = &PORTH;
    bit = 4;                   // PH4

}else if (pin == 8) {              // Pin D8 (PORTH)
    port = &PORTH;
    bit = 5; // PH5


  } else if (pin == 9) {              // Pin D9 (PORTH)
    port = &PORTH;
    bit = 6; // PH6


  } else if (pin >= 10 && pin <= 13) { // Pins 10–13 (PORTB)
    port = &PORTB;
    bit = pin - 6; // PB4–PB7

}else if (pin == 14) {                // D14/TX3
    port = &PORTJ;
    bit = 1;                    // PJ1

    
} else if (pin == 15) {         // D15/RX3
    port = &PORTJ;
    bit = 0;                    // PJ0


} else if (pin == 16) {         // D16/TX2
    port = &PORTH;
    bit = 1;                    // PH1


} else if (pin == 17) {         // D17/RX2
    port = &PORTH;
    bit = 0;                    // PH0


} else if (pin == 18) {         // D18/TX1
    port = &PORTD;
    bit = 3;                    // PD3


} else if (pin == 19) {         // D19/RX1
    port = &PORTD;
    bit = 2;                    // PD2


} else if (pin == 20) {         // D20/SDA
    port = &PORTD;
    bit = 1;                    // PD1


} else if (pin == 21) {         // D21/SCL
    port = &PORTD;
    bit = 0;                    // PD0

}else if (pin >= 22 && pin <= 29) { // Pins 22–29 (PORTA)
    port = &PORTA;
    bit = pin - 22; // PA0–PA7


}else if (pin == 30) {                // Pin D30
    port = &PORTC;
    bit = 7;                    // PC7

} else if (pin == 31) {         // Pin D31
    port = &PORTC;
    bit = 6;                    // PC6

} else if (pin == 32) {         // Pin D32
    port = &PORTC;
    bit = 5;                    // PC5

} else if (pin == 33) {         // Pin D33
    port = &PORTC;
    bit = 4;                    // PC4

} else if (pin == 34) {         // Pin D34
    port = &PORTC;
    bit = 3;                    // PC3

} else if (pin == 35) {         // Pin D35
    port = &PORTC;
    bit = 2;                    // PC2

} else if (pin == 36) {         // Pin D36
    port = &PORTC;
    bit = 1;                    // PC1

} else if (pin == 37) {         // Pin D37
    port = &PORTC;
    bit = 0;                    // PC0

} else if (pin == 38) {         // Pin D38
    port = &PORTD;
    bit = 7;                    // PD7

} else if (pin == 39) {         // Pin D39
    port = &PORTG;
    bit = 2;                    // PG2

} else if (pin == 40) {         // Pin D40
    port = &PORTG;
    bit = 1;                    // PG1

} else if (pin == 41) {         // Pin D41
    port = &PORTG;
    bit = 0;                    // PG0

} else if (pin == 42) {         // Pin D42
    port = &PORTL;
    bit = 7;                    // PL7

} else if (pin == 43) {         // Pin D43
    port = &PORTL;
    bit = 6;                    // PL6

} else if (pin == 44) {         // Pin D44
    port = &PORTL;
    bit = 5;                    // PL5

} else if (pin == 45) {         // Pin D45
    port = &PORTL;
    bit = 4;                    // PL4

} else if (pin == 46) {         // Pin D46
    port = &PORTL;
    bit = 3;                    // PL3

} else if (pin == 47) {         // Pin D47
    port = &PORTL;
    bit = 2;                    // PL2

} else if (pin == 48) {         // Pin D48
    port = &PORTL;
    bit = 1;                    // PL1

} else if (pin == 49) {         // Pin D49
    port = &PORTL;
    bit = 0;                    // PL0

} else if (pin == 50) {         // Pin D50
    port = &PORTB;
    bit = 3;                    // PB3

} else if (pin == 51) {         // Pin D51
    port = &PORTB;
    bit = 2;                    // PB2

} else if (pin == 52) {         // Pin D52
    port = &PORTB;
    bit = 1;                    // PB1

} else if (pin == 53) {         // Pin D53
    port = &PORTB;
    bit = 0;                    // PB0

} else if (pin >= A0 && pin <= A7) { // A0–A7 (PORTF)
    port = &PORTF;
    bit = pin - A0; // PF0–PF7


  } else if (pin >= A8 && pin <= A15) { // A8–A15 (PORTK)
    port = &PORTK;
    bit = pin - A8; // PK0–PK7


  } else {
    return; 
  }

  if (value) {
    *port |= (1 << bit); 
  } else {
    *port &= ~(1 << bit); 
  }
}

void motorStart(int speed) {
  myDigitalWrite(IN1_PIN, HIGH);  
  myDigitalWrite(IN2_PIN, LOW);   
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

bool vent_state = 0;
bool print_vent_message = 0;
unsigned vent_position = 0;
void vent_button_ISR() {
  vent_state = !vent_state;
  print_vent_message = 1;
}

void leds_off() {
  myDigitalWrite(GREEN_LED, LOW);
  myDigitalWrite(YELLOW_LED, LOW);
  myDigitalWrite(RED_LED, LOW);
  myDigitalWrite(BLUE_LED, LOW);
}

void startSystem() {
  if (state == 'd') { 
    state = 'i';
  }
}

unsigned pinNumberToBitNumber(unsigned pinNumber) {
  switch(pinNumber) {
    case 13:
      return 7;
    case 12:
      return 6;
    case 11:
      return 5;
    case 10:
      return 4;
    case 9:
      return 6;
    case 8:
      return 5;
    case 7:
      return 4;
    case 6:
      return 3;
    case 5:
      return 3;
    case 4:
      return 5;
    case 3:
      return 5;
    case 2:
      return 4;
    case 1:
      return 7;
    case 0:
      return 0;
    case 14:
      return 1;
    case 15:
      return 0;
    case 16:
      return 1;
    case 17:
      return 0;
    case 18:
      return 3;
    case 19:
      return 2;
    case 20:
      return 1;
    case 21:
      return 0;
    case 22:
      return 0;
    case 23:
      return 1;
    case 24:
      return 2;
    case 25:
      return 3;
    case 26:
      return 4;
    case 27:
      return 5;
    case 28:
      return 6;
    case 29:
      return 7;
  }
}

unsigned char* pinNumberToDdrAddress(unsigned pinNumber) {
  switch (pinNumber) {

    // DDRB
    case 13:
    case 12:
    case 11:
    case 10:
      return DDRB_ADDR;
    
    // DDRH
    case 9:
    case 8:
    case 7:
    case 6:
    case 16:
    case 17:
      return DDRH_ADDR;
    
    // DDRE
    case 5:
    case 3:
    case 2:
    case 1:
    case 0:
      return DDRE_ADDR;

    // DDRG
    case 4:
    case 39:
    case 40:
    case 41:
      return DDRG_ADDR;

    // DDRJ
    case 14:
    case 15:
      return DDRJ_ADDR;

    // DDRD
    case 18:
    case 19:
    case 20:
    case 21:
      return DDRD_ADDR;

    // DDRA
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
      return DDRA_ADDR;
  }
}

void myPinMode(uint8_t pinNumber, uint8_t mode) {
  volatile uint8_t* myDdrRegister = pinNumberToDdrAddress(pinNumber);
  volatile uint8_t* myPortRegister = (uint8_t)(pinNumberToDdrAddress(pinNumber) + 1);
  uint8_t myBitNumber = pinNumberToBitNumber(pinNumber);

  if (mode == OUTPUT) {
    *myDdrRegister |= (1 << myBitNumber);
  } else if (mode == INPUT) {
    *myDdrRegister &= ~(1 << myBitNumber);
  } else if (mode == INPUT_PULLUP) {
    *myDdrRegister &= ~(1 << myBitNumber);
    *myPortRegister |= (1 << myBitNumber);
  }
}

void printTimeAndDate() {
  DateTime now = rtc.now();
  char dateString[21];
  int y = now.year();
  int mon = now.month();
  int d = now.day();
  int h = now.hour();
  int min = now.minute();
  int s = now.second();
  int i = 3;
  while(y > 0) {
    dateString[i] = (y % 10) + 48;
    y = y / 10;
    i--;
  }
  dateString[4] = '/';
  dateString[5] = (mon / 10) + 48;
  dateString[6] = (mon % 10) + 48;
  dateString[7] = '/';
  dateString[8] = (d / 10) + 48;
  dateString[9] = (d % 10) + 48;
  dateString[10] = ' ';
  dateString[11] = ' ';
  dateString[12] = (h / 10) + 48;
  dateString[13] = (h % 10) + 48;
  dateString[14] = ':';
  dateString[15] = (min / 10) + 48;
  dateString[16] = (min % 10) + 48;
  dateString[17] = ':';
  dateString[18] = (s / 10) + 48;
  dateString[19] = (s % 10) + 48;
  dateString[20] = '\0';
  myUARTPrint(dateString);
}

unsigned long previousMillis = 0;
const long lcdInterval = 60000; //shows updates to humidity/temp every minute

void setup() {
  myUARTBegin(9600);

  rtc.begin();

  adc_init();

  state = 'd';
  previousState = ' ';

  // put your setup code here, to run once:
  myPinMode(GREEN_LED, OUTPUT);
  myPinMode(RED_LED, OUTPUT);
  myPinMode(YELLOW_LED, OUTPUT);
  myPinMode(BLUE_LED, OUTPUT);

  myPinMode(START, INPUT_PULLUP);
  myPinMode(RESET, INPUT_PULLUP);
  myPinMode(STOP, INPUT_PULLUP);
  myPinMode(VENT_ADJUST_BUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RESET), reset_button_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(START), start_button_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP), stop_button_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(VENT_ADJUST_BUTTON), vent_button_ISR, FALLING);

  myPinMode(DHTPIN, INPUT);
  myPinMode(WATER_SENSOR_PIN, INPUT);
  
  myPinMode(ENA_PIN, OUTPUT);
  myPinMode(IN1_PIN, OUTPUT);
  myPinMode(IN2_PIN, OUTPUT);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Evaporation");
  lcd.setCursor(0, 1);
  lcd.print("Cooling System");
}

void loop() {

  if (vent_state && vent_position < VENT_MAX_POS && state != 'd') {
    stepper.setSpeed(10);
    stepper.step(10);
    vent_position += 10;
  } else if (!vent_state && vent_position > 0 && state != 'd') {
    stepper.setSpeed(10);
    stepper.step(-10);
    vent_position -= 10;
  } else {
    myDigitalWrite(STEPPER_1N1, LOW);
    myDigitalWrite(STEPPER_1N2, LOW);
    myDigitalWrite(STEPPER_1N3, LOW);
    myDigitalWrite(STEPPER_1N4, LOW);
  }

  if (print_vent_message && state != 'd') {
    print_vent_message = 0;
    myUARTPrint("Stepper Motor/Vent Movement on ");
    printTimeAndDate();
    U0putchar('\n');
  }
  
  unsigned water_sensor_value = adc_read(WATER_SENSOR_PIN);


  if (previousState != state) {
    leds_off();
    previousState = state;
    switch(state) {
      case 'd':
        motorStop();
        myDigitalWrite(YELLOW_LED, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Evaporation");
        lcd.setCursor(0, 1);
        lcd.print("Cooling System");
        previousMillis = 0;
        myUARTPrint("State Transition to Disabled on ");
        printTimeAndDate();
        U0putchar('\n');
        break;
      case 'r':
        motorStart(254);
        myDigitalWrite(BLUE_LED, HIGH);
        myUARTPrint("State Transition to Running on ");
        printTimeAndDate();
        U0putchar('\n');
        break;
      case 'i':
        motorStop();
        myDigitalWrite(GREEN_LED, HIGH);
        myUARTPrint("State Transition to Idle on ");
        printTimeAndDate();
        U0putchar('\n');
        break;
      case 'e':
        motorStop();
        myDigitalWrite(RED_LED, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Error! Water");
        lcd.setCursor(0, 1);
        lcd.print("level is too low");
        previousMillis = 0;
        myUARTPrint("State Transition to Error on ");
        printTimeAndDate();
        U0putchar('\n');
        break;
    }
  }

  if (state != 'd' && state != 'e') {
    if (water_sensor_value < 5) {
      state = 'e';
    }

    int chk = DHT.read11(DHTPIN);

    float temperature = DHT.temperature;  // this should be in celsius
    float humidity = DHT.humidity;  // this reads humidity
    
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= lcdInterval || previousMillis == 0) {
      previousMillis = currentMillis;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
    }

    if (state == 'r' && temperature < LOWTEMP) {
      state = 'i'; 
    } else if (state == 'i' && temperature > HIGHTEMP) {
      state = 'r';
    }
  }
}