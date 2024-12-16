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
#define LOWTEMP 25
#define HIGHTEMP 27
#define WATER_SENSOR_PIN A0
#define RS 8
#define EN 13
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define STEPPER_1N1 9
#define STEPPER_1N2 10
#define STEPPER_1N3 11
#define STEPPER_1N4 12
#define ENA_PIN 8  //  motor speed (PWM control)   all of this are form the link from modules
#define IN1_PIN 53  // direction 1
#define IN2_PIN 52  //  direction 2

const int stepsPerRevolution = 2038;
Stepper stepper = Stepper(stepsPerRevolution, STEPPER_1N1, STEPPER_1N3, STEPPER_1N2, STEPPER_1N4);

#define ENA_PIN 9  //  motor speed (PWM control)   all of this are form the link from modules
#define IN1_PIN 6  // direction 1
#define IN2_PIN 5  //  direction 2


volatile char state;
char previousState;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

dht DHT;
// DHT dhDHTPIN, DHTTYPE;
// dht dhDHTPIN, DHTTYPE;
void start_button_ISR() {
  state = 'i';
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
    bit = pin - 14; // PF0–PF7


  } else if (pin >= A8 && pin <= A15) { // A8–A15 (PORTK)
    port = &PORTK;
    bit = pin - 22; // PK0–PK7


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
  mydigitalWrite(IN2_PIN, LOW);   
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
unsigned vent_position = 0;
void vent_button_ISR() {
  vent_state = !vent_state;
  Serial.println(vent_state);
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

void setup() {
  
  myUARTBegin(9600);
  adc_init();
  //Serial.begin(9600);
  // Test Code
  myUARTPrint("Print function works");
  U0putchar('\n');
  rtc.begin();

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

  // Test Code //
  // stepper.setSpeed(10);
  // stepper.step(stepsPerRevolution);

  motorStart(254);
  // Test Code //

}

void loop() {

  if (vent_state && vent_position < VENT_MAX_POS) {
    stepper.setSpeed(10);
    stepper.step(10);
    vent_position += 10;
  } else if (!vent_state && vent_position > 0) {
    stepper.setSpeed(10);
    stepper.step(-10);
    vent_position -= 10;
  }
  
  unsigned testValue = adc_read(WATER_SENSOR_PIN);
  //Serial.println(testValue);


  if (previousState != state) {
    leds_off();
    previousState = state;
    switch(state) {
      case 'd':
        myDigitalWrite(YELLOW_LED, HIGH);
        break;
      case 'r':
        myDigitalWrite(BLUE_LED, HIGH);
        break;
      case 'i':
        myDigitalWrite(GREEN_LED, HIGH);
        break;
      case 'e':
        myDigitalWrite(RED_LED, HIGH);
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

