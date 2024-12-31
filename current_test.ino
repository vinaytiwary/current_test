#define TRUE (1)
#define FALSE (0)
#define SCHEDULE_50MS_CNT  5    
#define SCHEDULE_100MS_CNT 10   // 100ms / 10ms
#define SCHEDULE_1SEC_CNT  100  // 1sec / 10ms
#define relayCW (7)
#define relayCCW (8)
#define TEST_TIMEOUT (15)
#define CURRENT_TIMEOUT  (5)
#define AUTOOFFTIMEOUT (15)

#include <Arduino.h>

volatile bool timeout = false;
volatile int elapsedtime = 0;
volatile int Curernt_count = 0;
bool runningmode = false;
volatile bool testmode = false;

typedef struct {
    unsigned char flg_10ms;
    unsigned char flg_50ms;
    unsigned char flg_100ms;
    unsigned char flg_1sec;
} schedular_flg_t;

schedular_flg_t schedular_flg;

#define SAMPLES (100)
#define AVERAGE_SAMPLES (10)
const int ACS712_PIN = A0;    // Analog pin connected to ACS712
const float VREF = 5.0;       // Arduino Uno reference voltage (in volts)
const int ADC_RES = 1024;     // ADC resolution
const float ACS712_OFFSET = 2.5; // No current offset voltage (in volts)
const float SENSITIVITY = 0.185; // Sensitivity for ACS712-05B (in volts per ampere)
//onst int relayPin = 7;
//const int pumpPin = 8;

enum WashMode { DELICATE, NORMAL, STRONG, HARD };
WashMode currentMode = DELICATE;

// Variables for 30-second operation
bool thirtySecondMode = false;
bool watercycleMode = false;

float thirtySecondSum = 0;
int thirtySecondSamples = 0;
float finalAverageCurrent = 0;
float current_rms = 0;

void TimerInit(void);
void schedularInit(void);
void togglePin(int pin);
void performMotorSequence();
int getSequenceDelay(int step);
void measureCurrent();
void getcurrent();

void setup() {
    pinMode(relayCW, OUTPUT);
    pinMode(relayCCW, OUTPUT);

    digitalWrite(relayCW, LOW);
    digitalWrite(relayCCW, LOW);
    Serial.begin(9600); // Initialize UART for serial communication
    TimerInit();
    Serial.println("ACS712 Current Measurement Initialized");
}

void loop() {
    if (Serial.available() > 0) {
        int command = Serial.read();

        switch (command) {
        case 'T':
            Serial.println("\nMOTOR ON (30 seconds)");
            digitalWrite(relayCCW, LOW);
            digitalWrite(relayCW, HIGH);
            testmode = true;
            elapsedtime = 0;
            runningmode = true;
            thirtySecondSum = 0;
            
            thirtySecondSamples = 0;
            finalAverageCurrent = 0;
            //digitalWrite(2, HIGH);
            
            break;
        default:
            break;
        }
    }

    if (schedular_flg.flg_10ms == TRUE) {
        schedular_flg.flg_10ms = FALSE;
        //Serial.print("\nFinal Average Current: ");
    }
    if (schedular_flg.flg_50ms == TRUE) {
        schedular_flg.flg_50ms = FALSE;
        if (testmode) {
            measureCurrent();
            performMotorSequence();
            delay(1);
            getcurrent();
        }
    }
    if (schedular_flg.flg_100ms == TRUE) {
        schedular_flg.flg_100ms = FALSE;
    }
    if (schedular_flg.flg_1sec == TRUE) {
        schedular_flg.flg_1sec = FALSE;
        static int aoutotimeout = 0;
        if (testmode) {
            aoutotimeout++;
            if (aoutotimeout >= AUTOOFFTIMEOUT) {
                testmode = false;
                digitalWrite(relayCW, LOW);
                digitalWrite(relayCCW, LOW);
                aoutotimeout = 0;
                finalAverageCurrent = thirtySecondSum / thirtySecondSamples;
            }
            
        }
        Serial.println("\nfinalAverageCurrent:");
        Serial.print(finalAverageCurrent, 3);
        Serial.println(" A\n");
    }
}

void TimerInit(void) {
    cli(); // Disable global interrupts
    TCCR1A = 0;   // Clear Timer/Counter1 Control Register A
    TCCR1B = 0;   // Clear Timer/Counter1 Control Register B
    OCR1A = 2499; // Set for 10ms interval
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler 64
    TIMSK1 |= (1 << OCIE1A); // Enable compare match interrupt
    sei(); // Enable global interrupts
    schedularInit();
}

void togglePin(int pin) {
    digitalWrite(pin, !digitalRead(pin));
}

void schedularInit(void) {
    schedular_flg.flg_10ms = 0;
    schedular_flg.flg_50ms = 0;
    schedular_flg.flg_100ms = 0;
    schedular_flg.flg_1sec = 0;
}

ISR(TIMER1_COMPA_vect) {
    static bool toggle = false;
    static unsigned char cnt_50ms = 0;
    static unsigned char cnt_100ms = 0;
    static unsigned int cnt_1sec = 0;

    schedular_flg.flg_10ms = 1;
    if (cnt_50ms++ >= SCHEDULE_50MS_CNT) {
        cnt_50ms = 0;
        schedular_flg.flg_50ms = 1;
    }
    if (cnt_100ms++ >= SCHEDULE_100MS_CNT) {
        cnt_100ms = 0;
        schedular_flg.flg_100ms = 1;
    }
    if (cnt_1sec++ >= SCHEDULE_1SEC_CNT) {
        cnt_1sec = 0;
        schedular_flg.flg_1sec = 1;
    }
    if(testmode)
    {
      elapsedtime +=10;
    }
    
}

void measureCurrent() {
    float sum_rms = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int sensorValue = analogRead(ACS712_PIN);
        float sensorVoltage = (sensorValue / float(ADC_RES)) * VREF;
        float current = (sensorVoltage - ACS712_OFFSET) / SENSITIVITY;
        sum_rms += (current * current);
    }
    current_rms = sqrt(sum_rms / SAMPLES);
    
}

void getcurrent() {
    if (testmode && runningmode) {
        thirtySecondSum += current_rms;
        thirtySecondSamples++;
    }
}

void performMotorSequence() {
    static int sequenceStep = 0;
    if (elapsedtime >= getSequenceDelay(sequenceStep)) {
        elapsedtime = 0;
        switch (sequenceStep) {
        case 0:
            digitalWrite(relayCCW, LOW);
            digitalWrite(relayCW, HIGH);
            runningmode = true;
            break;
        case 1:
            digitalWrite(relayCW, LOW);
            digitalWrite(relayCCW, LOW);
            runningmode = false;
            break;
        case 2:
            digitalWrite(relayCW, LOW);
            digitalWrite(relayCCW, HIGH);
            runningmode = true;
            break;
        case 3:
            digitalWrite(relayCW, LOW);
            digitalWrite(relayCCW, LOW);
            runningmode = false;
            break;
        }
        sequenceStep = (sequenceStep + 1) % 4;
    }
}

int getSequenceDelay(int step) {
    if (step == 0 || step == 2) return 2000;
    return 1000;
}
