#define TRUE (1)
#define FALSE (0)
#define SCHEDULE_50MS_CNT  5    // 50ms / 10ms
#define SCHEDULE_100MS_CNT 10   // 100ms / 10ms
#define SCHEDULE_1SEC_CNT  100  // 1sec / 10ms
typedef struct 
{
    unsigned char flg_10ms;
    unsigned char flg_50ms;
    unsigned char flg_100ms;
    unsigned char flg_1sec;
}schedular_flg_t;
schedular_flg_t schedular_flg;
void timerInit(void);
void schedularInit(void);

#define SAMPLES (100)
#define AVERAGE_SAMPLES (10)
const int ACS712_PIN = A0;    // Analog pin connected to ACS712
const float VREF = 5.0;       // Arduino Uno reference voltage (in volts)
const int ADC_RES = 1024;     // ADC resolution
const float ACS712_OFFSET = 2.5; // No current offset voltage (in volts)
const float SENSITIVITY = 0.185; // Sensitivity for ACS712-05B (in volts per ampere)
const int relayPin = 7;
const int pumpPin = 8;

volatile unsigned long timerCounter = 0;
volatile unsigned long motorTimer = 0; // Time in seconds to auto-off motor
bool motorRunning = false;             // Flag to check motor state

volatile unsigned long washcyclecount = 0;

// Mode timing parameters
enum WashMode { DELICATE, NORMAL, STRONG, HARD };
WashMode currentMode = DELICATE;

float onTime = 2.5;
float offTime = 2.0;

// Buffer to store the last 10 samples of current_rms
float currentBuffer[AVERAGE_SAMPLES] = {0};
int currentIndex = 0;

// Variables for 30-second operation
bool thirtySecondMode = false;
bool watercycleMode =false;

float thirtySecondSum = 0;
int thirtySecondSamples = 0;
float finalAverageCurrent =0;

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  digitalWrite(pumpPin, LOW);
  Serial.begin(9600); // Initialize UART for serial communication
 // timerInit();
  schedularInit();
  Serial.println("ACS712 Current Measurement Initialized");
}

void loop() {
  // Read analog value from ACS712
  float sum_rms = 0;
  float current_rms = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int sensorValue = analogRead(ACS712_PIN);
    // Convert analog value to voltage
    float sensorVoltage = (sensorValue / float(ADC_RES)) * VREF;
    // Calculate current (I = (Vout - Voffset) / Sensitivity)
    float current = (sensorVoltage - ACS712_OFFSET) / SENSITIVITY;
    sum_rms += (current * current);
  }
  current_rms = sqrt(sum_rms / SAMPLES);
  // If in 30-second mode, accumulate current data
  if (thirtySecondMode) {
    thirtySecondSum += current_rms;
    thirtySecondSamples++;
  }

  Serial.print("\nMOTOR OFF (30 seconds elapsed)");
  Serial.print("\nFinal Average Current: ");
  Serial.print(finalAverageCurrent, 3);
  Serial.println(" A\n");
  Serial.print("\nMotor time: ");
  Serial.print(motorTimer);
  delay(500); // Delay for readability

  if (Serial.available() > 0) {
    int command = Serial.read();

    switch (command) {
      case 'T':
        //digitalWrite(relayPin, HIGH);
       // thirtySecondMode = true;
        //thirtySecondSum = 0;
        //thirtySecondSamples = 0;
        //timerCounter = 0; // Reset timer for 30 seconds
        Serial.println("\nMOTOR ON (30 seconds)");
        break;
      default:
        
        break;
    }
  }
   if(schedular_flg.flg_10ms == TRUE)
   {
     schedular_flg.flg_10ms = FALSE;       
   }
   if(schedular_flg.flg_50ms == TRUE)
   {
     schedular_flg.flg_50ms = FALSE;
   }     
   if (schedular_flg.flg_100ms == TRUE)
   {
    schedular_flg.flg_100ms = FALSE;
   }
   if (schedular_flg.flg_1sec == TRUE)
   {
    schedular_flg.flg_1sec = FALSE;
   }
   
}
void timerInit(void) {
    cli(); // Disable global interrupts

    TCCR1A = 0;   // Clear Timer/Counter1 Control Register A
    TCCR1B = 0;   // Clear Timer/Counter1 Control Register B
    OCR1A = 2499; // Set for 10ms interval
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler 64
    TIMSK1 |= (1 << OCIE1A); // Enable compare match interrupt

    sei(); // Enable global interrupts
    schedularInit();
}

void schedularInit(void) {
    schedular_flg.flg_10ms = 0;
    schedular_flg.flg_50ms = 0;
    schedular_flg.flg_100ms = 0;
    schedular_flg.flg_1sec = 0;
}
ISR(TIMER1_COMPA_vect) {
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
}
