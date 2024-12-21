// Define sensor and calibration parameters
#define SAMPLES (100)
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

void timer_init() {
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 24999;
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  digitalWrite(pumpPin, LOW);
  Serial.begin(9600); // Initialize UART for serial communication
  timer_init();
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

  // Print current value
  Serial.print("\nCurrent: ");
  Serial.print(current_rms, 3); // Print with 3 decimal precision
  Serial.println(" A");
  Serial.print("\nMotor time: ");
  Serial.print(motorTimer);
  delay(500); // Delay for readability

  if (Serial.available() > 0) {
    int command = Serial.read();

    if (command == 'A') {
      digitalWrite(relayPin, HIGH);
      motorRunning = true;
      Serial.println("\nMOTOR ON");
    } else if (command == 'B') {
      digitalWrite(relayPin, LOW);
      motorRunning = false;
      motorTimer = 0;
      Serial.println("\nMOTOR OFF");
    } else if (command == 'C') {
      digitalWrite(pumpPin, HIGH);
      Serial.println("\nPUMP ON");
    } else if (command == 'D') {
      digitalWrite(pumpPin, LOW);
      Serial.println("\nPUMP OFF");
    } else {
      motorTimer = command; // Interpret the received value as seconds
      Serial.print("\nMotor Timer Set to: ");
      Serial.print(motorTimer);
      Serial.println(" seconds");
    }
  }
}

ISR(TIMER1_COMPA_vect) {
  timerCounter++;

  if (motorRunning && motorTimer > 0 && timerCounter % 10 == 0) { // Decrement every second
    motorTimer--;
    Serial.print("\nMotor Timer Remaining: ");
    Serial.print(motorTimer);
    Serial.println(" seconds");

    if (motorTimer == 0) {
      digitalWrite(relayPin, LOW);
      motorRunning = false;
      Serial.println("\nMOTOR AUTO-OFF");
    }
  }
}