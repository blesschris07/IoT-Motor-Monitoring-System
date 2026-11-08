// Smart Industrial IoT Motor Monitoring System
// Controller: Arduino UNO

#define LM35_PIN A0
#define ACS712_PIN A1
#define HALL_PIN 2

volatile unsigned long pulseCount = 0;

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);

  pinMode(HALL_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(HALL_PIN),
    countPulse,
    FALLING
  );

  Serial.println("Motor Monitoring System Started");
}

void loop() {
  int temperatureRaw = analogRead(LM35_PIN);
  int currentRaw = analogRead(ACS712_PIN);

  noInterrupts();
  unsigned long rpmPulses = pulseCount;
  pulseCount = 0;
  interrupts();

  Serial.print("Temperature Sensor Raw: ");
  Serial.println(temperatureRaw);

  Serial.print("Current Sensor Raw: ");
  Serial.println(currentRaw);

  Serial.print("Hall Sensor Pulses: ");
  Serial.println(rpmPulses);

  Serial.println("----------------------");

  delay(1000);
}
