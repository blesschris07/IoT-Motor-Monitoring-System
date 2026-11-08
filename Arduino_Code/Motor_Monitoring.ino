#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// =====================================================
// MOTOR MONITORING SYSTEM
// Arduino Uno
// =====================================================

// =====================================================
// PIN DEFINITIONS
// =====================================================

const int LM35_PIN   = A0;
const int ACS712_PIN = A1;

const int HALL_PIN   = 2;
const int RELAY_PIN  = 7;
const int BUZZER_PIN = 8;

// =====================================================
// I2C DEVICES
// =====================================================

const int MPU_ADDR = 0x68;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// EASY-TO-CHANGE ALARM THRESHOLDS
// =====================================================

const float TEMP_LIMIT      = 45.0;   // °C
const float CURRENT_LIMIT   = 0.50;   // A
const float RPM_LOW_LIMIT   = 90.0;   // RPM
const float RPM_HIGH_LIMIT  = 150.0;  // RPM
const float VIBRATION_LIMIT = 0.800;  // g

// =====================================================
// ACS712 CALIBRATION
// =====================================================

const float ACS_OFFSET      = 2.526;  // Zero-current voltage
const float ACS_SENSITIVITY = 0.185; // V/A

// =====================================================
// MPU6050 VARIABLES
// =====================================================

int16_t AcX, AcY, AcZ;
int16_t Tmp;
int16_t GyX, GyY, GyZ;

float vibration = 0.0;

// =====================================================
// RPM VARIABLES
// =====================================================

volatile unsigned long pulseCount = 0;

unsigned long previousRPMTime = 0;
float rpm = 0.0;

// =====================================================
// SENSOR VALUES
// =====================================================

float temperature = 0.0;
float current = 0.0;

// =====================================================
// STATUS
// =====================================================

bool warning = false;

// =====================================================
// LCD PAGE CONTROL
// =====================================================

unsigned long previousLCDTime = 0;

const unsigned long LCD_PAGE_TIME = 2000;

int lcdPage = 0;

// =====================================================
// BUZZER CONTROL
// =====================================================

unsigned long previousBuzzerTime = 0;

bool buzzerState = false;

const unsigned long BUZZER_INTERVAL = 500;

// =====================================================
// RELAY CONTROL
// =====================================================

const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;

// =====================================================
// HALL SENSOR INTERRUPT
// =====================================================

void hallPulse()
{
  pulseCount++;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // -------------------------
  // Pin setup
  // -------------------------

  pinMode(HALL_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, RELAY_OFF);
  noTone(BUZZER_PIN);

  // -------------------------
  // Hall sensor interrupt
  // -------------------------

  attachInterrupt(
    digitalPinToInterrupt(HALL_PIN),
    hallPulse,
    FALLING
  );

  // -------------------------
  // I2C
  // -------------------------

  Wire.begin();

  // -------------------------
  // LCD
  // -------------------------

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MOTOR MONITOR");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  
  delay(1500);

  // -------------------------
  // MPU6050 wake-up
  // -------------------------

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);

  byte error = Wire.endTransmission();

  lcd.clear();

  if (error == 0)
  {
    Serial.println("MPU6050 detected at 0x68");

    lcd.setCursor(0, 0);
    lcd.print("MPU6050 OK");
  }
  else
  {
    Serial.println("MPU6050 communication error!");

    lcd.setCursor(0, 0);
    lcd.print("MPU ERROR");
  }

  delay(1000);

  // -------------------------
  // Serial startup message
  // -------------------------

  Serial.println();
  Serial.println("================================");
  Serial.println("     MOTOR MONITORING SYSTEM");
  Serial.println("================================");

  Serial.println("Thresholds:");
  Serial.print("Temperature : ");
  Serial.print(TEMP_LIMIT);
  Serial.println(" C");

  Serial.print("Current     : ");
  Serial.print(CURRENT_LIMIT);
  Serial.println(" A");

  Serial.print("RPM Low     : ");
  Serial.print(RPM_LOW_LIMIT);
  Serial.println(" RPM");

  Serial.print("RPM High    : ");
  Serial.print(RPM_HIGH_LIMIT);
  Serial.println(" RPM");

  Serial.print("Vibration   : ");
  Serial.print(VIBRATION_LIMIT);
  Serial.println(" g");

  Serial.println("================================");
  Serial.println();

  previousRPMTime = millis();
  previousLCDTime = millis();
  previousBuzzerTime = millis();
}

// =====================================================
// READ MPU6050
// =====================================================

void readMPU6050()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
  {
    return;
  }

  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() >= 14)
  {
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    // Temperature register
    Tmp = Wire.read() << 8 | Wire.read();

    // Gyroscope
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    // Convert acceleration to g
    float ax = AcX / 16384.0;
    float ay = AcY / 16384.0;
    float az = AcZ / 16384.0;

    // Calculate total acceleration
    float accelerationMagnitude =
      sqrt(
        ax * ax +
        ay * ay +
        az * az
      );

    // Simple vibration indicator
    vibration = fabs(accelerationMagnitude - 1.0);
  }
}

// =====================================================
// READ TEMPERATURE
// =====================================================

void readTemperature()
{
  int tempADC = analogRead(LM35_PIN);

  float tempVoltage =
    tempADC * (5.0 / 1023.0);

  temperature =
    tempVoltage * 100.0;
}

// =====================================================
// READ CURRENT
// =====================================================

void readCurrent()
{
  int currentADC = analogRead(ACS712_PIN);

  float currentVoltage =
    currentADC * (5.0 / 1023.0);

  current =
    (currentVoltage - ACS_OFFSET)
    / ACS_SENSITIVITY;

  // Remove very small zero-current noise
  if (fabs(current) < 0.05)
  {
    current = 0.0;
  }
}

// =====================================================
// CALCULATE RPM
// =====================================================

void calculateRPM()
{
  unsigned long currentTime = millis();

  if (currentTime - previousRPMTime >= 1000)
  {
    noInterrupts();

    unsigned long pulses = pulseCount;
    pulseCount = 0;

    interrupts();

    // One pulse = one revolution
    rpm = pulses * 60.0;

    previousRPMTime = currentTime;

    // -------------------------
    // Serial output
    // -------------------------

    Serial.println();
    Serial.println("========== MOTOR DATA ==========");

    Serial.print("Temperature : ");
    Serial.print(temperature, 1);
    Serial.println(" C");

    Serial.print("Current     : ");
    Serial.print(current, 2);
    Serial.println(" A");

    Serial.print("RPM         : ");
    Serial.print(rpm, 0);
    Serial.println(" RPM");

    Serial.print("Vibration   : ");
    Serial.print(vibration, 3);
    Serial.println(" g");

    Serial.print("Hall Pulses : ");
    Serial.println(pulses);

    Serial.print("Status      : ");

    if (warning)
    {
      Serial.println("WARNING");
    }
    else
    {
      Serial.println("NORMAL");
    }

    Serial.println("===============================");
  }
}

// =====================================================
// CHECK THRESHOLDS
// =====================================================

void checkThresholds()
{
  warning = false;

  // Temperature
  if (temperature > TEMP_LIMIT)
  {
    warning = true;
  }

  // Current
  if (current > CURRENT_LIMIT)
  {
    warning = true;
  }

  // RPM
  if (rpm < RPM_LOW_LIMIT)
  {
    warning = true;
  }

  if (rpm > RPM_HIGH_LIMIT)
  {
    warning = true;
  }

  // Vibration
  if (vibration > VIBRATION_LIMIT)
  {
    warning = true;
  }
}

// =====================================================
// CONTROL BUZZER
// =====================================================

void controlBuzzer()
{
  unsigned long currentTime = millis();

  if (warning)
  {
    if (currentTime - previousBuzzerTime >= BUZZER_INTERVAL)
    {
      previousBuzzerTime = currentTime;

      buzzerState = !buzzerState;

      if (buzzerState)
      {
        tone(BUZZER_PIN, 1000);
      }
      else
      {
        noTone(BUZZER_PIN);
      }
    }
  }
  else
  {
    noTone(BUZZER_PIN);
    buzzerState = false;
  }
}

// =====================================================
// CONTROL RELAY
// =====================================================

void controlRelay()
{
  if (warning)
  {
    digitalWrite(RELAY_PIN, RELAY_ON);
  }
  else
  {
    digitalWrite(RELAY_PIN, RELAY_OFF);
  }
}

// =====================================================
// UPDATE LCD
// =====================================================

void updateLCD()
{
  unsigned long currentTime = millis();

  if (currentTime - previousLCDTime >= LCD_PAGE_TIME)
  {
    previousLCDTime = currentTime;

    lcdPage++;

    if (lcdPage > 2)
    {
      lcdPage = 0;
    }

    lcd.clear();

    // -------------------------
    // PAGE 0
    // -------------------------

    if (lcdPage == 0)
    {
      lcd.setCursor(0, 0);
      lcd.print("TEMP:");

      lcd.print(temperature, 1);
      lcd.print((char)223);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("CURR:");

      lcd.print(current, 2);
      lcd.print(" A");
    }

    // -------------------------
    // PAGE 1
    // -------------------------

    else if (lcdPage == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("RPM:");

      lcd.print(rpm, 0);

      lcd.setCursor(0, 1);
      lcd.print("VIB:");

      lcd.print(vibration, 3);
      lcd.print(" g");
    }

    // -------------------------
    // PAGE 2
    // -------------------------

    else if (lcdPage == 2)
    {
      lcd.setCursor(0, 0);

      if (warning)
      {
        lcd.print("STATUS: WARNING");
      }
      else
      {
        lcd.print("STATUS: NORMAL");
      }

      lcd.setCursor(0, 1);

      if (temperature > TEMP_LIMIT)
      {
        lcd.print("TEMP HIGH");
      }
      else if (current > CURRENT_LIMIT)
      {
        lcd.print("CURRENT HIGH");
      }
      else if (rpm < RPM_LOW_LIMIT)
      {
        lcd.print("RPM LOW");
      }
      else if (rpm > RPM_HIGH_LIMIT)
      {
        lcd.print("RPM HIGH");
      }
      else if (vibration > VIBRATION_LIMIT)
      {
        lcd.print("VIBRATION HIGH");
      }
      else
      {
        lcd.print("ALL OK");
      }
    }
  }
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  // Read sensors
  readTemperature();
  readCurrent();
  readMPU6050();

  // Calculate RPM
  calculateRPM();

  // Check thresholds
  checkThresholds();

  // Control outputs
  controlBuzzer();
  controlRelay();

  // Update LCD
  updateLCD();

  // Small delay for stable sensor reading
  delay(20);
}
