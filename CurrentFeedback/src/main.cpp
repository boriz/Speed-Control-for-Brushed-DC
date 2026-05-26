/*
  Arduino Mega brushed DC motor duty calibration experiment
    D11: PWM output to low-side N-channel MOSFET
    A0: Current from the shunt (1 Ohm) with 1.1V ADC reference, so max measurable current is ~1.1A
    D2: Scope trigger to show when ADC is sampling
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// -------------------- Hardware constants --------------------

constexpr float _SHUNT_OHMS = 1.0f;

// 10-bit ADC
constexpr float _ADC_COUNTS = 1023.0f;
constexpr float _ADC_REF_V = 1.1f;
constexpr float _ADC_VOLTS_PER_COUNT = _ADC_REF_V / _ADC_COUNTS;

// -------------------- PWM constants --------------------

// Arduino Mega clock = 16 MHz
// Fast PWM with TOP = ICR1
// Fpwm = 16 MHz / (1 * (1 + TOP))
// TOP = 1599 gives 10 kHz
constexpr uint16_t _PWM_TOP = 1599;

// ADC sample instant lags compare trigger by ~1.5 ADC clocks.
// ADC clock is 500 kHz (2 us), so use ~3 us = 48 timer ticks advance.
constexpr uint16_t _ADC_TRIGGER_ADVANCE_TICKS = 48;

// -------------------- Control constants --------------------

constexpr float _MIN_DUTY = 0.10f;  // This motor is not happy below ~10% duty
constexpr float _MAX_DUTY = 0.99f;  // Also limit the max to 99%
constexpr float _DUTY_STEP_SMALL = 0.01f; // 1% duty step for fine manual adjustment
constexpr float _DUTY_STEP_LARGE = 0.10f; // 10% duty step for coarse manual adjustment

constexpr uint32_t _PRINT_PERIOD_MS = 500; // Print status periodically
constexpr uint8_t _CONTROL_CYCLES = 10; // Number of PWM cycles to average before updating the control loop

float _dutySp = 0.25f;  // Reasonable default duty SP (scaled 0 to 1)

// Control mode. Open loop by default
enum class Mode : uint8_t
{
  OPEN_LOOP,
  CLOSED_LOOP
};
Mode _controlMode = Mode::OPEN_LOOP;


// -------------------- ADC globals --------------------
// These are updated in the ADC ISR and read in the main loop
volatile uint32_t _adcRawAverageSum = 0;  // Accumulates raw ADC samples for averaging
volatile uint8_t _adcRawAverageCount = 0;
volatile uint16_t _adcRawAverageForLoop = 0;
volatile bool _adcRawAverageReady = false;  // Let the main loop know when a new averaged ADC sample is ready
float _currentA = 0.0f; // Scaled current in Amps calculated from the latest ADC average

// Correction variables
float _point1Duty = 0.10f;
float _point1CurrentA = 0.100f;
float _dutyCorrectionPerAmp = 4.2f;

// Keep track of the last print time
uint32_t _lastPrintMs = 0;


// Updates the PWM duty register and aligns the ADC trigger to the on-time midpoint.
void setPwmDuty(float duty_1)
{
  // duty_1 scaled from 0.0 to 1.0
  float duty_constrained = constrain(duty_1, _MIN_DUTY, _MAX_DUTY);

  uint16_t duty_cnt = (uint16_t)(duty_constrained * _PWM_TOP);

  OCR1A = duty_cnt;

  // Trigger ADC so S&H lands at the midpoint of the ON pulse (duty_cnt / 2).
  // In non-inverting Fast PWM, OC1A is high from BOTTOM to OCR1A.
  uint16_t sampleTick = 1;

  if (duty_cnt / 2 > _ADC_TRIGGER_ADVANCE_TICKS)
  {
    sampleTick = duty_cnt / 2 - _ADC_TRIGGER_ADVANCE_TICKS;
  }

  OCR1B = sampleTick;
}


// Writes the latest operating values to the VFD over Serial1.
void updateVfdDisplay(Mode mode, float currentA, float dutySp, float dutyClosedLoopOut)
{
  Serial1.write(0x0C);  // Cursor home
  Serial1.print("\r\n");  // Clearing display is flicery, just shift old text out
  Serial1.print("\r\n");

  // Print mode and current
  Serial1.print("M:");
  switch (mode)
  {
    case Mode::OPEN_LOOP:
      Serial1.print("OPEN  ");
      break;

    case Mode::CLOSED_LOOP:
      Serial1.print("CLOSED");
      break;
  }
  Serial1.print(" | I:");
  Serial1.print(currentA, 3);
  Serial1.print("A");

  // Next line are duty cycles
  Serial1.print("\r\n");
  Serial1.print("SP:");
  Serial1.print(dutySp * 100.0f, 0);
  Serial1.print("%   | Out:");
  Serial1.print(dutyClosedLoopOut * 100.0f, 0);
  Serial1.print("%");
}


// Stores the no-load calibration point from open-loop duty and current sample.
void storeCalibrationPoint1()
{
  _point1Duty = _dutySp;
  _point1CurrentA = _currentA;

  Serial.print("Set Point 1: duty=");
  Serial.print(_point1Duty * 100.0f, 1);
  Serial.print("%  I=");
  Serial.print(_point1CurrentA, 3);
  Serial.println("A");
}


// Stores the load calibration point and calculates the duty correction per amp.
void storeCalibrationPoint2()
{
  float currentDeltaA = _currentA - _point1CurrentA;
  float dutyDelta = _dutySp - _point1Duty;

  // Calcualte correction factor in PWM duty per Amp 
  _dutyCorrectionPerAmp = dutyDelta / currentDeltaA;

  Serial.print("Set Point 2: duty=");
  Serial.print(_dutySp * 100.0f, 1);
  Serial.print("%  I=");
  Serial.print(_currentA, 3);
  Serial.print("A  corr=");
  Serial.print(_dutyCorrectionPerAmp, 4);
  Serial.println(" duty/A");
}


// Initializes the timer, ADC, serial port, and first PWM command.
void setup()
{
  Serial.begin(115200);

  // VFD display on Serial1 at 19200 baud
  Serial1.begin(19200);
  Serial1.write(0x16);  // Hide cursor
  Serial1.write(0x0E);  // Clear display
  Serial1.write(0x0C);  // Cursor home

  Serial.println("Setup starting...");

  pinMode(11, OUTPUT); // D11 = OC1A on Arduino Mega
  pinMode(2, OUTPUT);  // D2 = scope trigger

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  ICR1 = _PWM_TOP;

  // Fast PWM mode 14:
  // WGM13:0 = 1110, TOP = ICR1
  // Non-inverting output on OC1A
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // prescaler = 1
  TIMSK1 = (1 << OCIE1B);

  _dutySp = constrain(_dutySp, _MIN_DUTY, _MAX_DUTY);
  setPwmDuty(_dutySp);

  // ADC0 / A0
  // Internal 1.1V reference on ATmega2560:
  // REFS1:0 = 10
  ADMUX = (1 << REFS1) | 0;

  // ADC auto trigger source:
  // ADTS2:0 = 101 = Timer/Counter1 Compare Match B
  ADCSRB = (1 << ADTS2) | (1 << ADTS0);

  // Disable digital input buffer on ADC0
  DIDR0 = (1 << ADC0D);

  // Enable ADC, ADC interrupt, auto-trigger
  // ADC prescaler = 32 -> 16 MHz / 32 = 500 kHz ADC clock
  //
  // This is above the best-accuracy ADC clock range, but fast enough
  // for 10 kHz PWM synchronized sampling.
  ADCSRA = (1 << ADEN)
         | (1 << ADIE)
         | (1 << ADATE)
         | (1 << ADPS2)
         | (1 << ADPS0);

  // Clear pending Timer1 Compare B flag.
  // Important: the ADC trigger uses the timer compare flag edge.
  TIFR1 = (1 << OCF1B);

  // Start ADC auto-trigger system
  ADCSRA |= (1 << ADSC);

  sei();

  Serial.println("Setup complete.");
}


// Processes commands, applies manual or corrected duty control, and prints the latest state.
void loop()
{
  if (!_adcRawAverageReady)
  {
    // ADC is not ready with a new averaged sample yet, so skip the control loop
    return;
  }

  // Get the latest ADC average atomically with interrupts disabled, then re-enable interrupts for the rest of the loop.
  noInterrupts();
  float _adcRaw = _adcRawAverageForLoop;
  _adcRawAverageReady = false;
  interrupts();

  _currentA = (_adcRaw * _ADC_VOLTS_PER_COUNT) / _SHUNT_OHMS;

  if (Serial.available())
  {
    char cmd = Serial.read();
    switch (cmd)
    {
      case 'o':
        _controlMode = Mode::OPEN_LOOP;
        Serial.println("Set Mode=OPEN_LOOP");
        break;

      case 'c':
        _controlMode = Mode::CLOSED_LOOP;
        Serial.println("Set Mode=CLOSED");
        break;

      case 'w':
        _dutySp = constrain(_dutySp + _DUTY_STEP_SMALL, _MIN_DUTY, _MAX_DUTY);
        break;

      case 'W':
        _dutySp = constrain(_dutySp + _DUTY_STEP_LARGE, _MIN_DUTY, _MAX_DUTY);
        break;

      case 's':
        _dutySp = constrain(_dutySp - _DUTY_STEP_SMALL, _MIN_DUTY, _MAX_DUTY);
        break;

      case 'S':
        _dutySp = constrain(_dutySp - _DUTY_STEP_LARGE, _MIN_DUTY, _MAX_DUTY);
        break;

      case '1':
        storeCalibrationPoint1();
        break;

      case '2':
        storeCalibrationPoint2();
        break;
    }
  }

  float dutyClosedLoopOut = 0.0f;
  switch (_controlMode)
  {
    case Mode::OPEN_LOOP:
      _dutySp = constrain(_dutySp, _MIN_DUTY, _MAX_DUTY);
      setPwmDuty(_dutySp);
      break;

    case Mode::CLOSED_LOOP:
      // Limit the correction to the positive side only
      float correctionCurrentA = max(0.0f, _currentA - _point1CurrentA);
      dutyClosedLoopOut = _dutySp + _dutyCorrectionPerAmp * correctionCurrentA;
      dutyClosedLoopOut = constrain(dutyClosedLoopOut, _MIN_DUTY, _MAX_DUTY);
      setPwmDuty(dutyClosedLoopOut);
      break;
  }

  // Print the latest status periodically
  uint32_t now = millis();
  if (now - _lastPrintMs < _PRINT_PERIOD_MS)
  {
    return;
  }
  _lastPrintMs = now;

  // Update the VFD display first
  updateVfdDisplay(_controlMode, _currentA, _dutySp, dutyClosedLoopOut);
  
  // Now print some status to the console
  Serial.print("I=");
  Serial.print(_currentA, 3);
  Serial.print("A, Duty SP=");
  Serial.print(_dutySp * 100.0f, 1);
  Serial.print("%, Corr=");
  Serial.print(_dutyCorrectionPerAmp, 2);
  Serial.print("duty/A, p1_I=");
  Serial.print(_point1CurrentA, 3);
  Serial.print("A, mode=");
  switch (_controlMode)
  {
    case Mode::OPEN_LOOP:
      Serial.println("OPEN_LOOP");
      break;

    case Mode::CLOSED_LOOP:
      Serial.print("CLOSED_LOOP");

      // Print target in closed loop mode
      Serial.print(", Duty Out=");
      Serial.print(dutyClosedLoopOut * 100.0f, 1);
      Serial.println("%");
      break;
  }
}


// Captures synchronized ADC samples and publishes their average to the main loop.
ISR(ADC_vect)
{
  uint16_t adcSample = ADC;

  PORTE &= ~(1 << PE4); // D2 low: ADC conversion complete
  _adcRawAverageSum += adcSample;
  _adcRawAverageCount++;
  if (_adcRawAverageCount >= _CONTROL_CYCLES)
  {
    _adcRawAverageForLoop = (uint16_t)(_adcRawAverageSum / _CONTROL_CYCLES);
    _adcRawAverageSum = 0;
    _adcRawAverageCount = 0;
    _adcRawAverageReady = true;
  }

  // Clear Timer1 Compare B flag so the next PWM cycle can trigger ADC again.
  TIFR1 = (1 << OCF1B);
}


// Toggles the scope pin at the programmed ADC trigger instant.
ISR(TIMER1_COMPB_vect)
{
  PORTE |= (1 << PE4); // D2 high: ADC trigger instant
}
