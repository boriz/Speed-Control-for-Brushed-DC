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

constexpr float _MIN_DUTY = 0.10f;  // That motor looks unhappy below 10% duty
constexpr float _MAX_DUTY = 0.99f;
constexpr float _DUTY_STEP_SMALL = 0.01f; // 1% duty step for fine manual adjustment
constexpr float _DUTY_STEP_LARGE = 0.10f; // 10% duty step for coarse manual adjustment

constexpr uint32_t _PRINT_PERIOD_MS = 1000; // Print status every 1 second
constexpr uint8_t _CONTROL_CYCLES = 10; // Number of PWM cycles to average before updating the control loop

float _dutyCmd1 = 0.25f;  // Reasonable default duty

// Control mode. Open loop by default
enum class Mode : uint8_t
{
  OPEN_LOOP,
  CLOSED_LOOP
};
Mode _controlMode = Mode::OPEN_LOOP;

// -------------------- ADC globals --------------------

volatile uint32_t _adcRawAverageSum = 0;  // Accumulates raw ADC samples for averaging
volatile uint8_t _adcRawAverageCount = 0;
volatile uint16_t _adcRawAverageForLoop = 0;
volatile bool _adcRawAverageReady = false;  // Let the main loop know when a new averaged ADC sample is ready

uint32_t _lastPrintMs = 0;
float _currentSampleA = 0.0f;
float _point1Duty = 0.0f;
float _point1CurrentA = 0.100f;
float _point2Duty = 0.0f;
float _point2CurrentA = 0.0f;
float _dutyCorrectionPerAmp = 4.2f;
bool _point1Stored = false;
bool _correctionReady = false;

// -------------------- PWM setup --------------------

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


// Calculates the closed-loop duty target from the manual setpoint and upward-only current correction.
float calculateClosedLoopDutyTarget()
{
  float dutyTarget_1 = _dutyCmd1;
  float correctionCurrentA = max(0.0f, _currentSampleA - _point1CurrentA);

  dutyTarget_1 += _dutyCorrectionPerAmp * correctionCurrentA;

  return dutyTarget_1;
}


// Stores the no-load calibration point from open-loop duty and current sample.
void storeCalibrationPoint1()
{
  _point1Duty = _dutyCmd1;
  _point1CurrentA = _currentSampleA;
  _point1Stored = true;
  _correctionReady = false;

  Serial.print("point1: duty=");
  Serial.print(_point1Duty * 100.0f, 1);
  Serial.print("%  I=");
  Serial.print(_point1CurrentA, 3);
  Serial.println("A");
}


// Stores the load calibration point and calculates the duty correction per amp.
void storeCalibrationPoint2()
{
  float currentDeltaA = 0.0f;
  float dutyDelta = 0.0f;


  _point2Duty = _dutyCmd1;
  _point2CurrentA = _currentSampleA;
  currentDeltaA = _point2CurrentA - _point1CurrentA;
  dutyDelta = _point2Duty - _point1Duty;

  _dutyCorrectionPerAmp = dutyDelta / currentDeltaA;
  _correctionReady = true;

  Serial.print("point2: duty=");
  Serial.print(_point2Duty * 100.0f, 1);
  Serial.print("%  I=");
  Serial.print(_point2CurrentA, 3);
  Serial.print("A  corr=");
  Serial.print(_dutyCorrectionPerAmp, 4);
  Serial.println(" duty/A");
}


// Applies one serial command according to the active control mode.
void processCommand(char cmd_1)
{
  switch (cmd_1)
  {
    case 'o':
      _controlMode = Mode::OPEN_LOOP;
      Serial.println("mode=OPEN");
      break;

    case 'c':
      _controlMode = Mode::CLOSED_LOOP;
      Serial.println("mode=CLOSED");
      break;

    case 'w':
      _dutyCmd1 = constrain(_dutyCmd1 + _DUTY_STEP_SMALL, _MIN_DUTY, _MAX_DUTY);
      break;

    case 'W':
      _dutyCmd1 = constrain(_dutyCmd1 + _DUTY_STEP_LARGE, _MIN_DUTY, _MAX_DUTY);
      break;

    case 's':
      _dutyCmd1 = constrain(_dutyCmd1 - _DUTY_STEP_SMALL, _MIN_DUTY, _MAX_DUTY);
      break;

    case 'S':
      _dutyCmd1 = constrain(_dutyCmd1 - _DUTY_STEP_LARGE, _MIN_DUTY, _MAX_DUTY);
      break;

    case '1':
      switch (_controlMode)
      {
        case Mode::OPEN_LOOP:
          storeCalibrationPoint1();
          break;

        case Mode::CLOSED_LOOP:
          Serial.println("point1: use OPEN mode");
          break;
      }
      break;

    case '2':
      switch (_controlMode)
      {
        case Mode::OPEN_LOOP:
          storeCalibrationPoint2();
          break;

        case Mode::CLOSED_LOOP:
          Serial.println("point2: use OPEN mode");
          break;
      }
      break;
  }
}


// Initializes the timer, ADC, serial port, and first PWM command.
void setup()
{
  Serial.begin(115200);

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

  _dutyCmd1 = constrain(_dutyCmd1, _MIN_DUTY, _MAX_DUTY);
  setPwmDuty(_dutyCmd1);

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

  _currentSampleA = (_adcRaw * _ADC_VOLTS_PER_COUNT) / _SHUNT_OHMS;

  uint32_t now = millis();

  if (Serial.available())
  {
    char cmd = Serial.read();
    processCommand(cmd);
  }

  switch (_controlMode)
  {
    case Mode::OPEN_LOOP:
      _dutyCmd1 = constrain(_dutyCmd1, _MIN_DUTY, _MAX_DUTY);
      setPwmDuty(_dutyCmd1);
      break;

    case Mode::CLOSED_LOOP:
      setPwmDuty(calculateClosedLoopDutyTarget());
      break;
  }

  if (now - _lastPrintMs < _PRINT_PERIOD_MS)
  {
    return;
  }

  _lastPrintMs = now;
  Serial.print("I=");
  Serial.print(_currentSampleA, 3);
  Serial.print("A, duty_sp=");
  Serial.print(_dutyCmd1 * 100.0f, 1);
  Serial.print(", corr=");
  Serial.print(_dutyCorrectionPerAmp, 4);
  Serial.print(" duty/A, p1_I=");
  Serial.print(_point1CurrentA, 3);
  Serial.print("A, p1_duty=");
  Serial.print(_point1Duty * 100.0f, 1);
  Serial.print(" , mode=");
  switch (_controlMode)
  {
    case Mode::OPEN_LOOP:
      Serial.println("OPEN LOOP");
      break;

    case Mode::CLOSED_LOOP:
      Serial.println("CLOSED LOOP");
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
