// Sketch for Attiny84 handling two-phase charging extension in BMW i3

#include "USISerialSend.h"

#define PIN_CP_MEAS 0   // PA0 pin 13
#define PIN_CP_OUT  1   // PA2 pin 12
#define PIN_CP_IN   2   // PA2 pin 11
#define PIN_LED_OUT 3   // PA3 pin 10
#define PIN_CH_ON   7   // PA7 pin 6

#define AD_CP_MEAS  0   // ADC port to read CP voltage
#define AD_CP_IN    2   // ADC port to detect ingoing CP

// Our clock frequency is 15.36 MHz. 6A = 10% pwm so 1A = 10/6% of 10 us = 16 2/3 us.
// Our clock tick is 1/15.36 us, so clocks/amp = 16 2/3*15.36 = 256.

#define AMPCNT(current) (static_cast<unsigned int>(current) << 8)  // Multiply by 256
#define AMPCNT_HALF     (128)

#define MEASCNT         AMPCNT(3)     // Measurement at 5% of cycle after each change
#define EXTENDCNT       (AMPCNT(15))  // If 16A (actually >= 15.5A) indicated, double current.

#define WR_HL_WRAP(REGH, REGL, VAL) { REGH = ((VAL)>>8)&0xff; REGL = (VAL)&0xff; }
#define WR_HL(REG, VAL) WR_HL_WRAP(REG##H, REG##L, VAL)

#define RD_LH_WRAP(REGH, REGL, VAL) { register byte reg = REGL; VAL = (static_cast<word>(REGH) << 8) | reg; }
#define RD_LH(REG, VAL) RD_LH_WRAP(REG##H, REG##L, VAL)

#define STOP_TIMER    { TCCR1B = 0; }         // Counter stopped
#define START_TIMER   { STOP_TIMER; WR_HL(TCNT1, 0); TCCR1B = _BV(CS10); } // No prescaling i.e. 15.36MHz counting.

#define LEVEL_UNDEF     0
#define LEVEL_LOW12     1
#define LEVEL_HIGH3     2
#define LEVEL_HIGH6     3
#define LEVEL_HIGH9     4
#define LEVEL_HIGH12    5
#define N_LEVELS        6

static const byte levelMap[1023*3/100] /*PROGMEM*/ = {
  LEVEL_LOW12,  // -12 - -11
  LEVEL_UNDEF,  // -11 - -10
  LEVEL_UNDEF,  // -10 - -9
  LEVEL_UNDEF,  // -9  - -8
  LEVEL_UNDEF,  // -8  - -7
  LEVEL_UNDEF,  // -7  - -6
  LEVEL_UNDEF,  // -6  - -5
  LEVEL_UNDEF,  // -5  - -4
  LEVEL_UNDEF,  // -4  - -3
  LEVEL_UNDEF,  // -3  - -2
  LEVEL_UNDEF,  // -2  - -1
  LEVEL_UNDEF,  // -1  -  0
  LEVEL_UNDEF,  //  0  -  1
  LEVEL_UNDEF,  //  1  -  2
  LEVEL_HIGH3,  //  2  -  3
  LEVEL_HIGH3,  //  3  -  4
  LEVEL_UNDEF,  //  4  -  5
  LEVEL_HIGH6,  //  5  -  6
  LEVEL_HIGH6,  //  6  -  7
  LEVEL_UNDEF,  //  7  -  8
  LEVEL_HIGH9,  //  8  -  9
  LEVEL_HIGH9,  //  9  - 10
  LEVEL_UNDEF,  // 10  - 11
  LEVEL_HIGH12, // 11  - 12
  LEVEL_HIGH12, // 12  - 13
  LEVEL_UNDEF,  // 13  - 14
  LEVEL_UNDEF,  // 14  - 15
  LEVEL_UNDEF,  // 15  - 16
  LEVEL_UNDEF,  // 16  - 17
  LEVEL_UNDEF,  // 17  - 18
};

static bool cpIn = false; // Indicate last CP input (true = high)

static word upCnt = 0;  // Number of rising CP edges
static word dnCnt = 0;  // Number of falling CP edges

static bool extending = false;  // Currently extending PWM
static byte evseCurrent = 0;  // Number of ampere indicated by EVSE

static bool doing32A = false;  // True when extending to 32A

void setup() {
  // put your setup code here, to run once:

  // Init status led:
  pinMode(PIN_LED_OUT, OUTPUT);  // Status LED
  digitalWrite(PIN_LED_OUT, LOW);

  // Init other pins:
  pinMode(PIN_CP_OUT, OUTPUT);     // PWM output initially high
  digitalWrite(PIN_CP_OUT, HIGH);

  pinMode(PIN_CH_ON, OUTPUT);       // Charge indication to car initially off
  digitalWrite(PIN_CH_ON, LOW);
  
  word pilot = 0;
  
  analogReference(DEFAULT);       // 5V analog reference
  
  // Set up AD manually for car CP measurements as it must be interrupt controlled:
  ADCSRA = _BV(ADEN)|_BV(ADIE)|_BV(ADPS2);  // Enable ADC, ADC interrupts, clock divisor 16
  ADCSRB = 0;
  ADMUX = AD_CP_MEAS;
  DIDR0 |= _BV(AD_CP_MEAS);  // Disable digital input on A0

  // Set up timer 1 for duty-cycle measurement and AD start interrupt
  TCCR1A  = 0;
  TCCR1B  = 0; // Prescaler disabled so timer is stopped
  TCCR1C  = 0;
  TIMSK1  = _BV(OCIE1A)|_BV(OCIE1B)|_BV(TOIE1);  // Interrupt on match OCR1A: 5% passed; measure CP voltage; interrupt on match OCR1B: Max PWM reached and on full run (line stuck)
  TIFR1   = 0;
  WR_HL(OCR1A, MEASCNT);      // First interrupt at measurement PWM
  WR_HL(OCR1B, EXTENDCNT);    // Second interrupt at PWM wide enough for us to expand to 32A
  WR_HL(ICR1, 0);
  WR_HL(TCNT1, 0);  // Initialize timer value

  // Initialize comparator
  ADCSRB  &=  ~_BV(ACME);           // Clear ACME flag to prepare for bandgap as reference
  ACSR    =   _BV(ACBG)|_BV(ACIE);  // (Enable), use bandgap, enable interrupts, (interrupt on toggle)
  DIDR0   |= _BV(AD_CP_IN);  // Disable digital input on A0

  // We're running; most stuff will run in interrupt routines.
  serialSetup();
  serialWrite("BMW i3 two phase charging running\n\r");
}

ISR(ANA_COMP_vect)  // Control pilot toggled.
{
  cpIn = (ACSR & _BV(ACO)) == 0;    // Current CP state from EVSE

  if (cpIn) {
    digitalWrite(PIN_CP_OUT, HIGH); // Output CP high
    upCnt++;  // We got a leading CP edge.
    WR_HL(OCR1B, EXTENDCNT);   // Interrupt at time ready for extending to 32A
    extending = false;  // not yet extending
    START_TIMER;  // Start the timer so we will make the relevant measurements.*/
  } else {
    if (!extending) {
      digitalWrite(PIN_CP_OUT, LOW); // Output CP low as we're not extending to 32A at the moment.
    }
    dnCnt++;
    // Read timer value to get high PWM %
    register word cnt;
    RD_LH(TCNT1, cnt);
    evseCurrent = (cnt+AMPCNT_HALF) >> 8;
    doing32A = cnt >= EXTENDCNT;
    if (!extending) {
      START_TIMER;  // Start timer for negative cycle measurement
    } // Else: Wait for extending done
  }
}

ISR(TIMER1_COMPA_vect)  // Timer compare interrupt - start AD
{
  ADCSRA |= _BV(ADSC); // Start conversion
  if (!cpIn) {
    STOP_TIMER;  // No need for timer interrupts during negative cycle
  }
}

ISR(TIMER1_COMPB_vect)  // Timer reached 15.5A or 32A marks
{
  if (!extending) {
    if (cpIn) {
      // CP signal is still active - extend to 32A
      WR_HL(OCR1B, AMPCNT(32));  // Call this routine again at 32A mark
      extending = true;
    } else {
      // CP signal not active any more - stop timer. Actually we should never get here as timer is reset at AD point in negative cycle.
      STOP_TIMER;
    }
  } else {
    // We are extending and reached 32A mark - stop extending and stop timer
    digitalWrite(PIN_CP_OUT, LOW);  // Stop overriding PWM
    extending = false;
    START_TIMER;  // Start timer for negative cycle measurement
  }
}

// CP measurement:
static byte portLevel[2] = { LEVEL_HIGH12, LEVEL_HIGH12 };
static volatile bool chargeOn = false;

static uint16_t measureLevel[2] = {0,0};

ISR(ADC_vect)
{
  uint16_t level;
  RD_LH(ADC, level);
  sei();
  measureLevel[cpIn] = level;
  portLevel[cpIn] = levelMap[level * 3 / 100];
  
  chargeOn = portLevel[false] == LEVEL_LOW12 && (portLevel[true] == LEVEL_HIGH6 || portLevel[true] == LEVEL_HIGH3);
  digitalWrite(PIN_CH_ON, chargeOn ? HIGH : LOW);
}

// No need for timer overflow for now: ISR(TIMER1_OVF_vect)  {}

static bool prevChargeOn = false;
static bool prev32A = false;
static bool prevDC = false;
static byte prevCurrent = 0;

char s[16];

static const char* on = "ON\r\n";
static const char* off = "OFF\r\n";

void loop()
{
  bool dc = (evseCurrent >= 2 && evseCurrent <= 4);
  register byte act = chargeOn ? LOW : HIGH;
  register byte inAct = chargeOn ? HIGH : LOW;
  digitalWrite(PIN_LED_OUT, inAct);
  serialDelay(1200);
  digitalWrite(PIN_LED_OUT, (doing32A) ? act : inAct);
  serialDelay(200);
  digitalWrite(PIN_LED_OUT, inAct);
  serialDelay(200);
  digitalWrite(PIN_LED_OUT, act);
  serialDelay(200);

  if (chargeOn != prevChargeOn) {
    serialWrite("Charging: ");
    serialWrite(chargeOn ? on : off);
    prevChargeOn = chargeOn;
  }

  if (doing32A != prev32A) {
    serialWrite("32A extending: ");
    serialWrite(doing32A ? on : off);
    prev32A = doing32A;
  }

  if (prevDC != dc) {
    serialWrite("DC charger: ");
    serialWrite(dc ? on : off);
    prevDC = dc;
  }

  if (prevCurrent != evseCurrent) {
    serialWrite("EVSE current: ");
    sprintf(s, "%d", evseCurrent);
    serialWrite(s);
    serialWrite("\r\n");
    prevCurrent = evseCurrent;
  }
  
  /*serialWrite("Upcount: ");
  sprintf(s, "%d\r\n", upCnt);
  serialWrite(s);
  serialWrite("Downcount: ");
  sprintf(s, "%d\r\n", dnCnt);
  serialWrite(s);

  serialWrite("Level[0]: ");
  sprintf(s, "%d\r\n", portLevel[false]);
  serialWrite(s);
  serialWrite("Level[1]: ");
  sprintf(s, "%d\r\n", portLevel[true]);
  serialWrite(s);*/
}

