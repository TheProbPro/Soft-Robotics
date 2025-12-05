/*
  Soft Robotic Flower — Autonomous Emotion Behaviors (+ Serial Debug)
  Hardware:
    - Arduino UNO
    - 2x Pumps (PWM capable pins)
    - 2x Solenoid Valves (digital on/off)
    - WS2812B LED ring (e.g., 12 LEDs)

  States: NEUTRAL, CALM, CURIOUS, ANGRY, SAD
  Debug: set DEBUG to 1 to enable serial logs (115200 baud).
*/

#include <Adafruit_NeoPixel.h>

// -------------------- DEBUG TOGGLE --------------------
#define DEBUG 0
#if DEBUG
  #define DBG_BEGIN()   Serial.begin(115200)
  #define DBG_PRINTLN(x) Serial.println(x)
  #define DBG_PRINT(x)   Serial.print(x)
#else
  #define DBG_BEGIN()
  #define DBG_PRINTLN(x)
  #define DBG_PRINT(x)
#endif

// -------------------- USER CONFIG --------------------
// Pins (change to match your wiring)
const uint8_t PIN_PUMP_BASE   = 6;   // PWM
const uint8_t PIN_VALVE_BASE  = 5;   // Digital
const uint8_t PIN_PUMP_LEAVES = 11;   // PWM
const uint8_t PIN_VALVE_LEAVES= 3;   // Digital
const uint8_t PIN_LEDS        = 7;   // WS2812B data

// LED ring config
const uint8_t NUM_LEDS = 16;         // adjust if your ring differs
Adafruit_NeoPixel leds(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

// Motion/LED global scaling
float SPEED_FACTOR = 1.0f;           // >1.0 faster, <1.0 slower
uint8_t MAX_PWM_BASE   = 220;        // cap to protect pumps
uint8_t MAX_PWM_LEAVES = 220;

// Small threshold to decide inflate vs deflate based on level change
const float LEVEL_EPS = 0.01f;

// -------------------- STATE MACHINE --------------------
enum State { NEUTRAL=0, CALM=1, CURIOUS=2, ANGRY=3, SAD=4 };
const char* STATE_NAMES[] = {"NEUTRAL","CALM","CURIOUS","ANGRY","SAD"}; // (indexing only used for prints)

// Transition probability matrix P[current][next] (sums to ~1.0 per row)
const float P[5][5] = {
  //               N      C      U      A      S
  /* NEUTRAL */ {0.10f, 0.35f, 0.25f, 0.15f, 0.15f},
  /* CALM    */ {0.25f, 0.10f, 0.30f, 0.20f, 0.15f},
  /* CURIOUS */ {0.10f, 0.25f, 0.10f, 0.25f, 0.30f},
  /* ANGRY   */ {0.25f, 0.20f, 0.10f, 0.10f, 0.35f},
  /* SAD     */ {0.30f, 0.30f, 0.10f, 0.20f, 0.10f}
};

// Dwell time ranges (seconds)
struct Range { float lo, hi; };
// <<< Place RGB here to avoid Arduino preprocessor quirks >>>
struct RGB { uint8_t r, g, b; };

const Range DWELL[5] = {
  /* NEUTRAL */ {5.0f, 7.0f},
  /* CALM    */ {7.0f, 15.0f},
  /* CURIOUS */ {5.0f, 10.0f},
  /* ANGRY   */ {10.0f, 13.0f},
  /* SAD     */ {5.0f, 12.0f}
};

// -------------------- RUNTIME --------------------
State current = NEUTRAL;
unsigned long stateEndMs = 0;
unsigned long lastUpdateMs = 0;
unsigned long lastDebugMs  = 0;

// Phase memory for direction detection
float prevDesiredPMiddle = 0.0f;

// LED helpers
uint16_t ledChaseIndex = 0;

// For debug readback
uint8_t pwmBase = 0, pwmLeaves = 0;
bool valveBaseOpen = false, valveLeavesOpen = false;

// -------------------- UTILITIES --------------------
float frand(float a, float b) { return a + (b - a) * (random(0, 10000) / 10000.0f); }

unsigned long sampleDwellMillis(State s) {
  float secs = frand(DWELL[s].lo, DWELL[s].hi) / SPEED_FACTOR;
  return (unsigned long)(secs * 1000.0f);
}

// Roulette wheel on one row of P
State sampleNextState(State s) {
  float r = frand(0.0f, 1.0f);
  float acc = 0.0f;
  for (int n=0; n<5; ++n) {
    acc += P[s][n];
    if (r <= acc) return (State)n;
  }
  return (State)4; // fallback
}

// Clamp helper
float clamp01(float x) { if (x < 0) return 0; if (x > 1) return 1; return x; }

// Map 0..1 → PWM
uint8_t levelToPWM(float level, uint8_t maxPWM) {
  level = clamp01(level);
  return (uint8_t)(level * maxPWM);
}

// Simple HSV → RGB (0..1) for smooth fades
RGB hsv(float h, float s, float v) {
  float i = floor(h * 6.0f);
  float f = h * 6.0f - i;
  float p = v * (1.0f - s);
  float q = v * (1.0f - f * s);
  float t = v * (1.0f - (1.0f - f) * s);
  float r,g,b;
  switch (((int)i) % 6) {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }
  RGB out = {(uint8_t)(r*255), (uint8_t)(g*255), (uint8_t)(b*255)};
  return out;
}

// Set all LEDs to a color scaled by brightness
void ledsFill(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, const char* modeTag=nullptr) {
  leds.setBrightness(brightness);
  for (uint8_t i=0; i<NUM_LEDS; ++i) leds.setPixelColor(i, leds.Color(r,g,b));
  leds.show();
  #if DEBUG
    if (modeTag) { DBG_PRINT("LED="); DBG_PRINT(modeTag); DBG_PRINT(",B="); DBG_PRINTLN(brightness); }
  #endif
}

// Drive one pneumatic channel given target "level" (0..1)
// If rising: close valve, pump proportional; if falling: open valve, pump off; hold: weak maintain
//void applyChannel(float targetLevel, float &lastLevel, uint8_t pinPump, uint8_t pinValve, uint8_t maxPWM,
//                  uint8_t &pwmOut, bool &valveOpen) {
//  float diff = targetLevel - lastLevel;
//  if (diff >= 0.01f) {
//   analogWrite(pinValve, 0); // closed
//    valveOpen = false;
//    pwmOut = levelToPWM(targetLevel, maxPWM);
//    analogWrite(pinPump, pwmOut);
//  } else if (diff <= -0.01f) {
//    analogWrite(pinPump, 0);
//    pwmOut = 0;
//    analogWrite(pinValve, 255); // open to deflate
//    valveOpen = true;
//  } else {
//    analogWrite(pinValve, 0);
//    valveOpen = false;
//    pwmOut = levelToPWM(targetLevel, maxPWM) / 3;
//    analogWrite(pinPump, pwmOut);
//  }
//  lastLevel = targetLevel;
//}

void setPressureMiddle(float desiredP, float inMin, float inMax){
  //Serial.println(desiredP);
  float minP1 = 40;
  float maxP1 = 110;

  float minP = minP1;
  float maxP = maxP1;
  //desiredP = map(desiredP, inMin, inMax, minP, maxP);
  desiredP = (maxP - minP) * desiredP + minP;
  float currentP = analogRead(A1);
  float error = currentP - desiredP;
  
  //Serial.println("pumping " + (String)desiredP + ", " + currentP + ", " + error);
  if(error < -10){
    int val = map(error*-1, 0, maxP, 50, 255);
    analogWrite(PIN_PUMP_BASE, val);
    analogWrite(PIN_VALVE_BASE, 0);
    
  }
  else if(error > 10){
    int val = map(error, 0, maxP, 230, 255);
    analogWrite(PIN_VALVE_BASE, val);
    analogWrite(PIN_PUMP_BASE, 0);
//    Serial.println((String) prevDesiredPMiddle + ", " + currentP);
//    if((prevDesiredPMiddle - currentP) < -25.0f){
//      Serial.println("allaaaarm");
//    }
    
    //Serial.println("valving " + (String)error + ", " + val);
  }
  else{
    analogWrite(PIN_PUMP_BASE, 0);
    analogWrite(PIN_VALVE_BASE, 0);
  }
  prevDesiredPMiddle = currentP;
}

void setPressureLeaves(float desiredP, float inMin, float inMax){
  float minP1 = 40;
  float maxP1 = 500;
  float minP2 = 40;
  float maxP2 = 600;//470

  float minP = minP2;
  float maxP = maxP2;
  //desiredP = map(desiredP, inMin, inMax, minP, maxP);
  desiredP = (maxP - minP) * desiredP + minP;
  float currentP = analogRead(A2);
  float error = currentP - desiredP;
  if(error < -10){
    int val = map(error*-1, 0, maxP, 100, 255);
    analogWrite(PIN_PUMP_LEAVES, val);
    analogWrite(PIN_VALVE_LEAVES, 0);
    //Serial.println("pumping " + (String)error + ", " + val);
  }
  else if(error > 10){
    int val = map(error, 0, maxP, 230, 255);
    analogWrite(PIN_VALVE_LEAVES, val);
    analogWrite(PIN_PUMP_LEAVES, 0);
    //Serial.println("valving " + (String)error + ", " + val);
  }
  else{
    analogWrite(PIN_PUMP_LEAVES, 0);
    analogWrite(PIN_VALVE_LEAVES, 0);
  }
}

// -------------------- WAVEFORMS --------------------
float triWave(unsigned long t, unsigned long period, float minL, float maxL) {
  if (period < 10) period = 10;
  unsigned long x = t % period;
  float phase = (float)x / (float)period;
  float y = (phase < 0.5f) ? (phase*2.0f) : (2.0f - phase*2.0f); // 0→1→0
  return minL + (maxL - minL) * y;
}

float sineWave(unsigned long t, unsigned long period, float minL, float maxL) {
  if (period < 10) period = 10;
  float phase = (float)(t % period) / (float)period; // 0..1
  float y = 0.5f + 0.5f * sinf(2.0f * 3.14159f * phase);
  return minL + (maxL - minL) * y;
}

float squareWave(unsigned long t, unsigned long period, float duty) {
  if (period < 10) period = 10;
  float phase = (float)(t % period) / (float)period;
  return (phase < duty) ? 1.0f : 0.0f;
}

float heartPulse(unsigned long t, unsigned long period) {
  float phase = (float)(t % period) / (float)period; // 0..1
  float s = sinf(2.0f*3.14159f*phase);
  float y = powf(fabs(s), 3.5f);
  return y;
}

// -------------------- BEHAVIORS --------------------
void runNeutral(unsigned long now) {
  unsigned long period = (unsigned long)(8000.0f / SPEED_FACTOR);
  float baseL   = sineWave(now, period, 0.1f, 0.85f);
  setPressureMiddle(baseL, 0.0f, 1.0f);
  setPressureLeaves(0.0f, 0.0f, 1.0f);
  ledsFill(100,100,100, 100, "NEUTRAL");
}

void runCalm(unsigned long now) {
  unsigned long period = (unsigned long)(8000.0f / SPEED_FACTOR);
  float baseL   = sineWave(now, period, 0.1f, 0.85f);
  float leavesL = sineWave(now + (unsigned long)(500.0f / SPEED_FACTOR), period, 0.1f, 0.55f);//0.80f);0.55
  setPressureMiddle(baseL, 0.0f, 1.0f);
  setPressureLeaves(leavesL, 0.0f, 1.0f);

  RGB c = hsv(0.48f, 0.6f, 0.9f); // teal
  uint8_t bright = (uint8_t)(20 + baseL*120);
  ledsFill(c.r, c.g, c.b, bright, "CALM");
}

void runCurious(unsigned long now) {
  unsigned long pulsePeriod = (unsigned long)(1200.0f / SPEED_FACTOR);
  float leavesPulse = triWave(now, pulsePeriod, 0.15f, 0.85f);

  setPressureMiddle(0.5f, 0.0f, 1.0f);
  setPressureLeaves(leavesPulse, 0.0f, 1.0f);
  
  // rotating warm yellow/white
  leds.setBrightness(60);
  for (uint8_t i=0; i<NUM_LEDS; ++i) {
    uint8_t idx = (i + (ledChaseIndex/2)) % NUM_LEDS;
    float atten = (idx==i) ? 1.0f : 0.25f;
    uint8_t r = (uint8_t)(220 * atten);
    uint8_t g = (uint8_t)(180 * atten);
    uint8_t b = (uint8_t)( 40 * atten);
    leds.setPixelColor(idx, leds.Color(r,g,b));
  }
  leds.show();
  ledChaseIndex = (ledChaseIndex + 1) % (NUM_LEDS*4);
  #if DEBUG
    DBG_PRINT("LED=CURIOUS, idx="); DBG_PRINTLN(ledChaseIndex);
  #endif
}

void runAngry(unsigned long now) {
  unsigned long burstPeriod = (unsigned long)(900.0f / SPEED_FACTOR);
  float baseL = squareWave(now, burstPeriod, 0.45f);
  float leavesL = 0.5f + baseL * (0.8f - 0.5f);

  setPressureMiddle(baseL, 0.0f, 1.0f);
  setPressureLeaves(leavesL, 0.0f, 1.0f);

  float hb = heartPulse(now, (unsigned long)(500.0f / SPEED_FACTOR));
  uint8_t brightness = (uint8_t)(80 + hb * 120);
  leds.setBrightness(brightness);
  for (uint8_t i=0; i<NUM_LEDS; ++i) {
    uint8_t on = (i == (ledChaseIndex % NUM_LEDS)) ? 40 : 0;
    leds.setPixelColor(i, leds.Color(255, 60 + on, 0));
  }
  leds.show();
  if ((now % 60) == 0) ledChaseIndex++;
  #if DEBUG
    DBG_PRINT("LED=ANGRY, hb="); DBG_PRINT(hb); DBG_PRINT(", idx="); DBG_PRINTLN(ledChaseIndex);
  #endif
}

void runSad(unsigned long now) {
  unsigned long period = (unsigned long)(12000.0f / SPEED_FACTOR);
  float baseL   = sineWave(now, period, 0.05f, 0.35f);
  float leavesL = sineWave(now + (unsigned long)(800.0f / SPEED_FACTOR), period, 0.03f, 0.25f);

  setPressureMiddle(baseL, 0.0f, 1.0f);
  setPressureLeaves(leavesL, 0.0f, 1.0f);

  float v = 0.15f + baseL * 0.4f;
  RGB c = hsv(0.70f, 0.6f, v);
  uint8_t bright = (uint8_t)(30 + v*80);
  ledsFill(c.r, c.g, c.b, bright, "SAD");
}

// -------------------- CORE LOOP --------------------
void runBehavior(State s, unsigned long now) {
  switch (s) {
    case NEUTRAL: runNeutral(now); break;
    case CALM:    runCalm(now);    break;
    case CURIOUS: runCurious(now); break;
    case ANGRY:   runAngry(now);   break;
    case SAD:     runSad(now);     break;
  }
}

void printStateHeader(State s, unsigned long dwellMs) {
  #if DEBUG
    DBG_PRINT("\n==> ENTER "); DBG_PRINT(STATE_NAMES[s]);
    DBG_PRINT(" | dwell(ms)="); DBG_PRINTLN(dwellMs);
  #endif
}

void debugTick(State s, unsigned long now) {
  #if DEBUG
    // Throttle logs to every ~250ms
    if ((long)(now - lastDebugMs) >= 250) {
      lastDebugMs = now;
      DBG_PRINT("t="); DBG_PRINT(now);
      DBG_PRINT(" | S="); DBG_PRINT(STATE_NAMES[s]);
      DBG_PRINT(" | basePWM="); DBG_PRINT(pwmBase);
      DBG_PRINT(" vBase="); DBG_PRINT(valveBaseOpen ? "OPEN" : "CLOSED");
      DBG_PRINT(" | leavesPWM="); DBG_PRINT(pwmLeaves);
      DBG_PRINT(" vLeaves="); DBG_PRINT(valveLeavesOpen ? "OPEN" : "CLOSED");
      DBG_PRINT(" | lvlB="); DBG_PRINT(lastLevelBase);
      DBG_PRINT(" lvlL="); DBG_PRINTLN(lastLevelLeaves);
    }
  #endif
}

void enterState(State s) {
  unsigned long dwell = sampleDwellMillis(s);
  stateEndMs = millis() + dwell;
  //printStateHeader(s, dwell);
  Serial.println((String)STATE_NAMES[s] + ", " + (String)dwell);
}

// -------------------- ARDUINO --------------------
void setup() {
  pinMode(PIN_PUMP_BASE,   OUTPUT);
  pinMode(PIN_VALVE_BASE,  OUTPUT);
  pinMode(PIN_PUMP_LEAVES, OUTPUT);
  pinMode(PIN_VALVE_LEAVES,OUTPUT);

  analogWrite(PIN_PUMP_BASE,   0);
  analogWrite(PIN_PUMP_LEAVES, 0);
  analogWrite(PIN_VALVE_BASE,  0);
  analogWrite(PIN_VALVE_LEAVES,0);

  leds.begin();
  leds.show();
  leds.setBrightness(50);

  DBG_BEGIN();
  randomSeed(analogRead(A0));

  current = NEUTRAL;
  enterState(current);
  lastUpdateMs = millis();

  #if DEBUG
    DBG_PRINTLN("Soft Flower: debug enabled @115200");
  #endif
  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis();
//  runBehavior(current, now);
//  debugTick(current, now);
//
//  if ((long)(now - stateEndMs) >= 0) {
//    State next = sampleNextState(current);
//    current = next;
//    enterState(current);
//  }
  // Behavior A: Sad
  //runSad(now);

  // Behavior B: Calm
  //runCalm(now);

  //Behavior C: Angry
  //runAngry(now);

  // Behavior D: Neutral
  runNeutral(now);

  // Behavior E: Curious
  //runCurious(now);
  delay(10);
}
