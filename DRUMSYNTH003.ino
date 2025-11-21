#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <math.h>

// ---------- Pitch Envelope Struct ----------
struct PitchEnv {
  float base;    // 0..1
  float attack;  // 0..1
  float decay;   // 0..1
  float amount;  // 0..1
  float extend;  // 0..1
};

// ---------- Audio objects ----------

// Kick: body oscillators
AudioSynthWaveform       oscBody;        // Sine A (osc 0)
AudioSynthWaveform       oscBody2;       // Sine B (osc 1)
AudioSynthWaveform       oscTri1;        // Tri A  (osc 2)
AudioSynthWaveform       oscTri2;        // Tri B  (osc 3)
AudioEffectEnvelope      envAmp;         // AMP A (kick amp envelope)

// Kick: noise
AudioSynthNoiseWhite     noise;          // shared noise source
AudioFilterStateVariable noiseFilter;    // kick noise filter
AudioEffectEnvelope      envNoise;       // kick noise envelope

// Kick: transient (tick/clap at start)
AudioSynthNoiseWhite     noiseTransient; // separate noise for transient
AudioFilterStateVariable transientFilter; // high-pass for clicky tick sound
AudioEffectEnvelope      envTransient;   // very short envelope for transient

// SNARE: tone/body
AudioSynthWaveform       oscSnareTone;
AudioEffectEnvelope      envSnareTone;

// SNARE: noise
AudioFilterStateVariable snareNoiseFilter;
AudioEffectEnvelope      envSnareNoise;

AudioMixer4              mixBody;        // kick body mix (4 oscillators)
AudioFilterStateVariable oscFilter;      // NEW: filter for summed oscillators
AudioEffectWaveshaper    oscDrive;       // NEW: distortion for summed oscillators
AudioMixer4              mixKick;        // kick: body + transient
AudioMixer4              mixFinal;       // final mix: kick + snare

// Mixers
//AudioMixer4              mixBody;        // kick body mix (4 oscillators)
//AudioMixer4              mixFinal;       // final mix: kick + snare

// Drive / distortion
AudioEffectWaveshaper    driveFX;

// Master volume (final stage)
AudioMixer4              mixMaster;      // final volume control

// Output
AudioOutputI2S           i2s1;

AudioControlSGTL5000     audioShield;

// ---------- Patch cords ----------

// Kick body
AudioConnection patchCord1(oscBody,  0, mixBody, 0);
AudioConnection patchCord2(oscBody2, 0, mixBody, 1);
AudioConnection patchCord3(oscTri1,  0, mixBody, 2);
AudioConnection patchCord4(oscTri2,  0, mixBody, 3);
AudioConnection patchCord5(mixBody,       oscFilter);        // summed osc -> filter
AudioConnection patchCord6(oscFilter, 0,  oscDrive, 0);      // LP out -> drive
AudioConnection patchCord7(oscDrive,  0,  envAmp,   0);      // driven osc -> amp env
AudioConnection patchCord8(envAmp,    0,  mixKick,  0);      // amp env -> kick mixer

// Transient path
AudioConnection patchCord8b(noiseTransient, transientFilter);
AudioConnection patchCord8c(transientFilter, 2, envTransient, 0); // HP out (2) for clicky tick
AudioConnection patchCord8d(envTransient, 0, mixKick, 1);    // transient -> kick mixer

AudioConnection patchCord8e(mixKick, 0, mixFinal, 0);        // combined kick -> final mix

// Kick noise
AudioConnection patchCord9(noise,           noiseFilter);
AudioConnection patchCord10(noiseFilter, 1,  envNoise, 0);
AudioConnection patchCord11(envNoise,    0,  mixFinal, 1);  // kick noise

// Snare
AudioConnection patchCord12(oscSnareTone, envSnareTone);
AudioConnection patchCord13(envSnareTone, 0, mixFinal, 2);

AudioConnection patchCord14(noise,            snareNoiseFilter);
AudioConnection patchCord15(snareNoiseFilter, 1, envSnareNoise, 0);
AudioConnection patchCord16(envSnareNoise, 0,  mixFinal, 3);

// Drive -> master volume -> output
AudioConnection patchCord17(mixFinal, 0, driveFX, 0);
AudioConnection patchCord18(driveFX, 0, mixMaster, 0);
AudioConnection patchCord19(mixMaster, 0, i2s1, 0);
AudioConnection patchCord20(mixMaster, 0, i2s1, 1);

// ---------- Triggers ----------
const int TRIG_KICK  = 2;
const int TRIG_SNARE = 3;

// ---------- Globals ----------

float masterGain = 0.12f;   // global output trim (target value)
float masterGainSmooth = 0.12f;  // smoothed version to prevent clicks
const float mixFinalBaseGains[4] = {0.7f, 0.4f, 0.4f, 0.6f};

// Oscillator levels
float kickOsc1LevelNorm = 1.0f; // K1 – Sine A
float kickOsc2LevelNorm = 0.5f; // K2 – Sine B
float tri1LevelNorm     = 0.0f; // T1 – Tri A
float tri2LevelNorm     = 0.0f; // T2 – Tri B

// Sine B ratio
float osc2RatioNorm     = 0.5f; // KR – 0.5..3x

// Pitch envelopes for 4 oscillators
PitchEnv oscEnv[4];

// AMP A envelope (normalized controls)
float ampAAttackNorm    = 0.0f; // AA
float ampADecayNorm     = 0.4f; // AD
float ampAExtLvlNorm    = 0.0f; // AE
float ampAExtFacNorm    = 0.5f; // AX

// Noise engine
float noiseLevelNorm      = 0.3f; // NL - level
float noiseCutoffNorm     = 0.6f; // NF - freq cutoff
float noiseResonanceNorm  = 0.5f; // NQ - rez
float noiseAttackNorm     = 0.0f; // NA - attack
float noiseDecayNorm      = 0.2f; // ND - decay

// Transient (tick/clap at start)
float transientLevelNorm  = 0.0f;  // TL - level
float transientCutoffNorm = 0.7f;  // TF - high-pass cutoff for clicky sound
float transientDecayNorm  = 0.1f;  // TD - very short decay

// osc sum filter & dist
float oscCutoffNorm  = 0.5f;  // OC – cutoff
float oscQNorm       = 0.3f;  // OQ – resonance
float oscDistNorm    = 0.2f;  // OD – drive

// Filter envelope (post-osc filter)
float filtEnvAttackNorm = 0.0f;  // FA
float filtEnvDecayNorm  = 0.4f;  // FD
float filtEnvAmountNorm = 0.5f;  // FM

// Drive / distortion
float driveNorm           = 0.0f;    // DR
float driveTable[257];               // -1..+1 waveshape

// Pitch envelope timing
elapsedMillis pitchTimer;
bool          pitchEnvActive = false;

// Filter envelope timing
elapsedMillis filterEnvTimer;
bool          filterEnvActive = false;
float         filterEnvCapturedAtkMs = 0.0f;  // Attack time captured at trigger
float         filterEnvCapturedDecMs = 0.0f;  // Decay time captured at trigger

// Trigger state
int lastKickTrig  = HIGH;
int lastSnareTrig = HIGH;

// Serial parser
String serialLine;

// ---------- Helpers ----------

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static inline float computeSimpleAD(float tMs, float attackMs, float decayMs) {
  if (attackMs < 0.5f) attackMs = 0.5f;
  if (decayMs  < 0.5f) decayMs  = 0.5f;

  if (tMs < attackMs) {
    return tMs / attackMs;
  }
  float td = tMs - attackMs;
  if (td < decayMs) {
    return 1.0f - (td / decayMs);
  }
  return 0.0f;
}

// Drive curve - High quality asymmetric distortion
void updateDriveCurve() {
  // DR = 0 -> y = x (clean)
  if (driveNorm <= 0.001f) {
    for (int i = 0; i < 257; i++) {
      float x = (float(i) - 128.0f) / 128.0f;  // -1..+1
      driveTable[i] = x;
    }
    driveFX.shape(driveTable, 257);
    return;
  }

  // High-quality distortion with asymmetric soft clipping
  // Drive amount controls both gain and saturation character
  float driveAmount = driveNorm;
  
  // Gain staging: keep range musical (less raw level jump)
  float preGain = 1.0f + driveAmount * 8.0f;  // 1x to 9x
  
  // Asymmetry factor: slight asymmetry adds character (tube-like)
  float asymmetry = 0.85f + driveAmount * 0.15f;  // 0.85 to 1.0
  
  // Normalization target for consistent output level
  float testInput = 0.8f;  // test at 80% to avoid edge cases
  float testPos = tanhf(preGain * testInput) / (1.0f + fabsf(testInput * preGain * 0.3f));
  float testNeg = tanhf(preGain * testInput * asymmetry) / (1.0f + fabsf(testInput * preGain * asymmetry * 0.3f));
  float maxTest = (fabsf(testPos) > fabsf(testNeg)) ? fabsf(testPos) : fabsf(testNeg);
  float normScale = 1.0f / maxTest;
  if (normScale > 2.0f) normScale = 2.0f;  // prevent excessive gain

  float sumSquares = 0.0f;

  for (int i = 0; i < 257; i++) {
    float x = (float(i) - 128.0f) / 128.0f;  // -1..+1
    
    // Apply pre-gain
    float driven = x * preGain;
    
    // Asymmetric soft clipping: different curves for positive/negative
    float y;
    if (driven >= 0.0f) {
      // Positive: standard tanh with gentle compression
      y = tanhf(driven) / (1.0f + fabsf(driven) * 0.3f);
    } else {
      // Negative: asymmetric curve (slightly different response)
      float asymDriven = driven * asymmetry;
      y = tanhf(asymDriven) / (1.0f + fabsf(asymDriven) * 0.35f);
    }
    
    // Apply normalization to maintain output level
    y *= normScale;
    
    // Add subtle harmonic enhancement at medium-high drive
    if (driveAmount > 0.3f) {
      float harmonic = sinf(x * 3.14159f) * 0.08f * driveAmount;
      y += harmonic * (1.0f - fabsf(x));
    }
    
    // Safety clamp
    if (y > 1.0f)  y = 1.0f;
    if (y < -1.0f) y = -1.0f;
    
    driveTable[i] = y;
    sumSquares += y * y;
  }

  // Loudness compensation: aim for roughly constant RMS regardless of drive
  float rms = (sumSquares > 0.0f) ? sqrtf(sumSquares / 257.0f) : 0.0f;
  float targetRms = 0.25f;  // lower target to tame high settings
  float postScale = (rms > 0.0001f) ? (targetRms / rms) : 1.0f;
  if (postScale > 1.0f) postScale = 1.0f;          // avoid boosting
  if (postScale < 0.2f) postScale = 0.2f;          // strong attenuation at high drive

  if (fabsf(postScale - 1.0f) > 0.01f) {
    for (int i = 0; i < 257; i++) {
      driveTable[i] *= postScale;
      if (driveTable[i] > 1.0f)  driveTable[i] = 1.0f;
      if (driveTable[i] < -1.0f) driveTable[i] = -1.0f;
    }
  }
  driveFX.shape(driveTable, 257);
}

// Compute per-oscillator pitch frequency from PitchEnv and time in ms
float computePitchFreq(const PitchEnv &p, float tMs) {
  // Deeper ranges:
  // base:   10..200 Hz
  // attack: 0..60 ms
  // decay:  10..1000 ms
  // amount: 0..1500 Hz
  // extend: 0 -> no drop, 1 -> slow decay below base pitch to tail

  float baseHz = 10.0f + p.base * 190.0f;
  float atkMs  = p.attack * 200.0f;
  float decMs  = 10.0f + p.decay * 990.0f;
  float amtHz  = p.amount * 800.0f;

  if (decMs < 1.0f) decMs = 1.0f;

  // Base attack/decay envelope (amount is NOT affected by extend)
  float env = 0.0f;
  if (tMs < atkMs && atkMs > 0.0f) {
    env = tMs / atkMs;  // 0->1
  } else {
    float td = tMs - atkMs;
    if (td < decMs) {
      env = 1.0f - (td / decMs);  // 1->0
    } else {
      env = 0.0f;
    }
  }

  // Main pitch from envelope (extend does NOT affect this)
  float freq = baseHz + env * amtHz;

  // Extend: sweep from normal pitch (base) down to lower pitch over extended tail
  // Starts AFTER the main decay finishes, sweeps from baseHz to (baseHz - dropAmount)
  if (p.extend > 0.001f) {
    // When main envelope finishes (returns to base pitch)
    float mainEnvEndMs = atkMs + decMs;
    
    // Extended tail time: much longer than main decay
    float extTailMs = decMs * (2.0f + p.extend * 8.0f);  // 2x to 10x longer
    
    // Drop amount: how far below base to sweep
    float dropAmount = baseHz * p.extend * 0.8f;  // drop up to 80% of base pitch
    
    // Start the sweep after main decay finishes
    if (tMs > mainEnvEndMs) {
      // Time since sweep started
      float sweepTime = tMs - mainEnvEndMs;
      
      // Sweep envelope: 0 (at start) to 1 (at end of tail)
      // Smooth exponential curve for gradual sweep
      float sweepEnv = 1.0f - expf(-sweepTime / (extTailMs * 0.6f));
      
      // Sweep from baseHz down to (baseHz - dropAmount)
      // At start: freq = baseHz (normal pitch after decay)
      // At end: freq = baseHz - dropAmount (lower pitch)
      freq = baseHz - (dropAmount * sweepEnv);
    }
  }

  if (freq < 5.0f) freq = 5.0f;  // Safety clamp
  return freq;
}

// ---------- Serial parsing ----------

void parseSerialLine(const String &line) {
  if (line.length() < 3) return;

  char c0 = line.charAt(0);
  char c1 = line.charAt(1);
  String arg = line.substring(3);
  float val  = arg.toFloat();
  val = clamp01(val);

  // ---- New: space-bar / GUI trigger for kick ----
  if (c0 == 'T' && c1 == 'K') {
    triggerKick();
    return;
  }

  // Osc levels & ratio
  if      (c0 == 'K' && c1 == '1') kickOsc1LevelNorm = val;
  else if (c0 == 'K' && c1 == '2') kickOsc2LevelNorm = val;
  else if (c0 == 'T' && c1 == '1') tri1LevelNorm     = val;
  else if (c0 == 'T' && c1 == '2') tri2LevelNorm     = val;
  else if (c0 == 'K' && c1 == 'R') osc2RatioNorm     = val;

  // AMP A envelope
  else if (c0 == 'A' && c1 == 'A') ampAAttackNorm    = val;
  else if (c0 == 'A' && c1 == 'D') ampADecayNorm     = val;
  else if (c0 == 'A' && c1 == 'E') ampAExtLvlNorm    = val;
  else if (c0 == 'A' && c1 == 'X') ampAExtFacNorm    = val;

  // Noise engine
  else if (c0 == 'N' && c1 == 'L') noiseLevelNorm    = val;
  else if (c0 == 'N' && c1 == 'F') noiseCutoffNorm   = val;
  else if (c0 == 'N' && c1 == 'Q') noiseResonanceNorm= val;
  else if (c0 == 'N' && c1 == 'A') noiseAttackNorm   = val;
  else if (c0 == 'N' && c1 == 'D') noiseDecayNorm    = val;

  // Transient
  else if (c0 == 'T' && c1 == 'L') transientLevelNorm  = val;
  else if (c0 == 'T' && c1 == 'F') transientCutoffNorm = val;
  else if (c0 == 'T' && c1 == 'D') transientDecayNorm  = val;

  // Master volume
  else if (c0 == 'M' && c1 == 'V') masterGain = val;

   // --- OSC tone section ---
  else if (c0 == 'O' && c1 == 'C') oscCutoffNorm = val;   // cutoff
  else if (c0 == 'O' && c1 == 'Q') oscQNorm      = val;   // resonance
  else if (c0 == 'O' && c1 == 'D') oscDistNorm   = val;   // distortion amount

  // Filter envelope (post-osc filter)
  else if (c0 == 'F' && c1 == 'A') filtEnvAttackNorm = val;
  else if (c0 == 'F' && c1 == 'D') filtEnvDecayNorm  = val;
  else if (c0 == 'F' && c1 == 'M') filtEnvAmountNorm = val;

  // Drive
  else if (c0 == 'D' && c1 == 'R') {
    driveNorm = val;
    updateDriveCurve();
  }

  // Per-oscillator pitch envelopes:
  // 0B,0A,0D,0M,0X ... 3B,3A,3D,3M,3X
  else if (c0 >= '0' && c0 <= '3') {
    int idx = c0 - '0';
    if (idx < 0 || idx > 3) return;

    if      (c1 == 'B') oscEnv[idx].base   = val;
    else if (c1 == 'A') oscEnv[idx].attack = val;
    else if (c1 == 'D') oscEnv[idx].decay  = val;
    else if (c1 == 'M') oscEnv[idx].amount = val;
    else if (c1 == 'X') oscEnv[idx].extend = val;
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        parseSerialLine(serialLine);
        serialLine = "";
      }
    } else {
      serialLine += c;
      if (serialLine.length() > 80) serialLine = "";
    }
  }
}

// ---------- Triggers ----------

void triggerKick() {
  pitchTimer = 0;
  pitchEnvActive = true;
  filterEnvTimer = 0;
  filterEnvActive = true;
  
  // Capture filter envelope attack/decay times at trigger to prevent premature deactivation
  // if user changes controls while envelope is playing
  filterEnvCapturedAtkMs = filtEnvAttackNorm * 80.0f;           // 0..80 ms
  filterEnvCapturedDecMs = 5.0f + filtEnvDecayNorm * 2000.0f;   // 5..2005 ms

  envAmp.noteOn();
  envNoise.noteOn();
  envTransient.noteOn();  // trigger transient tick/clap
}

void triggerSnare() {
  envSnareTone.noteOn();
  envSnareNoise.noteOn();
}

// ---------- Setup ----------

void setup() {
  pinMode(TRIG_KICK,  INPUT_PULLUP);
  pinMode(TRIG_SNARE, INPUT_PULLUP);

  Serial.begin(115200);

  AudioMemory(40);

  audioShield.enable();
  // Initialize audioShield volume from masterGain (will be updated in loop)
  audioShield.volume(masterGain);

  // Init oscillators
  oscBody.begin(WAVEFORM_SINE);
  oscBody.amplitude(1.0f);

  oscBody2.begin(WAVEFORM_SINE);
  oscBody2.amplitude(1.0f);

  oscTri1.begin(WAVEFORM_TRIANGLE);
  oscTri1.amplitude(1.0f);

  oscTri2.begin(WAVEFORM_TRIANGLE);
  oscTri2.amplitude(1.0f);

  // Initial pitch env defaults (rough 808-ish)
  oscEnv[0].base   = 0.30f;  // Sine A
  oscEnv[0].attack = 0.0f;
  oscEnv[0].decay  = 0.5f;
  oscEnv[0].amount = 0.9f;
  oscEnv[0].extend = 0.2f;

  oscEnv[1].base   = 0.40f;  // Sine B
  oscEnv[1].attack = 0.0f;
  oscEnv[1].decay  = 0.4f;
  oscEnv[1].amount = 0.7f;
  oscEnv[1].extend = 0.2f;

  oscEnv[2].base   = 0.50f;  // Tri A
  oscEnv[2].attack = 0.0f;
  oscEnv[2].decay  = 0.3f;
  oscEnv[2].amount = 0.5f;
  oscEnv[2].extend = 0.3f;

  oscEnv[3].base   = 0.60f;  // Tri B
  oscEnv[3].attack = 0.0f;
  oscEnv[3].decay  = 0.3f;
  oscEnv[3].amount = 0.5f;
  oscEnv[3].extend = 0.3f;

  // AMP A env (values updated every loop from normalized)
  envAmp.attack(0.0f);
  envAmp.decay(200.0f);
  envAmp.sustain(0.0f);
  envAmp.release(0.0f);

  // Noise env
  envNoise.attack(0.0f);
  envNoise.decay(40.0f);
  envNoise.sustain(0.0f);
  envNoise.release(0.0f);

  // Transient (very short tick/clap)
  envTransient.attack(0.0f);
  envTransient.decay(5.0f);  // very short, will be updated in loop
  envTransient.sustain(0.0f);
  envTransient.release(0.0f);

  noiseTransient.amplitude(0.0f);  // will be updated in loop

  transientFilter.frequency(3000.0f);  // high-pass for clicky sound
  transientFilter.resonance(1.0f);
  transientFilter.octaveControl(2.0f);  // high-pass mode

  // Noise filters
  noise.amplitude(noiseLevelNorm);

  noiseFilter.frequency(6000.0f);
  noiseFilter.resonance(2.0f);
  noiseFilter.octaveControl(1.0f);

  snareNoiseFilter.frequency(2500.0f);
  snareNoiseFilter.resonance(1.0f);
  snareNoiseFilter.octaveControl(1.0f);

  // Snare tone
  oscSnareTone.begin(WAVEFORM_TRIANGLE);
  oscSnareTone.frequency(180.0f);
  oscSnareTone.amplitude(0.7f);

  envSnareTone.attack(0.0f);
  envSnareTone.decay(120.0f);
  envSnareTone.sustain(0.0f);
  envSnareTone.release(0.0f);

  envSnareNoise.attack(0.0f);
  envSnareNoise.decay(160.0f);
  envSnareNoise.sustain(0.0f);
  envSnareNoise.release(0.0f);

  // Kick body mixer initial gains
  mixBody.gain(0, kickOsc1LevelNorm * 0.6f);
  mixBody.gain(1, kickOsc2LevelNorm * 0.6f);
  mixBody.gain(2, tri1LevelNorm     * 0.6f);
  mixBody.gain(3, tri2LevelNorm     * 0.6f);

  // --- OSC filter / drive initial setup ---
  oscFilter.frequency(800.0f);     // just a starting value
  oscFilter.resonance(1.0f);
  oscFilter.octaveControl(3.0f);   // wide sweep range, we'll still update every loop

  // Initialize OSC waveshaper with identity table (clean pass-through)
  float initOscShape[257];
  for (int i = 0; i < 257; i++) {
    float x = (float(i) - 128.0f) / 128.0f;  // -1..+1
    initOscShape[i] = x;
  }
  oscDrive.shape(initOscShape, 257);

  // Initialize smoothed master gain
  masterGainSmooth = masterGain;
  
  // Kick mixer (body + transient)
  mixKick.gain(0, 1.0f);   // kick body
  mixKick.gain(1, transientLevelNorm * 0.5f);  // transient (will be updated in loop)
  mixKick.gain(2, 0.0f);
  mixKick.gain(3, 0.0f);

  // Final mix gains (no master volume here - applied at end of chain)
  mixFinal.gain(0, mixFinalBaseGains[0]);  // kick (body + transient combined)
  mixFinal.gain(1, mixFinalBaseGains[1]);  // kick noise
  mixFinal.gain(2, mixFinalBaseGains[2]);  // snare tone
  mixFinal.gain(3, mixFinalBaseGains[3]);  // snare noise
  
  // Master volume mixer (final stage)
  // Initialize all channels - mixer just passes signal through
  // Actual volume control is via audioShield.volume()
  mixMaster.gain(0, 1.0f);
  mixMaster.gain(1, 0.0f);
  mixMaster.gain(2, 0.0f);
  mixMaster.gain(3, 0.0f);

  // Initialize drive curve
  updateDriveCurve();
}

// ---------- Loop ----------

void loop() {
  handleSerial();

  // Triggers
  int k = digitalRead(TRIG_KICK);
  if (lastKickTrig == HIGH && k == LOW) {
    triggerKick();
  }
  lastKickTrig = k;

  int s = digitalRead(TRIG_SNARE);
  if (lastSnareTrig == HIGH && s == LOW) {
    triggerSnare();
  }
  lastSnareTrig = s;

  float tMs = pitchTimer;

  // Pitch envelopes for 4 oscillators
  float f0 = computePitchFreq(oscEnv[0], tMs);  // Sine A
  float f1 = computePitchFreq(oscEnv[1], tMs);  // Sine B
  float f2 = computePitchFreq(oscEnv[2], tMs);  // Tri A
  float f3 = computePitchFreq(oscEnv[3], tMs);  // Tri B

  // Apply Sine B ratio (KR)
  float ratio = 0.5f + osc2RatioNorm * 2.5f;    // 0.5..3.0
  f1 *= ratio;

  if (f0 < 5.0f) f0 = 5.0f;
  if (f1 < 5.0f) f1 = 5.0f;
  if (f2 < 5.0f) f2 = 5.0f;
  if (f3 < 5.0f) f3 = 5.0f;

  oscBody.frequency(f0);
  oscBody2.frequency(f1);
  oscTri1.frequency(f2);
  oscTri2.frequency(f3);

  // Stop pitch env after a while (avoid wasting CPU)
  // Account for extended tail: 2x to 10x longer than main decay
  float maxDec = 0.0f;
  for (int i = 0; i < 4; i++) {
    float dMs = 10.0f + oscEnv[i].decay * 990.0f;
    // New extend: tail is 2x to 10x longer than decay
    float extTailMs = dMs * (2.0f + oscEnv[i].extend * 8.0f);
    if (extTailMs > maxDec) maxDec = extTailMs;
  }
  if (pitchEnvActive && tMs > (maxDec + 200.0f)) {
    pitchEnvActive = false;
  }

  // AMP A envelope mapping (deeper)
  float ampAttackMs = ampAAttackNorm * 80.0f;           // 0..80 ms
  float baseDecayMs = 10.0f + ampADecayNorm * 1990.0f;  // 10..2000 ms
  float extendScale = 1.0f + ampAExtLvlNorm * 3.0f;     // 1x..4x
  float curve       = 0.3f + ampAExtFacNorm * 1.7f;     // 0.3..2.0

  float normDecay   = baseDecayMs / 2000.0f;
  float shapedNorm  = powf(normDecay, curve);
  float shapedDecayMs = shapedNorm * 2000.0f * extendScale;

  envAmp.attack(ampAttackMs);
  envAmp.decay(shapedDecayMs);
  envAmp.sustain(0.0f);
  envAmp.release(0.0f);

  // Noise engine mapping (deeper-ish)
  noise.amplitude(noiseLevelNorm * 0.3);

  float noiseFreq = 120.0f + noiseCutoffNorm * 7000.0f;   // 300..7000 Hz
  noiseFilter.frequency(noiseFreq);

  float noiseQ = 0.7f + noiseResonanceNorm * 5.3f;         // 0.7..6.0
  noiseFilter.resonance(noiseQ);

  float nAttackMs = noiseAttackNorm * 500.0f;               // 0..60 ms
  float nDecayMs  = 5.0f + noiseDecayNorm * 3000.0f;        // 5..500 ms

  envNoise.attack(nAttackMs);
  envNoise.decay(nDecayMs);
  envNoise.sustain(0.0f);
  envNoise.release(0.0f);

  // Transient mapping (very short tick/clap)
  noiseTransient.amplitude(transientLevelNorm * 0.4f);  // scale level

  // High-pass cutoff for clicky tick sound: 2000 Hz to 18000 Hz
  float transientFreq = 2000.0f + transientCutoffNorm * 16000.0f;
  transientFilter.frequency(transientFreq);

  // Very short decay: 1 ms to 30 ms
  float transientDecayMs = 1.0f + transientDecayNorm * 29.0f;
  envTransient.attack(0.0f);
  envTransient.decay(transientDecayMs);
  envTransient.sustain(0.0f);
  envTransient.release(0.0f);

  // Update kick mixer gains
  mixKick.gain(0, 1.0f);   // kick body always at 1.0
  mixKick.gain(1, transientLevelNorm * 0.5f);  // transient level

    // --- OSC tone shaping (filter + distortion) ---

  // Cutoff: keep it more kick-focused and less fizz
  // 0..1 => ~80 Hz .. ~4000 Hz
  float oscCutHz = 80.0f + oscCutoffNorm * 3920.0f;
  
  // Use captured attack/decay times for envelope computation and deactivation check
  // This prevents premature deactivation if user changes FA/FD controls while playing
  float filtAtkMs = filterEnvActive ? filterEnvCapturedAtkMs : (filtEnvAttackNorm * 80.0f);
  float filtDecMs = filterEnvActive ? filterEnvCapturedDecMs : (5.0f + filtEnvDecayNorm * 2000.0f);
  
  float filterEnv = filterEnvActive ? computeSimpleAD(filterEnvTimer, filtAtkMs, filtDecMs) : 0.0f;
  float envSweepHz = filtEnvAmountNorm * 4000.0f * filterEnv;  // up to +4 kHz
  oscFilter.frequency(oscCutHz + envSweepHz);
  if (filterEnvActive && filterEnv <= 0.0f && filterEnvTimer > (filtAtkMs + filtDecMs)) {
    filterEnvActive = false;
  }

  // Q: 0..1 => 0.7 .. 8.0
  float oscQ = 0.7f + oscQNorm * 7.3f;
  oscFilter.resonance(oscQ);

  // Distortion: OD (oscDistNorm) 0..1
  // 0    => perfectly clean (identity curve)
  // 0.2  => subtle saturation
  // 0.5+ => beefy, but controlled
  float drive = oscDistNorm;

  static float oscShape[257];
  static float lastDrive = -1.0f;  // Track previous drive value

  if (drive < 0.001f) {
    // Identity waveshape: y = x (completely clean)
    // Only recalculate if drive changed from non-zero to zero
    if (lastDrive >= 0.001f || lastDrive < 0.0f) {
      // Ensure perfect linear mapping to avoid discontinuities
      for (int i = 0; i < 257; i++) {
        float x = (float(i) - 128.0f) / 128.0f;  // -1..+1
        // Ensure exact linear mapping with proper bounds
        oscShape[i] = x;
      }
      oscDrive.shape(oscShape, 257);
    }
    lastDrive = drive;
    // Skip the rest of distortion calculation
    envNoise.attack(nAttackMs);
    envNoise.decay(nDecayMs);
    envNoise.sustain(0.0f);
    envNoise.release(0.0f);
    
    // Update body mixer from level controls
    mixBody.gain(0, kickOsc1LevelNorm * 0.6f);
    mixBody.gain(1, kickOsc2LevelNorm * 0.6f);
    mixBody.gain(2, tri1LevelNorm     * 0.6f);
    mixBody.gain(3, tri2LevelNorm     * 0.6f);
    return;  // Early return to avoid recalculating
  }
  
  // Non-zero drive: apply distortion
  {
    // Pregain: how hard we hit the nonlinearity
    float pregain  = 1.0f + drive * 14.0f;       // up to ~15x
    // Postgain: keep overall loudness from exploding
    float postgain = 1.0f / (1.0f + drive * 3.0f);

    for (int i = 0; i < 257; i++) {
      float x = (float(i) - 128.0f) / 128.0f;    // -1..+1
      float y = pregain * x;

      // Curve 1: soft clip (tube-ish) y / (1 + |y|)
      float y1 = y / (1.0f + fabsf(y));

      // Curve 2: arctan "tape-ish" saturation
      float y2 = atan2f(y, 1.0f) * (2.0f / 3.14159265f);  // roughly -1..+1

      // Morph between them as drive increases
      // low drive = more soft clip, high drive = more arctan
      float mix = drive;  // 0..1
      float z = (1.0f - mix) * y1 + mix * y2;

      // Apply postgain
      z *= postgain;

      // Safety clamp
      if (z > 1.0f)  z = 1.0f;
      if (z < -1.0f) z = -1.0f;

      oscShape[i] = z;
    }
    
    // Update waveshaper when drive changes
    if (fabsf(drive - lastDrive) > 0.001f) {
      oscDrive.shape(oscShape, 257);
    }
    lastDrive = drive;
  }

  envNoise.attack(nAttackMs);
  envNoise.decay(nDecayMs);
  envNoise.sustain(0.0f);
  envNoise.release(0.0f);

  // Update body mixer from level controls
  mixBody.gain(0, kickOsc1LevelNorm * 0.6f);
  mixBody.gain(1, kickOsc2LevelNorm * 0.6f);
  mixBody.gain(2, tri1LevelNorm     * 0.6f);
  mixBody.gain(3, tri2LevelNorm     * 0.6f);

  // Update master volume (final stage - after all processing)
  // Smooth master gain to prevent clicks when dragging slider
  // Interpolate toward target at ~10ms time constant (smooth but responsive)
  float smoothRate = 0.08f;  // adjust this: higher = faster, lower = smoother
  float diff = masterGain - masterGainSmooth;
  if (fabsf(diff) > 0.0001f) {  // only update if there's a meaningful difference
    masterGainSmooth += diff * smoothRate;
  } else {
    masterGainSmooth = masterGain;  // snap to target when very close
  }
  
  // Apply master volume at the very end of the chain (after distortion)
  // Use both mixer and audioShield for reliable control
  float gainTrimLive = masterGainSmooth;
  // Set mixer gain (keep at 1.0 for signal pass-through)
  mixMaster.gain(0, 1.0f);
  mixMaster.gain(1, 0.0f);
  mixMaster.gain(2, 0.0f);
  mixMaster.gain(3, 0.0f);
  // Use audioShield volume for actual master volume control (more reliable)
  // Map 0.0-1.0 to 0.0-1.0 range for audioShield
  audioShield.volume(gainTrimLive);
}
