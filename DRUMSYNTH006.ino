/*
 * DRUMSYNTH006.ino
 * Version: 1.1.1 (Snapshot of DRUMSYNTH005)
 * 
 * Teensy 4.x Drum Synthesizer - Kick/Snare with 909-style core
 * 
 * This is a snapshot of DRUMSYNTH005.ino - the working version before further changes.
 * 
 * Recent Changes (v1.1.1):
 * - Increased smoothing for MAIN VOLUME and OSC DISTORT sliders (0.03 rate) to eliminate clicking/poping when dragging
 * 
 * Previous Changes (v1.1.0):
 * - Reordered audio processing chain: Distortion now occurs BEFORE filter (distortion -> filter -> envelope)
 *   This allows the filter to shape the distorted signal more effectively
 * - Updated filter cutoff minimum range from 80 Hz to 120 Hz
 * - Added guitar overdrive-style distortion algorithm with asymmetric soft clipping, harmonic enhancement, and soft-knee compression
 * - 909 core implementation on oscillator 3 with specialized pitch envelope
 * - Added transient section (tick/clap) for punch at the beginning of sounds
 * - Master volume control via audioShield.volume() for reliable operation
 * - Filter envelope timing fixes to prevent premature deactivation
 */

// DrumSynth005NewCore - Teensy 4.x kick/snare with 909-style core on osc3
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

// ---------- Simple AD Helper For Filter Env ----------
float computeSimpleAD(float tMs, float attackMs, float decayMs) {
  if (attackMs < 0.1f) attackMs = 0.1f;
  if (decayMs  < 0.1f) decayMs  = 0.1f;

  if (tMs < attackMs) {
    return tMs / attackMs;                // 0 -> 1
  } else {
    float td = tMs - attackMs;
    if (td < decayMs) {
      return 1.0f - (td / decayMs);       // 1 -> 0
    } else {
      return 0.0f;
    }
  }
}

// ---------- Audio objects ----------

// Kick: body oscillators
AudioSynthWaveform       oscBody;        // Sine A (osc 0)
AudioSynthWaveform       oscBody2;       // Sine B (osc 1)
AudioSynthWaveform       oscTri1;        // Tri A  (osc 2)
AudioSynthWaveform       oscTri2;        // Osc 3: 909 core / helper
AudioEffectEnvelope      envAmp;         // AMP A (kick amp envelope)

// Kick: noise
AudioSynthNoiseWhite     noise;          // shared noise source
AudioFilterStateVariable noiseFilter;    // kick noise filter
AudioEffectEnvelope      envNoise;       // kick noise envelope

// Transient (tick/clap at start)
AudioSynthNoiseWhite     noiseTransient;   // separate noise for transient
AudioFilterStateVariable transientFilter;  // high-pass for clicky tick sound
AudioEffectEnvelope      envTransient;     // very short envelope for transient

// SNARE: tone/body
AudioSynthWaveform       oscSnareTone;
AudioEffectEnvelope      envSnareTone;

// SNARE: noise
AudioFilterStateVariable snareNoiseFilter;
AudioEffectEnvelope      envSnareNoise;

// Kick body + transient mix
AudioMixer4              mixBody;        // kick body mix (4 oscillators)
AudioMixer4              mixKick;        // body + transient submix

// Post-osc filter & distortion
AudioFilterStateVariable oscFilter;      // filter after osc sum
AudioEffectWaveshaper    oscDrive;       // distortion for osc path

// Final mix & master drive
AudioMixer4              mixFinal;       // final mix: kick + snare
AudioEffectWaveshaper    driveFX;        // master waveshaper / compressor

// Output
AudioOutputI2S           i2s1;

AudioControlSGTL5000     audioShield;

// ---------- Patch cords ----------

// Kick body
AudioConnection patchCord1(oscBody,  0, mixBody, 0);
AudioConnection patchCord2(oscBody2, 0, mixBody, 1);
AudioConnection patchCord3(oscTri1,  0, mixBody, 2);
AudioConnection patchCord4(oscTri2,  0, mixBody, 3);

// Transient
AudioConnection patchCord5(noiseTransient, transientFilter);
AudioConnection patchCord6(transientFilter, 0, envTransient, 0);
AudioConnection patchCord7(envTransient, 0, mixKick, 1);   // transient -> mixKick ch1

// Body + transient -> drive/filter (distortion before filter for better filter effect)
AudioConnection patchCord8(mixBody, 0, mixKick, 0);        // body -> mixKick ch0
AudioConnection patchCord9(mixKick, oscDrive);             // mixKick output -> oscDrive (distort first)
AudioConnection patchCord10(oscDrive, 0, oscFilter, 0);    // distorted -> oscFilter (filter after)
AudioConnection patchCord11(oscFilter, 0, envAmp, 0);      // filtered -> amp env
AudioConnection patchCord12(envAmp, 0, mixFinal, 0);       // amp env -> final mix

// Kick noise
AudioConnection patchCord13(noise, noiseFilter);
AudioConnection patchCord14(noiseFilter, 1, envNoise, 0);
AudioConnection patchCord15(envNoise, 0, mixFinal, 1);     // kick noise -> final

// Snare
AudioConnection patchCord16(oscSnareTone, envSnareTone);
AudioConnection patchCord17(envSnareTone, 0, mixFinal, 2);

AudioConnection patchCord18(noise, snareNoiseFilter);
AudioConnection patchCord19(snareNoiseFilter, 1, envSnareNoise, 0);
AudioConnection patchCord20(envSnareNoise, 0, mixFinal, 3);

// Master drive -> outputs
AudioConnection patchCord21(mixFinal, 0, driveFX, 0);
AudioConnection patchCord22(driveFX, 0, i2s1, 0);
AudioConnection patchCord23(driveFX, 0, i2s1, 1);

// ---------- Triggers ----------
const int TRIG_KICK  = 2;
const int TRIG_SNARE = 3;

// ---------- Globals ----------

float masterGain = 0.12f;          // global output trim (target value)
float masterGainSmooth = 0.12f;    // smoothed master for de-click
const float mixFinalBaseGains[4] = {0.7f, 0.4f, 0.4f, 0.6f};

// Oscillator levels
float kickOsc1LevelNorm = 1.0f; // K1 – Sine A
float kickOsc2LevelNorm = 0.5f; // K2 – Sine B
float tri1LevelNorm     = 0.0f; // T1 – Tri A
float tri2LevelNorm     = 0.4f; // T2 – 909 core level

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

// Transient (tick / click)
float transientLevelNorm  = 0.7f; // TL - level
float transientCutoffNorm = 0.6f; // TF - HP cutoff
float transientDecayNorm  = 0.1f; // TD - very short decay

// osc sum filter & dist
float oscCutoffNorm  = 0.5f;  // OC – cutoff
float oscQNorm       = 0.3f;  // OQ – resonance
float oscDistNorm    = 0.2f;  // OD – drive (target value)
float oscDistNormSmooth = 0.2f;  // smoothed drive for de-click

// Filter envelope (post-osc filter)
float filtEnvAttackNorm = 0.0f;  // FA
float filtEnvDecayNorm  = 0.4f;  // FD
float filtEnvAmountNorm = 0.5f;  // FM

// Drive / distortion
float driveNorm           = 0.0f;       // DR
float driveTable[257];                  // -1..+1 waveshape

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

// ---------- Drive curve (master) ----------

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

  // DR > 0 -> Guitar overdrive pedal simulation
  // Pregain: how hard we hit the nonlinearity (like drive knob)
  float pregain = 1.0f + driveNorm * 8.0f;  // 1..9x
  
  // Tube-like asymmetric clipping with harmonic enhancement
  for (int i = 0; i < 257; i++) {
    float x = (float(i) - 128.0f) / 128.0f;  // -1..+1
    float driven = x * pregain;
    
    // Asymmetric soft clipping (tube-like behavior)
    // Positive side clips softer, negative side clips harder
    float out;
    if (driven >= 0.0f) {
      // Positive side: soft tube-like saturation
      out = tanhf(driven * 1.2f);
    } else {
      // Negative side: slightly harder clipping (more asymmetric)
      float neg = -driven;  // make positive
      out = -tanhf(neg * 1.4f);  // harder clipping
    }
    
    // Add subtle harmonic enhancement (like tube warmth)
    float harmonic = sinf(x * 3.14159f) * 0.05f * driveNorm;
    out += harmonic;
    
    // Soft knee compression-like behavior
    float compRatio = 1.0f + driveNorm * 2.0f;
    if (fabsf(out) > 0.7f) {
      float excess = fabsf(out) - 0.7f;
      float compressed = 0.7f + excess / compRatio;
      out = (out > 0.0f) ? compressed : -compressed;
    }
    
    // Normalize to prevent hard clipping
    if (out > 0.95f)  out = 0.95f + (out - 0.95f) * 0.1f;
    if (out < -0.95f) out = -0.95f + (out + 0.95f) * 0.1f;
    if (out > 1.0f)  out = 1.0f;
    if (out < -1.0f) out = -1.0f;
    
    driveTable[i] = out;
  }
  driveFX.shape(driveTable, 257);
}

// ---------- Pitch envelopes ----------

// Wild, Prok-style flexible env (for osc 0,1,2)
float computePitchFreq(const PitchEnv &p, float tMs) {
  // base:   10..200 Hz
  // attack: 0..200 ms
  // decay:  10..1000 ms
  // amount: 0..800 Hz
  // extend: extra downward sweep after main env

  float baseHz = 10.0f + p.base * 190.0f;
  float atkMs  = p.attack * 200.0f;
  float decMs  = 10.0f + p.decay * 990.0f;
  float amtHz  = p.amount * 800.0f;

  if (decMs < 1.0f) decMs = 1.0f;

  float env = 0.0f;
  if (tMs < atkMs && atkMs > 0.0f) {
    env = tMs / atkMs;  // 0->1
  } else {
    float td = tMs - atkMs;
    if (td < 0.0f) td = 0.0f;
    if (td < decMs) {
      env = 1.0f - (td / decMs);  // 1->0
    } else {
      env = 0.0f;
    }
  }

  float freq = baseHz + env * amtHz;

  if (p.extend > 0.001f) {
    float mainEnvEndMs = atkMs + decMs;
    float extTailMs = decMs * (2.0f + p.extend * 8.0f);
    float dropAmount = baseHz * p.extend * 0.8f;

    if (tMs > mainEnvEndMs) {
      float sweepTime = tMs - mainEnvEndMs;
      float sweepEnv = 1.0f - expf(-sweepTime / (extTailMs * 0.6f));
      if (sweepEnv > 1.0f) sweepEnv = 1.0f;
      float minFreq = baseHz - dropAmount;
      if (minFreq < 5.0f) minFreq = 5.0f;
      freq = baseHz - dropAmount * sweepEnv;
      if (freq < minFreq) freq = minFreq;
    }
  }

  if (freq < 5.0f) freq = 5.0f;
  return freq;
}

// 909-style core pitch envelope for Osc 3 (Tri2 core)
// Uses a much narrower range so it stays in that "compressed 909" zone.
float compute909CoreFreq(const PitchEnv &p, float tMs) {
  // Base: lock around classic 909-ish region
  // p.base ~0..1  =>  40..70 Hz
  float baseHz = 40.0f + p.base * 30.0f;

  // Very fast attack, short decay for "thump"
  // p.attack => 0..1  =>  0..5 ms
  float atkMs = p.attack * 5.0f;

  // p.decay => 0..1  =>  40..160 ms
  float decMs = 40.0f + p.decay * 120.0f;
  if (decMs < 1.0f) decMs = 1.0f;

  // Amount: how far above base we sweep at the very start
  // p.amount => 0..1  =>  0..80 Hz
  float amtHz = p.amount * 80.0f;

  // Simple attack/decay envelope
  float env = 0.0f;
  if (atkMs > 0.0f && tMs < atkMs) {
    env = tMs / atkMs;         // 0 -> 1
  } else {
    float td = tMs - atkMs;
    if (td < 0.0f) td = 0.0f;
    if (td < decMs) {
      env = 1.0f - (td / decMs);  // 1 -> 0
    } else {
      env = 0.0f;
    }
  }

  // Slightly square the envelope for more punch at the start
  env = env * env;

  float freq = baseHz + env * amtHz;
  if (freq < 20.0f) freq = 20.0f;   // keep it in bass range
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

  // Space-bar / GUI trigger for kick
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

  // OSC tone section
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

  // Per-oscillator pitch envelopes: 0B..3X
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

  // capture filter env at trigger
  filterEnvCapturedAtkMs = filtEnvAttackNorm * 80.0f;
  filterEnvCapturedDecMs = 5.0f + filtEnvDecayNorm * 2000.0f;

  envAmp.noteOn();
  envNoise.noteOn();
  envTransient.noteOn();
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

  AudioMemory(60);

  audioShield.enable();
  audioShield.volume(0.9f);   // as requested

  // Init oscillators
  oscBody.begin(WAVEFORM_SINE);
  oscBody.amplitude(1.0f);

  oscBody2.begin(WAVEFORM_SINE);
  oscBody2.amplitude(1.0f);

  oscTri1.begin(WAVEFORM_TRIANGLE);
  oscTri1.amplitude(1.0f);

  oscTri2.begin(WAVEFORM_TRIANGLE);   // now 909 core helper
  oscTri2.amplitude(1.0f);

  // Initial pitch env defaults
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

  oscEnv[3].base   = 0.60f;  // 909 core base
  oscEnv[3].attack = 0.0f;
  oscEnv[3].decay  = 0.3f;
  oscEnv[3].amount = 0.5f;
  oscEnv[3].extend = 0.3f;

  // AMP A env
  envAmp.attack(0.0f);
  envAmp.decay(200.0f);
  envAmp.sustain(0.0f);
  envAmp.release(0.0f);

  // Noise env
  envNoise.attack(0.0f);
  envNoise.decay(40.0f);
  envNoise.sustain(0.0f);
  envNoise.release(0.0f);

  // Transient env (very short tick)
  envTransient.attack(0.0f);
  envTransient.decay(30.0f);
  envTransient.sustain(0.0f);
  envTransient.release(0.0f);

  // Noise & filters
  noise.amplitude(noiseLevelNorm);
  noiseFilter.frequency(6000.0f);
  noiseFilter.resonance(2.0f);
  noiseFilter.octaveControl(1.0f);

  noiseTransient.amplitude(0.7f);
  transientFilter.frequency(8000.0f);
  transientFilter.resonance(0.7f);
  transientFilter.octaveControl(1.0f);

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

  // Kick mixer (body + transient)
  mixKick.gain(0, 1.0f);   // kick body
  mixKick.gain(1, transientLevelNorm * 0.5f);
  mixKick.gain(2, 0.0f);
  mixKick.gain(3, 0.0f);

  // OSC filter initial
  oscFilter.frequency(800.0f);
  oscFilter.resonance(1.0f);
  oscFilter.octaveControl(3.0f);

  // Final mix gains will be set each loop from masterGainSmooth
  mixFinal.gain(0, mixFinalBaseGains[0] * masterGainSmooth);
  mixFinal.gain(1, mixFinalBaseGains[1] * masterGainSmooth);
  mixFinal.gain(2, mixFinalBaseGains[2] * masterGainSmooth);
  mixFinal.gain(3, mixFinalBaseGains[3] * masterGainSmooth);

  // Initialize master drive curve
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
  float f0 = computePitchFreq(oscEnv[0], tMs);   // Sine A (wild)
  float f1 = computePitchFreq(oscEnv[1], tMs);   // Sine B (wild, detuned)
  float f2 = computePitchFreq(oscEnv[2], tMs);   // Tri A  (helper)
  float f3 = compute909CoreFreq(oscEnv[3], tMs); // Osc 3: 909-style core

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

  // Stop pitch env after a while (accounting for extended tails)
  float maxDec = 0.0f;
  for (int i = 0; i < 4; i++) {
    float dMs = 10.0f + oscEnv[i].decay * 990.0f;
    float extTailMs = dMs * (2.0f + oscEnv[i].extend * 8.0f);
    float eff = dMs + extTailMs;
    if (eff > maxDec) maxDec = eff;
  }
  if (pitchEnvActive && tMs > (maxDec + 200.0f)) {
    pitchEnvActive = false;
  }

  // AMP A envelope mapping
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

  // Noise engine mapping
  noise.amplitude(noiseLevelNorm * 0.3f);

  float noiseFreq = 120.0f + noiseCutoffNorm * 7000.0f;   // 120..7120 Hz
  noiseFilter.frequency(noiseFreq);

  float noiseQ = 0.7f + noiseResonanceNorm * 5.3f;        // 0.7..6.0
  noiseFilter.resonance(noiseQ);

  float nAttackMs = noiseAttackNorm * 60.0f;              // 0..60 ms
  float nDecayMs  = 5.0f + noiseDecayNorm * 500.0f;       // 5..505 ms

  envNoise.attack(nAttackMs);
  envNoise.decay(nDecayMs);
  envNoise.sustain(0.0f);
  envNoise.release(0.0f);

  // Transient mapping
  noiseTransient.amplitude(transientLevelNorm * 0.4f);

  float transientFreq = 2000.0f + transientCutoffNorm * 16000.0f;
  transientFilter.frequency(transientFreq);

  float tDecayMs = 5.0f + transientDecayNorm * 60.0f;     // 5..65 ms
  envTransient.attack(0.0f);
  envTransient.decay(tDecayMs);
  envTransient.sustain(0.0f);
  envTransient.release(0.0f);

  // Update kick mixer gains
  mixKick.gain(0, 1.0f);   // kick body always at 1.0
  mixKick.gain(1, transientLevelNorm * 0.5f);  // transient level

  // Filter envelope for osc filter
  float oscCutHzBase = 120.0f + oscCutoffNorm * 3880.0f;   // 120..4000 Hz
  float filtAtkMs = filterEnvActive ? filterEnvCapturedAtkMs : (filtEnvAttackNorm * 80.0f);
  float filtDecMs = filterEnvActive ? filterEnvCapturedDecMs : (5.0f + filtEnvDecayNorm * 2000.0f);
  float filterEnv = filterEnvActive ? computeSimpleAD(filterEnvTimer, filtAtkMs, filtDecMs) : 0.0f;
  float envSweepHz = filtEnvAmountNorm * 4000.0f * filterEnv;
  float oscCutHz = oscCutHzBase + envSweepHz;

  oscFilter.frequency(oscCutHz);

  if (filterEnvActive && filterEnv <= 0.0f && filterEnvTimer > (filtAtkMs + filtDecMs)) {
    filterEnvActive = false;
  }

  // Filter Q
  float oscQ = 0.7f + oscQNorm * 4.0f;   // tamed a bit vs 7.3f
  oscFilter.resonance(oscQ);

  // Smooth OSC distortion (to prevent clicks)
  float distSmoothRate = 0.03f;  // slower smoothing for distortion
  float distDiff = oscDistNorm - oscDistNormSmooth;
  oscDistNormSmooth += distDiff * distSmoothRate;
  
  // Osc distortion (oscDistNormSmooth) on oscDrive
  float drive = oscDistNormSmooth;
  static float oscShape[257];
  static float lastDrive = -1.0f;

  if (drive < 0.001f) {
    if (lastDrive >= 0.001f || lastDrive < 0.0f) {
      for (int i = 0; i < 257; i++) {
        float x = (float(i) - 128.0f) / 128.0f;
        oscShape[i] = x;
      }
      oscDrive.shape(oscShape, 257);
    }
    lastDrive = drive;
  } else {
    float d = 1.0f + drive * 6.0f;   // slightly tamer
    float norm = tanhf(d);
    if (norm < 0.0001f) norm = 0.0001f;

    for (int i = 0; i < 257; i++) {
      float x = (float(i) - 128.0f) / 128.0f;
      float y = tanhf(d * x) / norm;
      if (y > 1.0f)  y = 1.0f;
      if (y < -1.0f) y = -1.0f;
      oscShape[i] = y;
    }
    oscDrive.shape(oscShape, 257);
    lastDrive = drive;
  }

  // Update body mixer from level controls
  mixBody.gain(0, kickOsc1LevelNorm * 0.6f);
  mixBody.gain(1, kickOsc2LevelNorm * 0.6f);
  mixBody.gain(2, tri1LevelNorm     * 0.6f);
  mixBody.gain(3, tri2LevelNorm     * 0.6f);

  // Smooth master volume (increased smoothing to prevent clicks)
  float smoothRate = 0.03f;  // slower smoothing for smoother transitions
  float diff = masterGain - masterGainSmooth;
  masterGainSmooth += diff * smoothRate;

  // Update final mix gains using smoothed master
  for (int i = 0; i < 4; i++) {
    mixFinal.gain(i, mixFinalBaseGains[i] * masterGainSmooth);
  }
}

