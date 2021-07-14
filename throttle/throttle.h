#include <Adafruit_MCP4725.h>

// Macros for "Arduino effect" - Ross
#define D2 2

// Run mode can either be calibration or regular operation
bool calibration {false};

// Flag to catch exceptions
bool exception {false};

// Circuit Parameters
/*
Seemed more organized to put the parameters in the global space near the top. 
Passing a bunch of parameters during contruction is great when you have a config file. 
When the code is your config file, might as well make things global
*/
bool invertReverse {true};

float throttleDeadbandRadiusRatio {4.0 / 1024.0};

struct FwdThrottleInput {
  int inputPin {A1};
  void setup() {}// called during the arduino setup
  float get() { // float is between 0  and 1
    float ratio = (float) analogRead(inputPin) / 1023.0;
    return (ratio > throttleDeadbandRadiusRatio)?(ratio):(0.0);
  }
} fwdThrottleInput;

struct RevThrottleInput {
  int inputPin {A2};
  void setup() {}// called during the arduino setup
  float get() { // float is between 0  and 1
    float ratio = (float) analogRead(inputPin) / 1023.0;
    return (ratio > throttleDeadbandRadiusRatio)?(ratio):(0.0);
  }
} revThrottleInput;

struct RegenInput {
  int inputPin {A3};
  void setup() {}// called during the arduino setup
  float get() { // float is between 0  and 1
    return (float) analogRead(inputPin) / 1023.0;
  }
} regenInput;

/// Throttle Output
struct ThrottleOutput {
  int dacI2CAddr {0x60};
  Adafruit_MCP4725 dac;

  void setup() { dac.begin(dacI2CAddr); }// called during the arduino setup
  void set(float inputRatio) { // float is between 0  and 1
    dac.setVoltage(inputRatio*4095.0, false);
  }
} throttleOutput;

struct ReverseSignalOutput {
  int outputPin {D2};

  void setup() { pinMode(outputPin, OUTPUT); }// called during the arduino setup
  void set(bool reverseCmd) { // float is between 0  and 1
    bool correctedReverseCmd = (invertReverse)?(!reverseCmd):(reverseCmd);
    digitalWrite(outputPin, (correctedReverseCmd)?(HIGH):(LOW));
  }
} reverseSignalOutput;


struct RegenOutput {
  int dacI2CAddr {0x61};
  Adafruit_MCP4725 dac;

  void setup() { dac.begin(dacI2CAddr); }// called during the arduino setup
  void set(float inputRatio) { // float is between 0  and 1
    dac.setVoltage(inputRatio*4095.0, false);
  }
} regenOutput;

// Conversion Logic

struct FwdThrottleScaleConverter {  
  const float inputMinRatio = throttleDeadbandRadiusRatio; // Give just a hair of deadband to account for noise
  const float inputMaxRatio = 0.5; // Determine this empirically
  const float outputMinScaleRatio = 0.5 / 5.0; // motor controller starting threshold
  const bool logarthimic {true}; // Use a logarthmic curve from input to output ratios (greater sensitivity at lower speeds)
  FwdThrottleScaleConverter() {
      if (inputMinRatio >= inputMaxRatio) {
      exception = true;
    }
  }

  float convert(float inputRatio) {
    // Deadband zone. Most Likely.
    if (inputRatio <= inputMinRatio) {
      return 0;
    }
  
    // Beyond full scale, peg to full output
    if (inputRatio >= inputMaxRatio) {
      return 1.0;
    }

    // Rescale input to account for center deadband gap
    inputRatio = (inputRatio - inputMinRatio) / (inputMaxRatio - inputMinRatio);

    // Linear
    if (!logarthimic) {
      return inputRatio;
    }

    // Log scale approximation
    return -inputRatio / (2.0*inputRatio - 3.0); 
  }
} fwdThrottleScaleConverter;

struct RevThrottleScaleConverter {  
  const float inputMinRatio = throttleDeadbandRadiusRatio; // Give just a hair of deadband to account for noise
  const float inputMaxRatio = 0.5; // Determine this empirically
  const float outputMinScaleRatio = 0.5 / 5.0; // motor controller starting threshold
  const bool logarthimic {true}; // Use a logarthmic curve from input to output ratios (greater sensitivity at lower speeds)
  RevThrottleScaleConverter() {
      if (inputMinRatio >= inputMaxRatio) {
      exception = true;
    }
  }

  float convert(float inputRatio) {
    // Deadband zone. Most Likely.
    if (inputRatio <= inputMinRatio) {
      return 0;
    }
  
    // Beyond full scale, peg to full output
    if (inputRatio >= inputMaxRatio) {
      return 1.0;
    }

    // Rescale input to account for center deadband gap
    inputRatio = (inputRatio - inputMinRatio) / (inputMaxRatio - inputMinRatio);

    // Linear
    if (!logarthimic) {
      return inputRatio;
    }

    // Log scale approximation
    return -inputRatio / (2.0*inputRatio - 3.0); 
  }
} revThrottleScaleConverter;

struct RegenScaleConverter {
  const float inputMinRatio = 0.1; // Give just a hair of deadband to account for noise
  const float inputMaxRatio = 0.2; // Determine this empirically
  const float outputMinScaleRatio = 0.5 / 5.0; // motor controller starting threshold
  const bool logarthimic {true}; // Use a logarthmic curve from input to output ratios (greater sensitivity at lower speeds)
  RegenScaleConverter() {
      if (inputMinRatio >= inputMaxRatio) {
      exception = true;
    }
  }

  float convert(float inputRatio) {
    // Regen Off. Most Likely.
    if (inputRatio <= inputMinRatio) {
      return 0;
    }
  
    // Beyond full scale, peg to full output
    if (inputRatio >= inputMaxRatio) {
      return 1.0;
    }

    // Rescale input to account for limited potentiometer travel
    inputRatio = (inputRatio - inputMinRatio) / (inputMaxRatio - inputMinRatio);
    
    // Linear
    if (!logarthimic) {
      return inputRatio;
    }

    // Log scale approximation
    return -inputRatio / (2.0*inputRatio - 3.0); 
  }
} regenScaleConverter;

void setup() {
  fwdThrottleInput.setup();
  revThrottleInput.setup();
  regenInput.setup();
  throttleOutput.setup();
  reverseSignalOutput.setup();
  regenOutput.setup();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);           //  setup serial
}

void loop() {
  if (exception) {
    // Blink twice a second to indicate exception
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    return;
  }

  // Get all inputs
  float fwdThrottleInputRatio = fwdThrottleInput.get();
  float revThrottleInputRatio = revThrottleInput.get();
  float regenInputRatio = regenInput.get();
  if (calibration) {
    Serial.print("Fwd In: ");
    Serial.print(fwdThrottleInputRatio);
    
    Serial.print(" Rev In: ");
    Serial.print(revThrottleInputRatio);

    Serial.print(" Regen In: ");
    Serial.print(regenInputRatio);

    Serial.print(" Fwd Out: ");
    Serial.print(fwdThrottleScaleConverter.convert(fwdThrottleInputRatio));
    
    Serial.print(" Rev Out: ");
    Serial.print(revThrottleScaleConverter.convert(revThrottleInputRatio));
    
    Serial.print(" Regen Out: ");
    Serial.print(regenScaleConverter.convert(regenInputRatio));
    
    Serial.println();
    delay(250);
    return;
  }

  // Check for faults
  if (fwdThrottleInputRatio && revThrottleInputRatio) {
    exception = true; // got signal from both potentiometer sides. either improper wiring or corroded pot.
    return;
  }

  // Regen comes next to hit the brakes just that little bit quicker
  if (regenInputRatio) {
    throttleOutput.set(0.0);
    reverseSignalOutput.set(false);
    regenOutput.set( regenScaleConverter.convert(regenInputRatio) );
    return;
  }

  // Fwd Throttle is the next most likely state
  if (fwdThrottleInputRatio) {
    regenOutput.set(0.0);
    reverseSignalOutput.set(false);
    throttleOutput.set(fwdThrottleScaleConverter.convert(fwdThrottleInputRatio));
    return;
  }

  // Last is the Rev Throttle
  regenOutput.set(0.0);
  reverseSignalOutput.set(true);
  throttleOutput.set(revThrottleScaleConverter.convert(revThrottleInputRatio));
}