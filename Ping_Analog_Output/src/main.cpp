#include <Arduino.h>
#include <FlashStorage.h>
#include <Wire.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t TRIGGER_PIN = 5;
constexpr uint8_t ECHO_PIN = 6;
constexpr uint8_t MCP4725_ADDRESS = 0x62;

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long MEASUREMENT_INTERVAL_MS = 50;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

constexpr uint16_t DEFAULT_ZERO_VOLT_MM = 10;
constexpr uint16_t DEFAULT_FULL_SCALE_MM = 100;
constexpr uint8_t DEFAULT_AVERAGE_POINTS = 8;

constexpr uint16_t MIN_CONFIG_MM = 1;
constexpr uint16_t MAX_CONFIG_MM = 4000;
constexpr uint8_t MIN_AVERAGE_POINTS = 1;
constexpr uint8_t MAX_AVERAGE_POINTS = 128;

constexpr uint32_t SETTINGS_MAGIC = 0x50414F31UL;  // "PAO1"

struct Settings {
  uint32_t magic;
  uint16_t zeroVoltMm;
  uint16_t fullScaleMm;
  uint8_t averagePoints;
  uint8_t reserved;
};

FlashStorage(settingsStore, Settings);

Settings settings;
uint16_t samples[MAX_AVERAGE_POINTS];
uint8_t sampleCount = 0;
uint8_t sampleIndex = 0;
uint32_t sampleSum = 0;

uint16_t lastRawDistanceMm = 0;
uint16_t lastAverageDistanceMm = 0;
uint16_t lastDacCode = 0;
bool hasValidReading = false;

char commandBuffer[48];
size_t commandLength = 0;
unsigned long lastMeasurementAtMs = 0;

void updateOutputFromDistance(uint16_t distanceMm);

Settings defaultSettings() {
  Settings defaults{};
  defaults.magic = SETTINGS_MAGIC;
  defaults.zeroVoltMm = DEFAULT_ZERO_VOLT_MM;
  defaults.fullScaleMm = DEFAULT_FULL_SCALE_MM;
  defaults.averagePoints = DEFAULT_AVERAGE_POINTS;
  defaults.reserved = 0;
  return defaults;
}

bool isValidSettings(const Settings &candidate) {
  return candidate.magic == SETTINGS_MAGIC &&
         candidate.zeroVoltMm >= MIN_CONFIG_MM &&
         candidate.zeroVoltMm <= MAX_CONFIG_MM &&
         candidate.fullScaleMm >= MIN_CONFIG_MM &&
         candidate.fullScaleMm <= MAX_CONFIG_MM &&
         candidate.averagePoints >= MIN_AVERAGE_POINTS &&
         candidate.averagePoints <= MAX_AVERAGE_POINTS;
}

void clearSamples() {
  sampleCount = 0;
  sampleIndex = 0;
  sampleSum = 0;
  memset(samples, 0, sizeof(samples));
  hasValidReading = false;
  lastRawDistanceMm = 0;
  lastAverageDistanceMm = 0;
}

void seedSamples(uint16_t distanceMm) {
  clearSamples();
  samples[0] = distanceMm;
  sampleCount = 1;
  sampleIndex = 1 % settings.averagePoints;
  sampleSum = distanceMm;
  lastRawDistanceMm = distanceMm;
  lastAverageDistanceMm = distanceMm;
  hasValidReading = true;
}

void persistSettings() {
  settings.magic = SETTINGS_MAGIC;
  settingsStore.write(settings);
}

void loadSettings() {
  const Settings stored = settingsStore.read();
  if (isValidSettings(stored)) {
    settings = stored;
  } else {
    settings = defaultSettings();
    persistSettings();
  }
}

void writeDacCode(uint16_t value) {
  value = min<uint16_t>(value, 4095);

  Wire.beginTransmission(MCP4725_ADDRESS);
  Wire.write(0x40);
  Wire.write(static_cast<uint8_t>(value >> 4));
  Wire.write(static_cast<uint8_t>((value & 0x0F) << 4));
  Wire.endTransmission();

  lastDacCode = value;
}

float currentVoltage() {
  return (static_cast<float>(lastDacCode) * 3.3f) / 4095.0f;
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  HELP"));
  Serial.println(F("  SHOW"));
  Serial.println(F("  SETMIN <mm>    Distance for 0.0 V"));
  Serial.println(F("  SETMAX <mm>    Distance for 3.3 V"));
  Serial.println(F("  SETAVG <n>     Moving average points"));
  Serial.println(F("  DEFAULTS"));
  Serial.println(F("  SAVE"));
  Serial.println(F("Limits:"));
  Serial.println(F("  Configurable distance: 1 to 4000 mm"));
  Serial.println(F("  Sensor nominal range: about 20 to 4000 mm"));
  Serial.println(F("  Moving average points: 1 to 128"));
  Serial.println(F("Notes:"));
  Serial.println(F("  Settings apply immediately and are saved automatically."));
  Serial.println(F("  Reversed mapping is allowed by setting MIN above MAX."));
}

void printConfiguration() {
  Serial.println(F("Configuration:"));
  Serial.print(F("  0.0 V distance: "));
  Serial.print(settings.zeroVoltMm);
  Serial.println(F(" mm"));
  Serial.print(F("  3.3 V distance: "));
  Serial.print(settings.fullScaleMm);
  Serial.println(F(" mm"));
  Serial.print(F("  Average points: "));
  Serial.println(settings.averagePoints);

  if (hasValidReading) {
    Serial.print(F("Current:"));
    Serial.print(F(" raw="));
    Serial.print(lastRawDistanceMm);
    Serial.print(F(" mm"));
    Serial.print(F(" avg="));
    Serial.print(lastAverageDistanceMm);
    Serial.print(F(" mm"));
    Serial.print(F(" dac="));
    Serial.print(lastDacCode);
    Serial.print(F("/4095"));
    Serial.print(F(" voltage="));
    Serial.print(currentVoltage(), 3);
    Serial.println(F(" V"));
  } else {
    Serial.println(F("Current: no valid reading yet"));
  }
}

bool parseUint16Value(const char *text, uint16_t *valueOut) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  const unsigned long parsed = strtoul(text, &endPtr, 10);
  if (*endPtr != '\0' || parsed > 65535UL) {
    return false;
  }

  *valueOut = static_cast<uint16_t>(parsed);
  return true;
}

bool parseUint8Value(const char *text, uint8_t *valueOut) {
  uint16_t parsed = 0;
  if (!parseUint16Value(text, &parsed) || parsed > 255) {
    return false;
  }

  *valueOut = static_cast<uint8_t>(parsed);
  return true;
}

void applyAveragePointChange(uint8_t newAveragePoints) {
  const bool hadReading = hasValidReading;
  const uint16_t retainedDistanceMm = lastAverageDistanceMm;

  settings.averagePoints = newAveragePoints;
  if (hadReading) {
    seedSamples(retainedDistanceMm);
  } else {
    clearSamples();
  }
  persistSettings();
}

void resetToDefaults() {
  const bool hadReading = hasValidReading;
  const uint16_t retainedDistanceMm = lastAverageDistanceMm;

  settings = defaultSettings();
  if (hadReading) {
    seedSamples(retainedDistanceMm);
  } else {
    clearSamples();
  }
  persistSettings();
  if (hasValidReading) {
    updateOutputFromDistance(lastAverageDistanceMm);
  } else {
    writeDacCode(0);
  }
}

float computeNormalizedDistance(uint16_t distanceMm) {
  const float zero = static_cast<float>(settings.zeroVoltMm);
  const float full = static_cast<float>(settings.fullScaleMm);

  if (settings.zeroVoltMm == settings.fullScaleMm) {
    return distanceMm >= settings.fullScaleMm ? 1.0f : 0.0f;
  }

  const float normalized =
      (static_cast<float>(distanceMm) - zero) / (full - zero);
  if (normalized < 0.0f) {
    return 0.0f;
  }
  if (normalized > 1.0f) {
    return 1.0f;
  }
  return normalized;
}

void updateOutputFromDistance(uint16_t distanceMm) {
  const float normalized = computeNormalizedDistance(distanceMm);
  const uint16_t dacCode = static_cast<uint16_t>(normalized * 4095.0f + 0.5f);
  writeDacCode(dacCode);
}

uint16_t measureDistanceMm() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  const unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (durationUs == 0) {
    return 0;
  }

  const unsigned long distanceMm = (durationUs * 343UL) / 2000UL;
  if (distanceMm > 65535UL) {
    return 0;
  }

  return static_cast<uint16_t>(distanceMm);
}

void addSample(uint16_t distanceMm) {
  const uint8_t windowSize = settings.averagePoints;

  if (sampleCount < windowSize) {
    samples[sampleIndex] = distanceMm;
    sampleSum += distanceMm;
    sampleCount++;
    sampleIndex = (sampleIndex + 1) % windowSize;
  } else {
    sampleSum -= samples[sampleIndex];
    samples[sampleIndex] = distanceMm;
    sampleSum += distanceMm;
    sampleIndex = (sampleIndex + 1) % windowSize;
  }

  lastRawDistanceMm = distanceMm;
  lastAverageDistanceMm = static_cast<uint16_t>(sampleSum / sampleCount);
  hasValidReading = true;
}

void processCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }

  size_t length = strlen(line);
  while (length > 0 &&
         (line[length - 1] == '\r' || line[length - 1] == '\n' ||
          line[length - 1] == ' ' || line[length - 1] == '\t')) {
    line[length - 1] = '\0';
    --length;
  }

  if (*line == '\0') {
    return;
  }

  for (char *p = line; *p != '\0'; ++p) {
    *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  }

  char *valueText = nullptr;
  char *separator = strchr(line, ' ');
  if (separator != nullptr) {
    *separator = '\0';
    valueText = separator + 1;
    while (*valueText == ' ' || *valueText == '\t') {
      ++valueText;
    }
    if (*valueText == '\0') {
      valueText = nullptr;
    }
  }

  if (strcmp(line, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcmp(line, "SHOW") == 0) {
    printConfiguration();
    return;
  }

  if (strcmp(line, "DEFAULTS") == 0) {
    resetToDefaults();
    Serial.println(F("Defaults restored and saved."));
    printConfiguration();
    return;
  }

  if (strcmp(line, "SAVE") == 0) {
    persistSettings();
    Serial.println(F("Settings saved. Changes are normally saved automatically."));
    return;
  }

  if (strcmp(line, "SETMIN") == 0) {
    uint16_t value = 0;
    if (!parseUint16Value(valueText, &value) || value < MIN_CONFIG_MM ||
        value > MAX_CONFIG_MM) {
      Serial.println(F("Invalid SETMIN value. Use: SETMIN <mm>"));
      return;
    }

    settings.zeroVoltMm = value;
    persistSettings();
    if (hasValidReading) {
      updateOutputFromDistance(lastAverageDistanceMm);
    } else {
      writeDacCode(0);
    }
    Serial.println(F("0.0 V distance updated and saved."));
    printConfiguration();
    return;
  }

  if (strcmp(line, "SETMAX") == 0) {
    uint16_t value = 0;
    if (!parseUint16Value(valueText, &value) || value < MIN_CONFIG_MM ||
        value > MAX_CONFIG_MM) {
      Serial.println(F("Invalid SETMAX value. Use: SETMAX <mm>"));
      return;
    }

    settings.fullScaleMm = value;
    persistSettings();
    if (hasValidReading) {
      updateOutputFromDistance(lastAverageDistanceMm);
    } else {
      writeDacCode(0);
    }
    Serial.println(F("3.3 V distance updated and saved."));
    printConfiguration();
    return;
  }

  if (strcmp(line, "SETAVG") == 0) {
    uint8_t value = 0;
    if (!parseUint8Value(valueText, &value) || value < MIN_AVERAGE_POINTS ||
        value > MAX_AVERAGE_POINTS) {
      Serial.println(F("Invalid SETAVG value. Use: SETAVG <n>"));
      return;
    }

    applyAveragePointChange(value);
    if (hasValidReading) {
      updateOutputFromDistance(lastAverageDistanceMm);
    } else {
      writeDacCode(0);
    }
    Serial.println(F("Average point count updated and saved."));
    printConfiguration();
    return;
  }

  Serial.println(F("Unknown command. Type HELP."));
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      Serial.println(F("Command too long. Type HELP."));
    }
  }
}

void performMeasurementIfDue() {
  const unsigned long nowMs = millis();
  if (nowMs - lastMeasurementAtMs < MEASUREMENT_INTERVAL_MS) {
    return;
  }
  lastMeasurementAtMs = nowMs;

  const uint16_t distanceMm = measureDistanceMm();
  if (distanceMm == 0) {
    return;
  }

  addSample(distanceMm);
  updateOutputFromDistance(lastAverageDistanceMm);
}

}  // namespace

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(SERIAL_BAUD);
  Wire.begin();

  loadSettings();
  clearSamples();
  writeDacCode(0);

  delay(250);
  Serial.println();
  Serial.println(F("Ping Analog Output ready."));
  printConfiguration();
  printHelp();
}

void loop() {
  readSerialCommands();
  performMeasurementIfDue();
}
