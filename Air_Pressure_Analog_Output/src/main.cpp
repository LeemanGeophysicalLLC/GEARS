#include <Adafruit_AD569x.h>
#include <Adafruit_BMP3XX.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t BMP390_ADDRESS_PRIMARY = 0x77;
constexpr uint8_t BMP390_ADDRESS_ALTERNATE = 0x76;
constexpr uint8_t AD5693_ADDRESS = 0x4C;

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long MEASUREMENT_INTERVAL_MS = 100;

constexpr uint16_t DEFAULT_ZERO_VOLT_DECI_HPA = 9300;
constexpr uint16_t DEFAULT_FULL_SCALE_DECI_HPA = 1010 * 10;
constexpr uint8_t DEFAULT_AVERAGE_POINTS = 8;

constexpr uint16_t MIN_CONFIG_DECI_HPA = 8000;
constexpr uint16_t MAX_CONFIG_DECI_HPA = 11000;
constexpr uint8_t MIN_AVERAGE_POINTS = 1;
constexpr uint8_t MAX_AVERAGE_POINTS = 128;

constexpr uint32_t SETTINGS_MAGIC = 0x41414F31UL;  // "AAO1"
constexpr float DAC_FULL_SCALE_VOLTS = 5.0f;
constexpr uint16_t DAC_MAX_CODE = 65535;

struct Settings {
  uint32_t magic;
  uint16_t zeroVoltDeciHpa;
  uint16_t fullScaleDeciHpa;
  uint8_t averagePoints;
  uint8_t reserved;
};

Settings settings;
uint32_t samples[MAX_AVERAGE_POINTS];
uint8_t sampleCount = 0;
uint8_t sampleIndex = 0;
uint32_t sampleSum = 0;

uint32_t lastRawPressurePa = 0;
uint32_t lastAveragePressurePa = 0;
uint16_t lastDacCode = 0;
bool hasValidReading = false;

char commandBuffer[48];
size_t commandLength = 0;
unsigned long lastMeasurementAtMs = 0;

Adafruit_BMP3XX bmp;
Adafruit_AD569x dac;
bool bmpReady = false;
bool dacReady = false;
uint8_t activeBmp390Address = 0;

void updateOutputFromPressure(uint32_t pressurePa);

Settings defaultSettings() {
  Settings defaults{};
  defaults.magic = SETTINGS_MAGIC;
  defaults.zeroVoltDeciHpa = DEFAULT_ZERO_VOLT_DECI_HPA;
  defaults.fullScaleDeciHpa = DEFAULT_FULL_SCALE_DECI_HPA;
  defaults.averagePoints = DEFAULT_AVERAGE_POINTS;
  defaults.reserved = 0;
  return defaults;
}

bool isValidSettings(const Settings &candidate) {
  return candidate.magic == SETTINGS_MAGIC &&
         candidate.zeroVoltDeciHpa >= MIN_CONFIG_DECI_HPA &&
         candidate.zeroVoltDeciHpa <= MAX_CONFIG_DECI_HPA &&
         candidate.fullScaleDeciHpa >= MIN_CONFIG_DECI_HPA &&
         candidate.fullScaleDeciHpa <= MAX_CONFIG_DECI_HPA &&
         candidate.averagePoints >= MIN_AVERAGE_POINTS &&
         candidate.averagePoints <= MAX_AVERAGE_POINTS;
}

void persistSettings() {
  settings.magic = SETTINGS_MAGIC;
  EEPROM.put(0, settings);
}

void loadSettings() {
  Settings stored{};
  EEPROM.get(0, stored);
  if (isValidSettings(stored)) {
    settings = stored;
  } else {
    settings = defaultSettings();
    persistSettings();
  }
}

void clearSamples() {
  sampleCount = 0;
  sampleIndex = 0;
  sampleSum = 0;
  memset(samples, 0, sizeof(samples));
  hasValidReading = false;
  lastRawPressurePa = 0;
  lastAveragePressurePa = 0;
}

void seedSamples(uint32_t pressurePa) {
  clearSamples();
  samples[0] = pressurePa;
  sampleCount = 1;
  sampleIndex = 1 % settings.averagePoints;
  sampleSum = pressurePa;
  lastRawPressurePa = pressurePa;
  lastAveragePressurePa = pressurePa;
  hasValidReading = true;
}

float pressurePaToHpa(uint32_t pressurePa) {
  return static_cast<float>(pressurePa) / 100.0f;
}

float currentVoltage() {
  return (static_cast<float>(lastDacCode) * DAC_FULL_SCALE_VOLTS) /
         static_cast<float>(DAC_MAX_CODE);
}

void writeDacCode(uint16_t value) {
  if (value > DAC_MAX_CODE) {
    value = DAC_MAX_CODE;
  }
  if (!dacReady) {
    lastDacCode = value;
    return;
  }

  if (!dac.writeUpdateDAC(value)) {
    dacReady = false;
    Serial.println(F("DAC write failed."));
  }
  lastDacCode = value;
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  HELP"));
  Serial.println(F("  SHOW"));
  Serial.println(F("  SETMIN <hPa>   Pressure for 0.0 V"));
  Serial.println(F("  SETMAX <hPa>   Pressure for 5.0 V"));
  Serial.println(F("  SETAVG <n>     Moving average points"));
  Serial.println(F("  DEFAULTS"));
  Serial.println(F("  SAVE"));
  Serial.println(F("Limits:"));
  Serial.println(F("  Configurable pressure: 800.0 to 1100.0 hPa"));
  Serial.println(F("  Moving average points: 1 to 128"));
  Serial.println(F("Notes:"));
  Serial.println(F("  Settings apply immediately and are saved automatically."));
  Serial.println(F("  Reversed mapping is allowed by setting MIN above MAX."));
}

void printConfiguration() {
  Serial.println(F("Configuration:"));
  Serial.print(F("  0.0 V pressure: "));
  Serial.print(settings.zeroVoltDeciHpa / 10.0f, 1);
  Serial.println(F(" hPa"));
  Serial.print(F("  5.0 V pressure: "));
  Serial.print(settings.fullScaleDeciHpa / 10.0f, 1);
  Serial.println(F(" hPa"));
  Serial.print(F("  Average points: "));
  Serial.println(settings.averagePoints);

  Serial.print(F("Devices:"));
  Serial.print(F(" BMP390="));
  if (bmpReady) {
    Serial.print(F("OK(0x"));
    if (activeBmp390Address < 0x10) {
      Serial.print('0');
    }
    Serial.print(activeBmp390Address, HEX);
    Serial.print(')');
  } else {
    Serial.print(F("ERROR"));
  }
  Serial.print(F(" AD5693="));
  Serial.println(dacReady ? F("OK") : F("ERROR"));

  if (hasValidReading) {
    Serial.print(F("Current:"));
    Serial.print(F(" raw="));
    Serial.print(pressurePaToHpa(lastRawPressurePa), 2);
    Serial.print(F(" hPa"));
    Serial.print(F(" avg="));
    Serial.print(pressurePaToHpa(lastAveragePressurePa), 2);
    Serial.print(F(" hPa"));
    Serial.print(F(" dac="));
    Serial.print(lastDacCode);
    Serial.print(F("/65535"));
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

bool parseDeciHpaValue(const char *text, uint16_t *valueOut) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  const unsigned long wholePart = strtoul(text, &endPtr, 10);
  if (endPtr == text || wholePart > 6553UL) {
    return false;
  }

  uint8_t tenthPart = 0;
  if (*endPtr == '.') {
    ++endPtr;
    if (!isdigit(static_cast<unsigned char>(*endPtr))) {
      return false;
    }

    tenthPart = static_cast<uint8_t>(*endPtr - '0');
    ++endPtr;

    while (*endPtr == '0') {
      ++endPtr;
    }
  }

  if (*endPtr != '\0') {
    return false;
  }

  const unsigned long deciHpa = wholePart * 10UL + tenthPart;
  if (deciHpa > 65535UL) {
    return false;
  }

  *valueOut = static_cast<uint16_t>(deciHpa);
  return true;
}

void applyAveragePointChange(uint8_t newAveragePoints) {
  const bool hadReading = hasValidReading;
  const uint32_t retainedPressurePa = lastAveragePressurePa;

  settings.averagePoints = newAveragePoints;
  if (hadReading) {
    seedSamples(retainedPressurePa);
  } else {
    clearSamples();
  }
  persistSettings();
}

void resetToDefaults() {
  const bool hadReading = hasValidReading;
  const uint32_t retainedPressurePa = lastAveragePressurePa;

  settings = defaultSettings();
  if (hadReading) {
    seedSamples(retainedPressurePa);
  } else {
    clearSamples();
  }
  persistSettings();
  if (hasValidReading) {
    updateOutputFromPressure(lastAveragePressurePa);
  } else {
    writeDacCode(0);
  }
}

float computeNormalizedPressure(uint32_t pressurePa) {
  const float zero = static_cast<float>(settings.zeroVoltDeciHpa) * 10.0f;
  const float full = static_cast<float>(settings.fullScaleDeciHpa) * 10.0f;

  if (settings.zeroVoltDeciHpa == settings.fullScaleDeciHpa) {
    return pressurePa >= static_cast<uint32_t>(full) ? 1.0f : 0.0f;
  }

  const float normalized =
      (static_cast<float>(pressurePa) - zero) / (full - zero);
  if (normalized < 0.0f) {
    return 0.0f;
  }
  if (normalized > 1.0f) {
    return 1.0f;
  }
  return normalized;
}

void updateOutputFromPressure(uint32_t pressurePa) {
  const float normalized = computeNormalizedPressure(pressurePa);
  const uint16_t dacCode =
      static_cast<uint16_t>(normalized * static_cast<float>(DAC_MAX_CODE) +
                            0.5f);
  writeDacCode(dacCode);
}

void addSample(uint32_t pressurePa) {
  const uint8_t windowSize = settings.averagePoints;

  if (sampleCount < windowSize) {
    samples[sampleIndex] = pressurePa;
    sampleSum += pressurePa;
    sampleCount++;
    sampleIndex = (sampleIndex + 1) % windowSize;
  } else {
    sampleSum -= samples[sampleIndex];
    samples[sampleIndex] = pressurePa;
    sampleSum += pressurePa;
    sampleIndex = (sampleIndex + 1) % windowSize;
  }

  lastRawPressurePa = pressurePa;
  lastAveragePressurePa = sampleSum / sampleCount;
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
    if (!parseDeciHpaValue(valueText, &value) || value < MIN_CONFIG_DECI_HPA ||
        value > MAX_CONFIG_DECI_HPA) {
      Serial.println(F("Invalid SETMIN value. Use: SETMIN <hPa>"));
      return;
    }

    settings.zeroVoltDeciHpa = value;
    persistSettings();
    if (hasValidReading) {
      updateOutputFromPressure(lastAveragePressurePa);
    } else {
      writeDacCode(0);
    }
    Serial.println(F("0.0 V pressure updated and saved."));
    printConfiguration();
    return;
  }

  if (strcmp(line, "SETMAX") == 0) {
    uint16_t value = 0;
    if (!parseDeciHpaValue(valueText, &value) || value < MIN_CONFIG_DECI_HPA ||
        value > MAX_CONFIG_DECI_HPA) {
      Serial.println(F("Invalid SETMAX value. Use: SETMAX <hPa>"));
      return;
    }

    settings.fullScaleDeciHpa = value;
    persistSettings();
    if (hasValidReading) {
      updateOutputFromPressure(lastAveragePressurePa);
    } else {
      writeDacCode(0);
    }
    Serial.println(F("5.0 V pressure updated and saved."));
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
      updateOutputFromPressure(lastAveragePressurePa);
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
  if (!bmpReady) {
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastMeasurementAtMs < MEASUREMENT_INTERVAL_MS) {
    return;
  }
  lastMeasurementAtMs = nowMs;

  if (!bmp.performReading()) {
    Serial.println(F("Failed to read BMP390."));
    return;
  }

  const float pressure = bmp.pressure;
  if (pressure <= 0.0f) {
    return;
  }

  const uint32_t pressurePa = static_cast<uint32_t>(pressure + 0.5f);
  addSample(pressurePa);
  updateOutputFromPressure(lastAveragePressurePa);
}

void initializeBmp390() {
  bmpReady = bmp.begin_I2C(BMP390_ADDRESS_PRIMARY, &Wire);
  if (bmpReady) {
    activeBmp390Address = BMP390_ADDRESS_PRIMARY;
  } else {
    bmpReady = bmp.begin_I2C(BMP390_ADDRESS_ALTERNATE, &Wire);
    if (bmpReady) {
      activeBmp390Address = BMP390_ADDRESS_ALTERNATE;
    }
  }

  if (!bmpReady) {
    activeBmp390Address = 0;
    Serial.println(F("Could not find BMP390 at 0x77 or 0x76."));
    return;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void initializeDac() {
  dacReady = dac.begin(AD5693_ADDRESS, &Wire);
  if (!dacReady) {
    Serial.println(F("Could not find AD5693 at 0x4C."));
    return;
  }

  if (!dac.setMode(NORMAL_MODE, true, true)) {
    dacReady = false;
    Serial.println(F("Failed to configure AD5693 for 0-5 V output."));
  }
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();

  loadSettings();
  clearSamples();

  delay(250);
  initializeBmp390();
  initializeDac();
  writeDacCode(0);

  Serial.println();
  Serial.println(F("Air Pressure Analog Output ready."));
  printConfiguration();
  printHelp();
}

void loop() {
  readSerialCommands();
  performMeasurementIfDue();
}
