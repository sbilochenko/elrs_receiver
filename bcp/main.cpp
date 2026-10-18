
#include <Arduino.h>
#include <math.h>

static constexpr int CRSF_RX_PIN = 16;
static constexpr int CRSF_TX_PIN = 17;
static constexpr uint32_t CRSF_BAUD = 420000;

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t OUTPUT_PERIOD_MS = 20; // 50 Hz
static constexpr uint32_t CHANNEL_TIMEOUT_MS = 150;
static constexpr float DEAD_ZONE = 0.05f;

// Калібрування з твого передавача
static constexpr int CH1_MIN = 174;
static constexpr int CH1_CENTER = 992;
static constexpr int CH1_MAX = 1811;

static constexpr int CH3_MIN = 191;
static constexpr int CH3_CENTER = 907;
static constexpr int CH3_MAX = 1811;

static uint16_t channels[16] = {};
static bool haveChannels = false;
static uint32_t lastChannelsMs = 0;
static uint32_t lastOutputMs = 0;

static uint8_t frame[64];
static size_t frameSize = 0;
static uint32_t crcErrors = 0;
static uint32_t validFrames = 0;

uint8_t crc8DvbS2(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80)
        ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
        : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool decodeChannels(const uint8_t *p, size_t len) {
  if (len != 22) return false;

  uint32_t bits = 0;
  int bitCount = 0;
  size_t pos = 0;

  for (int ch = 0; ch < 16; ++ch) {
    while (bitCount < 11) {
      bits |= static_cast<uint32_t>(p[pos++]) << bitCount;
      bitCount += 8;
    }

    channels[ch] = bits & 0x07FF;
    bits >>= 11;
    bitCount -= 11;
  }

  return true;
}

void processFrame(const uint8_t *f, size_t total) {
  const uint8_t length = f[1];
  const uint8_t type = f[2];
  const uint8_t expectedCrc = f[total - 1];
  const uint8_t actualCrc = crc8DvbS2(&f[2], length - 1);

  if (actualCrc != expectedCrc) {
    ++crcErrors;
    return;
  }

  // CRSF packed RC channels: type 0x16, payload 22 bytes
  if (type == 0x16 && length == 24) {
    if (decodeChannels(&f[3], 22)) {
      haveChannels = true;
      lastChannelsMs = millis();
      ++validFrames;
    }
  }
}

void readCrsf() {
  while (Serial2.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(Serial2.read());

    if (frameSize >= sizeof(frame)) {
      frameSize = 0;
    }
    frame[frameSize++] = b;

    while (frameSize >= 2) {
      const uint8_t length = frame[1];

      if (length < 2 || length > 62) {
        memmove(frame, frame + 1, --frameSize);
        continue;
      }

      const size_t total = static_cast<size_t>(length) + 2;
      if (frameSize < total) break;

      processFrame(frame, total);

      frameSize -= total;
      if (frameSize > 0) {
        memmove(frame, frame + total, frameSize);
      }
    }
  }
}

float normalizeChannel(int value, int minValue,
                       int centerValue, int maxValue) {
  float result;

  if (value >= centerValue) {
    const int range = maxValue - centerValue;
    result = range > 0
      ? static_cast<float>(value - centerValue) / range : 0.0f;
  } else {
    const int range = centerValue - minValue;
    result = range > 0
      ? -static_cast<float>(centerValue - value) / range : 0.0f;
  }

  return constrain(result, -1.0f, 1.0f);
}

float applyDeadZone(float value) {
  if (fabsf(value) < DEAD_ZONE) return 0.0f;

  // Компенсуємо dead zone, щоб решта діапазону знову була [-1, 1]
  const float adjusted =
      (fabsf(value) - DEAD_ZONE) / (1.0f - DEAD_ZONE);
  return copysignf(adjusted, value);
}

void setup() {
  Serial.begin(USB_BAUD);
  Serial2.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
}

void loop() {
  readCrsf();

  const uint32_t now = millis();
  if (now - lastOutputMs < OUTPUT_PERIOD_MS) return;
  lastOutputMs = now;

  const bool valid =
      haveChannels && (now - lastChannelsMs <= CHANNEL_TIMEOUT_MS);

  float linear = 0.0f;
  float angular = 0.0f;

  if (valid) {
    // CH3: вгору = додатний рух
    linear = applyDeadZone(
        normalizeChannel(channels[2], CH3_MIN, CH3_CENTER, CH3_MAX));

    // CH1: праворуч = додатне значення повороту
    angular = applyDeadZone(
        normalizeChannel(channels[0], CH1_MIN, CH1_CENTER, CH1_MAX));
  }

  // USB Serial містить лише машинозчитуваний протокол
  Serial.printf("RC,%.3f,%.3f,%d\n",
                linear, angular, valid ? 1 : 0);
}
