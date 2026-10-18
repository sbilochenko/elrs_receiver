
#include <Arduino.h>

static constexpr int CRSF_RX_PIN = 16;
static constexpr int CRSF_TX_PIN = 17;
static constexpr uint32_t CRSF_BAUD = 420000;

void setup() {
    Serial.begin(115200);

    Serial2.begin(
        CRSF_BAUD,
        SERIAL_8N1,
        CRSF_RX_PIN,
        CRSF_TX_PIN
    );

    Serial.println("CRSF UART2 ready");
}

void loop() {
    while (Serial2.available() > 0) {
        const int value = Serial2.read();

        Serial.printf("%02X ", value);

        static uint16_t byte_count = 0;
        if (++byte_count % 32 == 0) {
            Serial.println();
        }
    }

    delay(1);
}
