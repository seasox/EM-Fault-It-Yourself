#include <Arduino.h>
#include <stdint-gcc.h>

// These are defined when compiling with pio
#ifdef STM32L0
#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#endif

#ifdef STM32F4
#define SCK_PIN PB3
#define MISO_PIN PB5
#define MOSI_PIN PB4
#endif

auto clk_state = HIGH;

inline void wait_falling() {
    while (true) {
        const auto next = digitalRead(SCK_PIN);
        if (clk_state and not next) {
            clk_state = next;
            break;
        }
        clk_state = next;
    }
}

extern "C" {
    void _transfer(const uint8_t *ptr, uint32_t num_bytes) {
        for (uint32_t i = 0; i < num_bytes; i++) {
            const uint8_t value = ptr[i];
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 7) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 6) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 5) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 4) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 3) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 2) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, (value >> 1) & 0x01);
            wait_falling();
            digitalWrite(MISO_PIN, value & 0x01);
        }
    }
    void init_r0_to_r7(void);
    void send_r0_to_r7(void);
    void send_fault_window_start(void);
    void wait_fault(void);
    void send_fault_window_end(void);
    void send_end_sequence(void);
}

void setup() {
    pinMode(SCK_PIN, INPUT);
    pinMode(MOSI_PIN, INPUT);
    pinMode(MISO_PIN, OUTPUT);
    clk_state = HIGH;
}

void loop() {
    init_r0_to_r7();
    send_r0_to_r7();
    send_fault_window_start();
    wait_fault();
    send_fault_window_end();
    send_r0_to_r7();
    send_end_sequence();
}
