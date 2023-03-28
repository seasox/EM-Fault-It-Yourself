#include <Arduino.h>
#include <stdint-gcc.h>

#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11

auto clk_state = HIGH;
const uint8_t end_sequence[8] = {0b10000000, 0, 0, 0, 0, 0, 0, 1};


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
}

void setup() {
    pinMode(SCK_PIN, INPUT);
    pinMode(MOSI_PIN, INPUT);
    pinMode(MISO_PIN, OUTPUT);
    clk_state = HIGH;
}


void loop() {
while (1){

    asm volatile (
            "mov r0, #0\n"
            "mov r1, #1\n"
            "mov r2, #2\n"
            "mov r3, #3\n"
            "mov r4, #4\n"
            "mov r5, #5\n"
            "mov r6, #6\n"
            "mov r7, #7\n");

    asm volatile("push {r0-r7} \n"
                 "mov r0, sp \n" // mov sp (location of the registers) to r0
                 "mov r1, #32 \n" // now, r0 = begin of registers, r1 = size
                 "bl _transfer\n"
                 "pop {r0-r7}\n");

    _transfer(end_sequence, 8);
    }
}
