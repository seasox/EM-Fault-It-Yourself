#include <Arduino.h>
#include <stdint-gcc.h>

#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
//#define FAULTING_WINDOW_SEC 2

auto clk_state = HIGH;

const uint8_t end_seq[4] = {0x42, 0x42, 0x42, 0x42};
const uint8_t fault_window_start_seq[4] = {42, 42, 42, 42};
const uint8_t fault_window_end_seq[4] = {0x13, 0x37, 0x13, 0x37};

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

uint32_t _load_num_iters(){
    // the loop has 4 instructions, we want to wait
    // TODO make depending on frequency
    return 8000000;
}

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

    /* IMPORTANT Host must wait a few cycles until _transfer is reached! */
    // Device sends register after reset as sanity check
    asm volatile (
    "mov r0, #0\n"
    "mov r1, #1\n"
    "mov r2, #2\n"
    "mov r3, #3\n"
    "mov r4, #4\n"
    "mov r5, #5\n"
    "mov r6, #6\n"
    "mov r7, #0\n"
    "push {r0-r7} \n"
    );

    asm volatile(
    "mov r0, sp \n" // mov sp (location of the registers) to r0
    "mov r1, #32 \n" // now, r0 = begin of registers, r1 = size
    "bl _transfer\n"
    );

    // we send a magic constant that indicates that the device is ready to be faulted
    _transfer(fault_window_start_seq, 4);

    // wait some time. The fault will happen during this wait.
    asm volatile(
    "bl _load_num_iters\n"
    "mov r7, r0\n" // return value is in r0, we want to use r7 as cnt
    "pop {r0-r6} \n"
    "add sp, sp, #4\n" // throw away r7
    "_loop: \n"
    "cmp r7, #0\n"
    "beq end_loop\n"
    "sub r7, r7, #1\n" // r7 <- r7 - 1
    "b _loop\n"
    "end_loop:\n"
    "push {r0-r7} \n" // we push the faulted data on the stack
    );

    _transfer(fault_window_end_seq, 4); // we send a magic constant that indicates that the fault window ended

    /* IMPORTANT Host must wait a few cycles until _transfer is reached! */
    asm volatile(
    "mov r0, sp \n" // mov sp (location of the registers) to r0
    "mov r1, #32 \n" // now, r0 = begin of registers, r1 = size
    "bl _transfer\n"
    "pop {r0-r7}\n"
    );

    _transfer(end_seq, 4);
    }
}
