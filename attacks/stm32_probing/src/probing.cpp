#include <Arduino.h>
#include <stdint-gcc.h>

#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#define FAULTING_WINDOW_SEC 2

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
    //return 160000000 / 4;
    // the loop has 4 instructions, we want to waurt
    // TODO make depending on frequency
    return 16000;
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

    // we send a magic constant that indicates that the device is ready to be faulted
    _transfer(fault_window_start_seq, 4);

    // the expected values for the registers (successful r7 fault is unknown as it is used as the counter)
    asm volatile (
    "mov r0, #0\n"
    "mov r1, #1\n"
    "mov r2, #2\n"
    "mov r3, #3\n"
    "mov r4, #4\n"
    "mov r5, #5\n"
    "mov r6, #6\n"
    "mov r7, #0\n"
    );

    // wait some time
    asm volatile(
    "bl _load_num_iters\n"
    "mov r7, r0\n" // return value is in r0, we want to use r7 as cnt
    "mov r0, #0\n" // we store the expected value in r0
    "_loop: \n"
    "cmp r7, #0\n"
    "beq end_loop\n"
    "sub r7, r7, #1\n" // r7 <- r7 - 1
    "b _loop\n"
    "end_loop:\n"
    "push {r0-r7} \n" // we push the faulted data on the stack
    );

    // ATTENTION! You have to wait a few cycles until the DUT branches to _transfer and is ready to transfer its register
    _transfer(fault_window_end_seq, 4); // we send a magic constant that indicates that the fault window ended

    asm volatile(
    "mov r0, sp \n" // mov sp (location of the registers) to r0
    "mov r1, #32 \n" // now, r0 = begin of registers, r1 = size
    "bl _transfer\n"
    "pop {r0-r7}\n"
    );

    _transfer(end_seq, 4);
    }
}
