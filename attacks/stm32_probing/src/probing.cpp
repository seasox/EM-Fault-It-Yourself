#include <Arduino.h>
#include <stdint-gcc.h>

// These are defined when compiling with pio
#ifdef STM32L0
#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#define NUM_ITERS "0x280000"
#endif

#ifdef STM32F4
#define SCK_PIN PB3
#define MISO_PIN PB5
#define MOSI_PIN PB4
#define NUM_ITERS "0xC80000"
#endif


asm(
    ".section .data\n"
    "num_iters:                 .word " NUM_ITERS "\n" // TODO change according to clk rate
    "register_value:            .word 0xaaaaaaaa\n"
    "end_seq:                   .byte 0x42, 0x42, 0x42, 0x42\n"
    "fault_window_start_seq:    .byte 42, 42, 42, 42\n"
    "fault_window_end_seq:      .byte 0x13, 0x37, 0x13, 0x37\n"
);

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
    /* IMPORTANT Host must wait a few cycles until _transfer is reached! */

    // Store values in the register
    "ldr r7, =register_value\n"
    "ldr r0, [r7]\n"
    "ldr r1, [r7]\n"
    "ldr r2, [r7]\n"
    "ldr r3, [r7]\n"
    "ldr r4, [r7]\n"
    "ldr r5, [r7]\n"
    "ldr r6, [r7]\n"
    "mov r7, #0\n"
    "push {r0-r7} \n"

    // send them to the host
    "mov r0, sp \n" // mov sp (location of the registers) to r0
    "mov r1, #32 \n" // r0 = begin of registers, r1 = size
    "bl _transfer\n"

    // Call transfer with fault_window_start_seq
    "ldr r0, =fault_window_start_seq\n"
    "mov r1, #4\n"
    "bl _transfer\n"

    // Load num iters to r7
    "ldr r7, =num_iters\n"
    "ldr r7, [r7]\n"
    "pop {r0-r6} \n"
    "add sp, sp, #4\n" // throw away r7

    // Faulting window
    "_loop: \n"
    "cmp r7, #0\n"
    "beq end_loop\n"
    "sub r7, r7, #1\n" // r7 <- r7 - 1
    "b _loop\n"
    "end_loop:\n"

    // we push the faulted data on the stack
    "push {r0-r7} \n"

    // Call transfer with fault_window_end_seq
    "ldr r0, =fault_window_end_seq\n"
    "mov r1, #4\n"
    "bl _transfer\n"

    /* IMPORTANT Host must wait a few cycles until _transfer is reached! */

    "mov r0, sp \n" // mov sp (location of the registers) to r0
    "mov r1, #32 \n" // r0 = begin of registers, r1 = size
    "bl _transfer\n"

    // pop the registers
    "pop {r0-r7}\n"

    // Call transfer with end_seq
    "ldr r0, =end_seq\n"
    "mov r1, #4\n"
    "bl _transfer\n"
    );
    }
}
