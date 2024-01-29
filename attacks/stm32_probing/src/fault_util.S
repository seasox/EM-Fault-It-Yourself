.global send_r0_to_r7
.global init_r0_to_r7
.global send_end_sequence
.global send_fault_window_start
.global send_fault_window_end
.global wait_fault


.section .data
    num_iters:               .word 0xC80000 // (1 secs * 168 MHz)/(4 [cycles per iteration]) = 4.2 * 10**7 (# of loop iterations)
    register_value:          .word 0xaaaaaaaa
    end_seq:                 .byte 0x42, 0x42, 0x42, 0x42
    fault_window_start_seq:  .byte 42, 42, 42, 42
    fault_window_end_seq:    .byte 0x13, 0x37, 0x13, 0x37


.section .text

init_r0_to_r7:
    push {lr}
    ldr r7, =register_value
    ldr r0, [r7]
    ldr r1, [r7]
    ldr r2, [r7]
    ldr r3, [r7]
    ldr r4, [r7]
    ldr r5, [r7]
    ldr r6, [r7]
    mov r7, #0
    pop {pc}

send_r0_to_r7:
    push {lr}
    push {r0-r7} // send the values of r0-r7
    mov r0, sp // mov sp (location of the registers) to r0
    mov r1, #32 // r0 = begin of registers, r1 = 8 x 4 bytes
    bl _transfer
    pop {r0-r7}
    pop {pc}

send_end_sequence:
    push {r0-r7, lr}
    ldr r0, =end_seq
    mov r1, #4
    bl _transfer
    pop {r0-r7, pc}

send_fault_window_start:
    push {r0-r7, lr}
    ldr r0, =fault_window_start_seq
    mov r1, #4
    bl _transfer
    pop {r0-r7, pc}

send_fault_window_end:
    push {r0-r7, lr}
    ldr r0, =fault_window_end_seq
    mov r1, #4
    bl _transfer
    pop {r0-r7, pc}

wait_fault:
    push {r7, lr}
    ldr r7, =num_iters
    ldr r7, [r7]
    _loop:
        cmp r7, #0
        beq end_loop
        sub r7, r7, #1  // r7 <- r7 - 1
        b _loop
    end_loop:
        pop {r7, pc}
