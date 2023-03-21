#include <Arduino.h>
#include <SPI.h>
#include <stddef.h>


#define get_reg(reg_name, i) do {   \
    asm("mov\t%0, " reg_name : "=r"(i));\
} while(0)

void setup() {
  // initialize SPI:
  SPI.begin();
}


void loop() {
    int i;
    get_reg("sp", i);
    SPI.transfer(i);


}
