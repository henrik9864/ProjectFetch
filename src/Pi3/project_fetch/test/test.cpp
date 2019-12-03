#include <pigpio.h>
#include <iostream>

int main() {
  std::cout << "Test" << std::endl;
  gpioSetMode(10, PI_OUTPUT);
  gpioWrite(10, 255);
  return 0;
}
