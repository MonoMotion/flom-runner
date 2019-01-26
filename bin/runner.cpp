#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <thread>

#include <flom/flom.hpp>
#include <servoarray/servoarray.h>
#include <servoarray/servomap.h>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " INPUT" << std::endl;
    return -1;
  }

  std::ifstream f(argv[1], std::ios::binary);
  auto const motion = flom::Motion::load(f);

  auto array = ServoArray::ServoArray();
  auto servos = ServoArray::ServoMap(array);

  for (auto const& [t, frame] : motion.frames(0.01)) {
    for (auto const& [name, pos] : frame.positions()) {
      servos.write(name, pos);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

