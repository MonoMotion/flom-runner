#include <fstream>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <string>
#include <memory>
#include <chrono>
#include <cstdint>

#include <flom/flom.hpp>
#include <ics-servo/servo.h>

std::unordered_map<std::string, unsigned> joint_index = {
  {"hip_joint_right_r", 4},
  {"hip_joint_right_p", 3},
  {"knee_right", 2},
  {"ankle_right_p", 1},
  {"ankle_right_r", 0},
  {"hip_joint_left_r", 9},
  {"hip_joint_left_p", 8},
  {"knee_left", 7},
  {"ankle_left_p", 6},
  {"ankle_left_r", 5},
};

int main(int argc, char *argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " DEVICE EN_PIN INPUT" << std::endl;
    return -1;
  }

  const std::string device {argv[1]};
  const std::uint8_t en_pin {static_cast<std::uint8_t>(strtol(argv[2], nullptr, 0))};

  auto io = std::make_shared<ICSServo::IOProvider>(device, en_pin);

  std::unordered_map<std::string, ICSServo::Servo> servos;
  for (auto const& [name, idx] : joint_index) {
    servos.emplace(name, ICSServo::Servo(io, idx));
  }

  std::ifstream f(argv[3], std::ios::binary);
  auto const motion = flom::Motion::load(f);

  auto const start = std::chrono::system_clock::now();
  while(true) {
    auto const now = std::chrono::system_clock::now();
    auto const t = now - start;
    auto const seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t);

    auto const frame = motion.frame_at(seconds.count());
    for (auto const& [name, pos] : frame.positions()) {
      auto servo = servos.at(name);
      servo.set_position(pos);
    }
  }
}

