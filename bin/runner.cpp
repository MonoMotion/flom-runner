#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <signal.h>

#include <flom/flom.hpp>
#include <servoarray/servoarray.h>
#include <servoarray/servomap.h>

static std::atomic<bool> quit(false);

void register_signal(int);
void quit_handler(int);

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " INPUT [FPS=0.01]" << std::endl;
    return -1;
  }

  register_signal(SIGINT);
  register_signal(SIGQUIT);
  register_signal(SIGTERM);

  std::ifstream f(argv[1], std::ios::binary);
  auto const motion = flom::Motion::load(f);

  double const fps = argc > 2 ? std::atof(argv[2]) : 0.01;

  auto array = ServoArray::ServoArray();
  auto servos = ServoArray::ServoMap(array);

  for (auto const& [t, frame] : motion.frames(fps)) {
    for (auto const& [name, pos] : frame.positions()) {
      servos.write(name, pos);
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(fps));

    if(quit.load()) break;
  }
}

void register_signal(int signal) {
  struct sigaction action{};
  action.sa_handler = quit_handler;
  sigfillset(&action.sa_mask);
  sigaction(signal, &action, nullptr);
}

void quit_handler(int) {
  quit.store(true);
}
