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

#include <args.hxx>

static std::atomic<bool> quit(false);

void register_signal(int);
void quit_handler(int);

int main(int argc, char *argv[]) {
  args::ArgumentParser argparser("Run flom motion file on real robot");
  args::HelpFlag help(argparser, "help", "Print this help", {'h', "help"});
  args::Positional<std::string> arg_motion(argparser, "motion", "motion file");
  args::ValueFlag<double> arg_fps(argparser, "fps", "fps", {'f', "fps"});

  try {
    argparser.ParseCLI(argc, argv);
  } catch (args::Help){
    std::cout << argparser;
    return 0;
  } catch (args::ParseError e){
    std::cerr << e.what() << std::endl;
    std::cerr << argparser;
    return -1;
  }

  if(!arg_motion) {
    std::cerr << "Specify the motion file to play" << std::endl;
    std::cerr << argparser;
    return -1;
  }

  register_signal(SIGINT);
  register_signal(SIGQUIT);
  register_signal(SIGTERM);

  std::ifstream f(args::get(arg_motion), std::ios::binary);
  auto const motion = flom::Motion::load(f);

  double const fps = arg_fps ? args::get(arg_fps) : 0.01;

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
