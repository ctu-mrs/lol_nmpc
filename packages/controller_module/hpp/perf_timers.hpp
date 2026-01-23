#ifndef IO_STREAM
#define IO_STREAM
#include <iostream>
#endif
#ifndef CHRONO
#define CHRONO
#include <chrono>
#endif
#ifndef THREAD
#define THREAD
#include <thread>
#endif

struct MrsTimer {
  std::chrono::high_resolution_clock::time_point start, end;
  std::chrono::duration<float> duration;
  std::string print_string;
  MrsTimer(std::string input_string) {
    start = std::chrono::high_resolution_clock::now();
    print_string = input_string;
  }
  ~MrsTimer() {
    end = std::chrono::high_resolution_clock::now();
    duration = end - start;

    float ms = duration.count() * 1e3;
    float us = duration.count() * 1e6;
    std::cout << print_string << " : " << ms << " ms or " << us << " us"
              << "\n";
  }
};