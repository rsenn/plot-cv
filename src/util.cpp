#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include "util.hpp"

static size_t heap_base = 0;

void*
get_heap_base() {
  if(!heap_base) {
    std::ifstream infile("/proc/self/maps");
    infile >> std::hex;
    for(std::string line; std::getline(infile, line);) {
      if(line.find("[heap]") != std::string::npos) {
        size_t start = line.find("-");
        heap_base = strtoull(line.c_str() + start + 1, nullptr, 16);
      }
    }
    infile >> heap_base;
    std::cerr << "heap_base: " << heap_base << std::endl;
  }
  return reinterpret_cast<void*>(heap_base);
}
