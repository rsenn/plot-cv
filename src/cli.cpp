#include "plot-cv.h"
#include "color.h"
#include "js.h"
#include "jsbindings.h"
#include "line.h"
#include "geometry.h"
#include "matrix.h"

#include "js.h"

std::ofstream logfile("clilog", std::ios_base::out | std::ios_base::ate);

extern "C" {

int
main(int argc, char* argv[]) {
  int i;
  js_init(argc, argv);

  if(argc < 2) {
    std::cerr << "Usage: " << basename(argv[0]) << " <script.js>" << std::endl;
    return 1;
  }

  for(i = 1; i < argc; i++) {
    int ret;

    std::cerr << "eval file: " << argv[i] << std::endl;

    ret = js.eval_file(argv[i]);
    std::cerr << "ret: " << ret << std::endl;
  }

  return 0;
}
}
