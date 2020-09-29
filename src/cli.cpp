#include "plot-cv.h"
#include "color.h"
#include "js.h"
#include "jsbindings.h"
#include "line.h"
#include "geometry.h"
#include "matrix.h"

#include "js.h"
#include "../quickjs/quickjs-libc.h"

#include <libgen.h>

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

  jsrt::value promise = js.get_global("Promise");
  jsrt::value promise_proto = js.get_property(promise, "prototype");

  std::cerr << "Promise is_function " << js.is_constructor(promise) << std::endl;
  std::cerr << "Promise type=" << js.typestr(promise) << std::endl;
  std::cerr << "Promise.prototype type=" << js.typestr(promise_proto) << std::endl;
  for(i = 1; i < argc; i++) {
    jsrt::value ret;

    std::cerr << "eval file: " << argv[i] << std::endl;

    ret = js.eval_file(argv[i]);

    js_std_loop(js.ctx);

    std::cerr << "ret is_promise " << js.is_promise(ret) << std::endl;

    std::cerr << "ret type=" << js.typestr(ret) << std::endl;
  }

  return 0;
}
}
