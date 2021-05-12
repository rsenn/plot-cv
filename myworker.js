/* Worker code for test_worker.js */
import * as std from 'std';
import * as os from 'os';
import * as cv from 'opencv';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Console } from 'console';
import { Pipeline, Processor } from './cvPipeline.js';
import { TickMeter } from 'opencv';

var parent = os.Worker.parent;

var configFile = 'test-opencv.config.json';

function SaveConfig(configObj) {
  configObj = Object.fromEntries(Object.entries(configObj).map(([k, v]) => [k, +v])
  );
  let file = std.open(configFile, 'w+b');
  file.puts(JSON.stringify(configObj, null, 2) + '\n');
  file.close();
}

function LoadConfig() {
  let str = std.loadFile(configFile);
  let configObj = JSON.parse(str ?? '{}');

  configObj = Object.fromEntries(Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:', configObj);
  return configObj;
}

function HandleMessage(e) {
  var ev = e.data;
  console.log('worker HandleMessage', ev);
  switch (ev.type) {
    case 'abort':
      parent.postMessage({ type: 'done' });
      break;
    case 'sab':
      /* modify the SharedArrayBuffer */
      ev.buf[2] = 10;
      parent.postMessage({ type: 'sab_done', buf: ev.buf });
      break;
  }
}

function worker_main() {
  globalThis.console = new Console({
    colors: true,
    stringBreakNewline: true,
    maxStringLength: Infinity,
    compact: 2
  });

  console.log('worker worker_main', parent);
  print('worker worker_main');
  /* let { frameShow = 1, ...config } = LoadConfig();

  let paramNav = new ParamNavigator({
      thres: new NumericParam(config.thres ?? 8, 0, 255),
      max: new NumericParam(config.max ?? 255, 0, 255),
      type: new NumericParam(config.type ?? cv.THRESH_BINARY_INV, 0, 4),
      kernel_size: new NumericParam(config.kernel_size ?? 1, 0, 10),
      k: new NumericParam(config.k ?? 24, 0, 100),
      thres1: new NumericParam(config.thres1 ?? 10, 0, 300),
      thres2: new NumericParam(config.thres2 ?? 20, 0, 300),
      thres2: new NumericParam(config.thres2 ?? 20, 0, 300),
      rho: new NumericParam(config.rho ?? 1, 1, 100),
      theta: new NumericParam(config.theta ?? 180, 1, 360),
      threshold: new NumericParam(config.threshold ?? 10, 0, 100),
      minLineLength: new NumericParam(config.minLineLength ?? 2, 0, 1000),
      maxLineGap: new NumericParam(config.maxLineGap ?? 4, 0, 1000),
      dp: new NumericParam(config.dp ?? 2, 0.1, 100),
      minDist: new NumericParam(config.minDist ?? 10, 1, 1000),
      param1: new NumericParam(config.param1 ?? 200, 1, 1000),
      param2: new NumericParam(config.param2 ?? 100, 1, 1000)
    },
    config.currentParam
  );
*/
  var i;

  parent.onmessage = HandleMessage;

  for(;;) {
    for(i = 0; i < 10; i++) {
      parent.postMessage({ type: 'num', num: i });
    }
    cv.waitKey(3000);
  }
}

worker_main();
