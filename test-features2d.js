import * as cv from 'opencv';
import Console from 'console';
import * as path from 'path';
import Util from './lib/util.js';

let basename = Util.getArgv()[1].replace(/\.js$/, '');

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: 100,
      compact: 1,
      depth: 10
    }
  });
  let ctor_names = Object.getOwnPropertyNames(cv).filter(name => typeof cv[name] == 'function');

  let features2d_names = ctor_names.filter(name => cv[name].prototype && cv[name].prototype[Symbol.toStringTag] == 'Feature2D'
  );

  console.log('cv', features2d_names);

  let kp = new cv.KeyPoint();
  console.log('kp', kp);
  console.log('kp.hash()', kp.hash());
  let f2d = new cv.FastFeatureDetector();
  console.log('f2d', f2d);
  console.log('f2d.defaultName', f2d.defaultName);
  console.log('f2d.descriptorSize', f2d.descriptorSize);
  const { CV_8U, CV_16U,CV_16S, CV_32F, CV_64F} = cv;
  console.log('f2d.descriptorType', f2d.descriptorType, { CV_8U, CV_16U,CV_16S,CV_32F,CV_64F});



  console.log('f2d.write', f2d.write("features2d.xml"));
  console.log('f2d.read', f2d.read("features2d.xml"));

  let img = cv.imread('Best_of_Italo_Disco_Vol._6.png');

  console.log('img', img);
  let keypoints;
  try {
    f2d.compute(img, (keypoints = []));
  } catch(e) {
    console.log('ERROR', e.message);
  }

  f2d.detect(img, (keypoints = []));

  console.log('keypoints', keypoints);
  console.log('f2d.defaultName', f2d.defaultName);

  console.log('EXIT');
}
try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
