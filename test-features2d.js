import { tryCatch } from './lib/misc.js';
import Console from 'console';
import * as cv from 'opencv';
let basename = process.argv[1].replace(/\.js$/, '');

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

  let features2d_names = ctor_names.filter(name => cv[name].prototype && cv[name].prototype[Symbol.toStringTag] == 'Feature2D');

  console.log('cv', features2d_names);

  let detectors = {
    //affine: cv.AffineFeature,
    agast: cv.AgastFeatureDetector,
    akaze: cv.AKAZE,
    //brisk: cv.BRISK,
    fast: cv.FastFeatureDetector,
    gftt: cv.GFTTDetector,
    kaze: cv.KAZE,
    mser: cv.MSER,
    orb: cv.ORB,
    //sift: cv.SIFT,
    //simple_blob: cv.SimpleBlobDetector,
    //affine: cv.AffineFeature2D,
    boost: cv.BoostDesc,
    //  brief: cv.BriefDescriptorExtractor,
    //  daisy: cv.DAISY,
    freak: cv.FREAK,
    harris_laplace: cv.HarrisLaplaceFeatureDetector,
    /* latch: cv.LATCH,
    lucid: cv.LUCID,*/
    msd: cv.MSDDetector,
    star: cv.StarDetector,
    surf: cv.SURF,
    vgg: cv.VGG
  };
  let detectorNames = Object.keys(detectors);

  /*let instances=Object.entries(detectors).reduce((acc,[name,ctor]) => ({ ...acc, [name]: new ctor() }), {});
console.log("instances",instances);*/
  let img = cv.imread('Muehleberg.png');

  for(let name of detectorNames) {
    const CTOR = detectors[name];

    let kp = new cv.KeyPoint();
    console.log(`kp`, kp);
    console.log(`kp.hash()`, kp.hash());
    let f2d = new CTOR();
    console.log(`${name}`, f2d);
    console.log(`${name}.defaultName`, f2d.defaultName);
    console.log(`${name}.descriptorSize`, f2d.descriptorSize);
    const { CV_8U, CV_16U, CV_16S, CV_32F, CV_64F } = cv;
    console.log(`${name}.descriptorType`, f2d.descriptorType, {
      CV_8U,
      CV_16U,
      CV_16S,
      CV_32F,
      CV_64F
    });

    console.log(`${name}.write`, f2d.write(`features2d.xml`));
    console.log(`${name}.read`, f2d.read(`features2d.xml`));

    console.log('img', img);
    let keypoints, keypoints2, descriptors;

    tryCatch(
      () => f2d.compute(img, (keypoints = []), (descriptors = [])),
      r => r,
      e => console.log('ERROR', e.message)
    );

    tryCatch(
      () => f2d.detect(img, (keypoints2 = [])),
      r => r,
      e => console.log('ERROR', e.message)
    );

    console.log('keypoints', keypoints);
    console.log('keypoints2', keypoints2);
    console.log('descriptors', descriptors);
    console.log('f2d.defaultName', f2d.defaultName);

    let gray = new cv.Mat();

    cv.cvtColor(img, gray, cv.COLOR_BGR2GRAY);
    cv.cvtColor(gray, img, cv.COLOR_GRAY2BGR);
    console.log('img', img);

    cv.drawKeypoints(img, keypoints2, img, [255, 120, 0], cv.DRAW_RICH_KEYPOINTS);

    cv.imshow('img', img);

    cv.resizeWindow('img', img.cols / 4, img.rows / 4);
    cv.moveWindow('img', 0, 0);
    cv.waitKey(-1);
  }

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