import * as cv from 'opencv';

function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: 3,
    maxArrayLength: 30,
    compact: 3
  });

  cv.namedWindow('test', 1200, 600);

  for(let arg of args) {
    let key,
      img = cv.imread(arg);
    console.log('img', img);

    cv.cvtColor(img, img, cv.COLOR_BGR2GRAY);

    cv.imshow('test', img);
    cv.resizeWindow('test', img.rows, img.cols);
    do {
      key = cv.waitKey(-1);
    } while(key != 13);
  }
}

main(...scriptArgs.slice(1));
