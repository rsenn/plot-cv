import * as cv from 'opencv';
#!/usr/bin/env qjsm
function main() {
  let image = new cv.Mat();
  let cam = new cv.RaspiCam();
  // cam.options->width=4056;
  // cam.options->height=3040;
  // cam.options->photo_width = 2028;
  // cam.options->photo_height = 1520;

  cam.options.photoWidth = 2592;
  cam.options.photoHeight = 1944;
  cam.options.verbose = false;
  cv.namedWindow('Image', cv.WINDOW_NORMAL);

  /*for(let i = 0; i < 100; i++)*/ {
    //console.log(i);

    if(cam.capturePhoto(image)) {
      console.log('Camera error');
    }
    //cv.imshow('Image', image);
    cv.imwrite('photo.png', image);
    console.log(`Saved as 'photo.png'.`);
    //cv.waitKey(30);
  }
  //cv.waitKey();
  cv.destroyWindow('Image');
}

main();