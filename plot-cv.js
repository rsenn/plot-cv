//
function process(contours, hier, ...args) {
  //let size = new Size(imgOriginal.cols, imgOriginal.rows);
  const [images, ...rest] = args;
  const { imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector } = images;

  let m = imgOriginal;
  console.log('Images: ', typeof images);
  console.log('Images: ',  Object.keys(images));
  console.log('rest.length: ', rest.length);
  console.log('rest: ', ...rest);

  console.log(`Video resolution: ${m.cols}x${m.rows}`);
  imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector;

  console.log('Num contours: ', contours.length);
}
