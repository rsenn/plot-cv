//
function process(contours, hier, images) {
  //let size = new Size(imgOriginal.cols, imgOriginal.rows);
  const { imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector } = images;

  let m = imgOriginal;
  //console.log('Video: ',Util.className(m));

  console.log(`Video resolution: ${m.cols}x${m.rows}`);
  imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector;

  console.log('Num contours: ', contours.length);
}
