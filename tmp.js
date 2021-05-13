console.log(`Trackbar 'frame' frameShow=${frameShow} pipeline.size - 1 = ${
    pipeline.size - 1
  }`
);
function getBitDepth(mat) {
  switch (mat.depth) {
    case cv.CV_8U:
    case cv.CV_8S:
      return 8;
    case cv.CV_16U:
    case cv.CV_16S:
      return 16;
    case cv.CV_32F:
      return 32;
    case cv.CV_64F:
      return 64;
  }
}
