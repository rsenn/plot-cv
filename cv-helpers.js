import { Mat, cvtColor, split, COLOR_BGR2Lab, NORM_L2, norm, CV_32F, sum, pow } from 'opencv';

export function Grayscale(src, dst) {
  const channels = [];
  let ret = !dst;
  cvtColor(src, (dst ??= new Mat()), COLOR_BGR2Lab);
  split(dst, channels);

  if(ret) return channels[0];

  channels[0].copyTo(dst);
}

export function Dup(src) {
  let dst = new Mat(src.size, src.type);
  src.copyTo(dst);
  return dst;
}

export function Convert(src, type, alpha = 1, beta = 0) {
  let dst = new Mat(src.size, type);
  src.convertTo(dst, type, alpha, beta);
  return dst;
}

// Compare two images by getting the L2 error (square-root of sum of squared error).
export function getSimilarity(A, B) {
  if(A.rows > 0 && A.rows == B.rows && A.cols > 0 && A.cols == B.cols) {
    // Calculate the L2 relative error between images.
    const errorL2 = norm(A, B, NORM_L2);
    // Convert to a reasonable scale, since L2 error is summed across all pixels of the image.
    const similarity = errorL2 / (A.rows * A.cols);
    return similarity;
  } else {
    //Images have a different size
    return 100000000.0; // Return a bad value
  }
}

export function mse(imageA, imageB) {
  // the 'Mean Squared Error' between the two images is the
  // sum of the squared difference between the two images;
  // NOTE: the two images must have the same dimension
  const diff = new Mat();

  Convert(imageA, CV_32F, 1 / 255).sub(Convert(imageB, CV_32F, 1 / 255), diff);

  const p = new Mat();

  pow(diff, 2, p);

  let [err] = sum(p);

  err /= imageA.rows * imageA.cols;

  // return the MSE, the lower the error, the more "similar"
  // the two images are
  return err;
}
