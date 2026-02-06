import { Mat, CV_8UC3, Rect, Point, Size, VideoWriter, imread, resize } from 'opencv';
import { readFileSync } from 'fs';
import { toString, getOpt } from 'util';

function main(...args) {
  let params = getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      '@': 'images.../c',
    },
    args,
  );

  console.log(params);

  const size = new Size(1080, 1440);
  const w = new VideoWriter(process.env.HOME + '/storage/movies/output.mp4', VideoWriter.fourcc('avc1'), 1, size);

  const frame = new Mat(size, CV_8UC3);

  for(const image of params['@']) {
    let m = imread(image);

    if(m.size.aspect != size.aspect) {
      let f = size.div(m.size);

      let ff = Math.min(...f);
      let ffi = [...f].indexOf(ff);
      let ffo = ffi ^ 1;
      let newsz = new Size(m.size).mul(ff).round();
      let dsz = new Size(size).sub(newsz);
      let r = new Rect(...new Size(dsz).div(2).round(), ...newsz);

      let diff = [...size][ffo] - [...newsz][ffo];

      //console.log('', { newsz, dsz, r, f, ff, ffi, diff });

      resize(m, m, newsz);
      frame.clear();
      m.copyTo(frame(r));

      m = frame;
    }

    if(!m.size.equals(size)) resize(m, m, size);

    console.log('frame size:', console.config({ compact: true }), m == frame, m.size);
    w.write(m);
  }

  console.log('writing...');
  m.release();
}

main(...scriptArgs.slice(1));
