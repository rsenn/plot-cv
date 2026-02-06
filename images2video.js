import { Mat, CV_8UC3, Rect, Point, Size, VideoWriter, imread, resize } from 'opencv';
import { readFileSync } from 'fs';
import { toString, getOpt } from 'util';

function main(...args) {
  let { fps=25, output = process.env.HOME + '/storage/movies/output.mp4', ...params } = getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      output: [true, null, 'o'],
      fps: [true, null, 'f'],
      '@': 'images.../c',
    },
    args,
  );

  const size = new Size(1080, 1440);
  const w = new VideoWriter(output, VideoWriter.fourcc('avc1'), fps, size);

  const frame = new Mat(size, CV_8UC3);
  const { length } = params['@'];
  
  let i = 0;

  for(const image of params['@']) {
    let m = imread(image);

    if(m.size.aspect != size.aspect) {
      let f = size.div(m.size);

      const factor = Math.min(...f);
      const index = [...f].indexOf(factor);
      const other = index ^ 1;
      const newsz = new Size(m.size).mul(factor).round();
      const delta = new Size(size).sub(newsz);
      const rect = new Rect(...new Size(delta).div(2).round(), ...newsz);

      const diff = [...size][other] - [...newsz][other];

      //console.log('', { newsz, delta, rect, f, factor, index, diff });

      resize(m, m, newsz);
      frame.clear();
      m.copyTo(frame(rect));

      m = frame;
    }

    if(!m.size.equals(size)) resize(m, m, size);

    console.log(`frame #${++i}/${length}`, console.config({ compact: true }), m == frame, m.size);
    w.write(m);
  }

  console.log(`writing '${output}'...`);
  w.release();
}

main(...scriptArgs.slice(1));
