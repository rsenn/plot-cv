import { Mat, cvtColor, VideoCapture, norm, COLOR_BGR2GRAY, imwrite } from 'opencv';
import { Grayscale, getSimilarity, Dup, mse } from './cv-helpers.js';
import { Window } from './qjs-opencv/js/cvHighGUI.js';
import { VideoSource } from './qjs-opencv/js/cvVideo.js';
import { roundTo } from 'util';

function nextFrame(cap) {
  let m = new Mat();
  cap.read(m);
  //cvtColor(m, m, COLOR_BGR2GRAY);
  return m.empty ? null : m;
}

function similarity(a, b) {
  let ac = new Mat(),
    bc = new Mat();

  cvtColor(a, ac, COLOR_BGR2GRAY);
  cvtColor(b, bc, COLOR_BGR2GRAY);

  //console.log('ac.channels', ac.channels);

  return roundTo(mse(ac, bc), 1e-5);
}

function main(file) {
  const vc = new VideoSource(file);
  const w = new Window(scriptArgs[0]);

  w.move(320, 180);

  let prev, frame;
  let i = 0,
    running = true;

  w.onkey = ({ key, keycode, scancode, mods }) => {
    switch (key) {
      case 's':
      case 'S': {
        const name = file.replace(/\.[^.]*$/g, '').replace(/.*\//g, '') + `-frame-${vc.props.pos_frames}.png`;
        imwrite(name, frame);

        console.log('Saved ' + name);
        break;
      }

      case 'q':
      case 'Q': {
        running = false;
        break;
      }

      case '\uff51':
        vc.seekMsecs(-1000);
        break;
      case '\uff53':
        vc.seekMsecs(1000);
        break;

      default: {
        console.log('key', { key, keycode, scancode, mods });
        break;
      }
    }
  };

  while(running && (frame = nextFrame(vc))) {
    if(prev) {
      let sim = similarity(prev, frame);

      w.show(frame);
      w.update();

      console.log(`Frame #${vc.props.pos_frames} similarity:`, sim);
    }

    prev = Dup(frame);
    ++i;
  }
}

main(...scriptArgs.slice(1));
