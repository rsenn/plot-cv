import * as cv from 'cv';
import { Mat } from 'mat';
import { drawCircle, drawContour, drawLine, drawPolygon, drawRect } from 'draw';
import { VideoCapture } from 'video-capture';
import Util from './lib/util.js';

export class ImageSequence {
  constructor(images = []) {
    const imgs = this;
    this.images = images;
    this.frame = null;
    this.index = 0;
    this.props = {
      get frame_count() {
        return imgs.images.length;
      },
      fps: 1,
      backend: 'imread',
      get pos_frames() {
        return imgs.index;
      },
      set pos_frames(value) {
        imgs.index = Util.mod(value, images.length);
      },
      get pos_msec() {
        const { pos_frames, fps } = this;
        return (pos_frames * 1000) / fps;
      },
      get frame_width() {
        if(imgs.frame) return imgs.frame.cols;
      },
      get frame_height() {
        if(imgs.frame) return imgs.frame.rows;
      }
    };
  }

  getBackendName() {
    return 'imread';
  }
  isOpened() {
    return true;
  }
  get(prop) {
    return this.props[prop.toLowerCase()];
  }
  set(prop, value) {
    this.props[prop.toLowerCase()] = value;
  }
  grab() {
    const { images, props } = this;
    return !!(this.frame = cv.imread(images[props.pos_frames++]));
  }
  retrieve(mat) {
    if(!mat) return this.frame;
    if(this.frame) this.frame.copyTo(mat);
    return !!this.frame;
  }
  read(mat) {
    if(this.grab()) return this.retrieve(mat);
  }
}

const isVideoPath = arg => /\.(3gp|avi|f4v|flv|m4v|m2v|mkv|mov|mp4|mpeg|mpg|ogm|vob|webm|wmv)$/i.test(arg);

export class VideoSource {
  static backends = Object.fromEntries(['ANY', 'VFW', 'V4L', 'V4L2', 'FIREWIRE', 'FIREWARE', 'IEEE1394', 'DC1394', 'CMU1394', 'QT', 'UNICAP', 'DSHOW', 'PVAPI', 'OPENNI', 'OPENNI_ASUS', 'ANDROID', 'XIAPI', 'AVFOUNDATION', 'GIGANETIX', 'MSMF', 'WINRT', 'INTELPERC', 'REALSENSE', 'OPENNI2', 'OPENNI2_ASUS', 'GPHOTO2', 'GSTREAMER', 'FFMPEG', 'IMAGES', 'ARAVIS', 'OPENCV_MJPEG', 'INTEL_MFX', 'XINE'].map(name => [name, cv['CAP_' + name]])
  );

  constructor(...args) {
    if(args.length > 0) {
      let [device, backend = 'ANY'] = args;

      if(typeof device == 'string' && isVideoPath(device)) if (backend == 'ANY') backend = 'FFMPEG';

      const driverId = VideoSource.backends[backend];
      console.log('VideoSource', { device, backend, driverId, args });

      if(typeof driverId == 'number') {
        this.capture(device, driverId);
      } else {
        this.fromImages(...args);
      }
    }
  }

  capture(device, driverId) {
    let cap = new VideoCapture(device, driverId);
    this.cap = cap;

    this.propId = prop => {
      if(typeof prop == 'string') {
        prop = prop.toUpperCase();
        if(!prop.startsWith('CAP_PROP_')) prop = 'CAP_PROP_' + prop;
        prop = cv[prop];
      }
      return prop;
    };

    this.read = function(mat) {
      const { cap } = this;
      if(!mat) mat = new Mat();
      if(cap.read(mat)) return mat;
    };
    this.retrieve = function(mat) {
      const { cap } = this;
      if(!mat) mat = new Mat();
      if(cap.retrieve(mat)) return mat;
    };
    Util.weakAssign(this, Util.bindMethods(this.cap, VideoCapture.prototype));
  }

  fromImages(...images) {
    let cap = new ImageSequence(images);
    this.cap = cap;

    this.propId = prop => {
      if(typeof prop == 'string') {
        prop = prop.toLowerCase();
        if(prop.startsWith('cap_prop_')) prop = prop.slice(9);
      }
      return prop;
    };

    Util.weakAssign(this, Util.bindMethods(this.cap, ImageSequence.prototype));
  }

  get(prop) {
    const { cap } = this;
    if(cap && typeof cap.get == 'function') return this.cap.get(this.propId(prop));
  }

  set(prop, value) {
    return this.cap.set(this.propId(prop), value);
  }

  get backend() {
    const { cap } = this;
    if(cap && typeof cap.getBackendName == 'function') return cap.getBackendName();

    if(typeof this.get == 'function') {
      const id = this.get('BACKEND');
      return Util.findKey(VideoSource.backends, id);
    }
  }

  get fps() {
    return this.get('fps');
  }

  dump(props = ['frame_count', 'frame_width', 'frame_height', 'fps', 'format', 'fourcc', 'backend', 'pos_frames', 'pos_msec']) {
    return new Map(props.map(propName => [propName, this.get(propName)]).filter(([k, v]) => v !== undefined));
  }

  seek_frames(relative) {
    const pos = this.get('pos_frames') + relative;
    this.set('pos_frames', pos);
    return this.get('pos_frames');
  }

  seek_msecs(relative) {
    const msec_per_frame = 1000 / this.fps;
    this.seek_frames(relative / msec_per_frame);
    return this.get('pos_msec');
  }

  get duration_msecs() {
    const msec_per_frame = 1000 / this.fps;
    return +(this.get('frame_count') * msec_per_frame).toFixed(3);
  }

  position(type = 'frames') {
    if(type.startsWith('frame')) return [this.get('pos_frames'), this.get('frame_count')];
    if(type.startsWith('percent') || type == '%') return (this.get('pos_frames') * 100) / this.get('frame_count');

    return [+this.get('pos_msec').toFixed(3), this.duration_msecs];
  }
}

export default VideoSource;
