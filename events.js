import Util from './lib/util.js';
import { Repeater } from './lib/repeater/repeater.js';

export function Emitter(target) {
  const listeners = new Map();
  let emitter = new.target ? this : {};
  Object.assign(emitter, {
    on: (type, handler) => (listeners.set(type, handler), target.addEventListener(type, handler), emitter),
    off: (type, handler) => (target.removeEventListener(type, handler || listeners.get(type)), listeners.delete(type), emitter),
    reset: () => {
      for(let [type, handler] of listeners) target.removeEventListener(type, handler);
      listeners.clear();
      return emitter;
    }
  });
  return Util.define(emitter, { listeners, target });
}

export function EventIterator(events, target = Util.tryCatch(() => window)) {
  let emitter = new Emitter(target);
  if(typeof events == 'string') events = EventIterator[events + 'Events'] || events.split(/,/g);

  let iter = new Repeater(async (push, stop) => {
    let handler = e => {
      e.emitter = emitter;
      push(e);
    };

    for(let type of events) emitter.on(type, handler);
    console.log('registered', events);
    await stop;
    /*for(let type of events) emitter.off(type, handler);
      console.log('unregistered', events);*/
    emitter.reset();
  });
  return Util.define(iter, { emitter, target });
  iter.emitter = emitter;
  return iter;
}

const touchEvents = ['touchmove', 'touchstart', 'touchcancel', 'mousemove', 'mouseup', 'mousedown'];
const keyEvents = ['keydown'];
Object.assign(EventIterator, { touchEvents, keyEvents });
