export const quarterDay = 21600;
export const localeStr = 'de-CH';

export function Time(t, offset = 0) {
  if(typeof t == 'string') t = Date.parse(t) * 1e-3;

  let dt = t ? new Date(t * 1e3) : new Date();
  return Math.floor(+dt * 1e-3 + offset);
}

export function TimeToStr(t, offset = 0) {
  if(typeof t == 'object' && t != null) {
    if(!(t instanceof Date)) {
      let obj = {};
      for(let [name, value] of Object.entries(t)) {
        obj[name] = TimeToStr(value);
      }
      return obj;
    } else {
      return TimeToStr(+t * 1e-3);
    }
  }
  t = Time(t, offset);
  if(t >= 1651363200) throw new Error(`Invalid time: ${t}`);
  let dt = new Date(t * 1000);
  //return dt.toISOString().replace(/T(.*)\.\d{3}(.*)/, ' $1$2');
  return dt.toLocaleDateString(localeStr) + ' ' + dt.toLocaleTimeString(localeStr);
}

export function FilenameToTime(str) {
  let quarter = str.replace(/\d\d\d\d-\d+-\d+-/, '').replace(/\.txt$/, '') - 1;
  str = str.replace(/-\d\.txt$/, '');
  return Math.floor(new Date(str) * 1e-3) + quarter * quarterDay;
}

export function DateToUnix(dt = new Date()) {
  return Math.floor(dt * 1e-3);
}

export function NextFile(filename) {
  let t = FilenameToTime(filename);
  t += quarterDay;
  return PhaseFile(t);
}

export function CurrentFile(filename) {
  return PhaseFile(DateToUnix());
}

export function DailyPhase(t) {
  t -= t % quarterDay;
  return Math.floor(t);
}

export function PhaseFile(t = DateToUnix()) {
  let date = new Date(DailyPhase(+t * 1000));
  let str = date.toISOString().replace(/T.*/g, '');
  let phaseStr = Math.floor((date / (quarterDay * 1000)) % 4) + 1 + '';
  let file = str + '-' + phaseStr + '.txt';
  return file;
}

export function TransformCoordinates(...args) {
  if(args.length == 2) return transform(args, 'EPSG:4326', 'EPSG:3857');
  if(args.length == 4) {
    let extent = [args.splice(0, 2), args.splice(0, 2)];
    return extent.reduce((acc, coord) => acc.concat(TransformCoordinates(...coord)), []);
  }

  //return [...TransformCoordinates(args.slice(0,2)), ...TransformCoordinates(args.slice(2,4))];
  if(typeof args[0] == 'string') return TransformCoordinates(args[0].split(',').map(n => +n));
}

export class Coordinate {
  constructor(lon, lat) {
    this.lon = lon;
    this.lat = lat;
    return new Proxy(this, {
      get(target, prop, receiver) {
        if(prop == '0' || prop == '1') return TransformCoordinates(target.lon, target.lat)[+prop];

        return Reflect.get(target, prop, receiver);
      }
    });
  }
}
