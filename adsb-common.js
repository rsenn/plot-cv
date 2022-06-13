export const quarterDay = 21600;
export const localeStr = 'de-CH';

export function Time(t, offset = 0) {
  if(typeof t == 'string') t = Date.parse(t) * 1e-3;

  let dt = t ? new Date(t * 1e3) : new Date();
  return Math.floor(+dt * 1e-3 + offset);
}

export function TimeToStr(t, offset = 0) {
  t ??= DateToUnix();
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
<<<<<<< HEAD
  if(t >= 1967241600) throw new Error(`Invalid time: ${t} >= 1967241600`);
=======
  if(t >= 2282508000) throw new Error(`Invalid time: ${t} (> 2282508000)`);
>>>>>>> feea8ded234c8df594d73f8da5eff05e69d5469a
  let dt = new Date(t * 1000);
  //return dt.toISOString().replace(/T(.*)\.\d{3}(.*)/, ' $1$2');
  return dt.toLocaleDateString(localeStr) + ' ' + dt.toLocaleTimeString(localeStr);
}

export function FilenameToTime(file) {
  let quarter = file.replace(/\d\d\d\d-\d+-\d+-/, '').replace(/\.txt$/, '') - 1;
  file = file.replace(/-\d\.txt$/, '');
  return Math.floor(new Date(file) * 1e-3) + quarter * quarterDay;
}

export function DateToUnix(dt = new Date()) {
  if(typeof dt != 'object' || dt == null || !(dt instanceof Date)) dt = new Date(dt);

  return Math.floor(dt * 1e-3);
}

export function UnixToDate(epoch = Math.floor(Date.now() * 1e-3)) {
  return new Date(+epoch * 1e3);
}

export function NextFile(filename) {
  let t = FilenameToTime(filename);
  t += quarterDay;
  return PhaseFile(t);
}

export function PrevFile(filename) {
  let t = FilenameToTime(filename);
  t -= quarterDay;
  return PhaseFile(t);
}

export function CurrentFile() {
  return PhaseFile(DateToUnix());
}

export function DailyPhase(t) {
  t ??= DateToUnix();
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
