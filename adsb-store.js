import * as fs from 'fs';
import { glob, memoize } from 'util';
import { CurrentFile, DateToUnix, FilenameToTime, NextFile, PhaseFile, TimeToStr } from './adsb-common.js';
import * as std from 'std';

export function TimesForPhase(file) {
  file ??= CurrentFile();
  if(typeof file == 'number' || !/\.txt$/.test(file)) file = PhaseFile(+file);
  let f = std.open(file, 'r');
  if(!f) throw new Error(`Error opening '${file}'`);
  let line,
    prevTime = 0,
    index = 0;
  let ret = [],
    prevOffset = 0;
  while(!f.eof()) {
    if(!(line = f.getline())) break;
    let offset = f.tell();
    let idx = line.indexOf(',');
    let timeStr = line.substring(8, idx);
    let item = [timeStr, null, prevOffset, offset - prevOffset];
    //console.log('TimesForPhase', { index, line });
    ret.push(item);
    if(ret.length) ret[index][1] = +timeStr - prevTime;
    prevTime = +timeStr;
    prevOffset = offset;
    ++index;
  }
  return ret;
}

export function ReadRange(file, offset, size) {
  let f = std.open(file, 'r');
  if(f.seek(offset, std.SEEK_SET)) return null;

  let str = size !== undefined ? f.readAsString(size) : f.getline();
  f.close();
  return str;
}

export function StateFiles() {
  const d = '[[:digit:]]';
  return glob([d.repeat(4), '-', d.repeat(2), '-', d.repeat(2), '-', d, '.txt'].join('')).map(file => [file, fs.sizeSync(file)]);
}

export function StatePhases() {
  let files = StateFiles();
  return files.map(([file, size]) => FilenameToTime(file));
}

export const timeStateCache = new Map();
export const timeStateMap = memoize(file => TimesForPhase(file), timeStateCache);

export function GetStates(file) {
  file ??= CurrentFile();
  timeStateCache.delete(CurrentFile());
  return timeStateMap(file);
}

export function GetNearestTime(t) {
  t ??= DateToUnix();

  let file = PhaseFile(t);
  //console.log('GetNearestTime', { t, file });

  let i = 0,
    prevTime = 0,
    nearest,
    ts = GetStates(file);
  for(let state of ts) {
    // console.log('GetNearestTime', {i,state});
    let [time, diff, offset, size] = state;
    //  console.log('GetNearestTime', time - t, TimeToStr({ time }), { diff, offset, size });
    if(t > prevTime && t <= time) {
      nearest = time;
      break;
    }
    prevTime = time;
    ++i;
  }
  nearest ??= prevTime;
  //console.log('GetNearestTime', TimeToStr({ t, nearest, diff: t - nearest }));
  return nearest;
}

export function GetStateArray(t) {
  let file = typeof t == 'string' ? t : PhaseFile(t);

  let ts = GetStates(file);
  return ts;
}

export function GetStateIndex(t) {
  let file = PhaseFile(t);

  let i = 0,
    prevTime = 0,
    ts = GetStates(file);
  for(let state of ts) {
    let [time, diff, offset, size] = state;
    if(t > prevTime && t <= time) return i;
    prevTime = time;
    ++i;
  }
}

export function DumpState(state) {
  if(Array.isArray(state)) {
    let [time, diff, offset, size] = state;
    return { time: TimeToStr(time), diff, offset, size };
  }
}

export function GetStateByTime(t) {
  let file = PhaseFile(t);

  let ts = GetStates(file);
  let index = GetStateIndex(t);
  return ts[index];
}

export function IsRange(str) {
  return /^\d+-\d+$/.test(str);
}

export function GetRange(str) {
  let matches = [...str.matchAll(/\d+/g)].map(([m]) => +m);
  return matches.slice(0, 2);
}

export function ResolveRange(start, end) {
  let span = end - start;
  let ranges = [],
    t = GetNearestTime(start),
    u = GetNearestTime(end);
  let file = PhaseFile(t);
  let stateArray = GetStateArray(t);
  let count = 0,
    index = GetStateIndex(t);

  while(true) {
    let state = stateArray[index];
    if(!state) {
      file = NextFile(file);
      stateArray = GetStateArray(file);
      index = 0;
      state = stateArray[index];
    }
    let [time, diff, offset, length] = state;
    let range = ReadRange(file, offset) ?? ReadRange(file, offset, length);
    ranges.push([time, range]);
    if(t == u || time == u) break;
    t += diff;

    ++index;
    if(++count == 1000) break;
  }

  return ranges;
}

export function GetTimes(start, count = 1) {
  let times = [],
    t = GetNearestTime(start);
  let state,
    i = 0,
    file = PhaseFile(t),
    states = GetStateArray(t),
    index = GetStateIndex(t);

  do {
    if(!(state = states[index])) {
      file = NextFile(file);
      states = GetStateArray(file);
      index = 0;
      state = states[index];
    }
    times.push(+state[0]);
    t += state[1];

    ++index;
  } while(++i < count);

  return times;
}