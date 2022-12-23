export function* findAllIndexes(haystack, needle) {
  let { length: n } = needle;
  for(let i, j = 0; (i = haystack.indexOf(needle, j)) != -1; j = i + n) yield i;
}

export function* findAllIndexesReverse(haystack, needle) {
  let { length: n } = needle;
  for(let i, j; (i = haystack.lastIndexOf(needle, j)) != -1; j = i - n) yield i;
}

export function countSubstring(haystack, needle) {
  let ret = 0,
    { length: n } = needle;

  for(let i, j = 0; (i = haystack.indexOf(needle, j)) != -1; j = i + n) ++ret;

  return ret;
}

export function parseDegMinSec(s) {
  let matches = [...s.matchAll(/([0-9.]*)\s*([^\s0-9]+)/g)].map(([m, ...rest]) => rest);
  let r;

  if(matches && matches.length) {
    r = 0;
    for(let [value, unit] of matches) {
      if(value === '') value = 1;
      value = +value;
      r = {
        deg: (v, r) => r + v,
        ["'"]: (v, r) => r + v / 60,
        ['"']: (v, r) => r + v / 3600,
        N: (v, r) => r,
        W: (v, r) => -r,
        S: (v, r) => -r,
        E: (v, r) => r
      }[unit](+value, r);
      // console.log('parseDegMinSec', {value,unit,r});
    }
  }
  return r;
}

export function parseGPSLocation(s) {
  let a = s.split(/,\s+/);
  if(a && a.length == 2) return a.map(parseDegMinSec);
}

export function parseDMS(dmsString) {
  dmsString = dmsString.trim();

  // Inspired by https://gist.github.com/JeffJacobson/2955437
  // See https://regex101.com/r/kS2zR1/3
  var dmsRe =
    /([NSEW])?\s?(-)?(\d+(?:\.\d+)?)[°º:d\s]?\s?(?:(\d+(?:\.\d+)?)['’‘′:]?\s?(?:(\d{1,2}(?:\.\d+)?)(?:"|″|’’|'')?)?)?\s?([NSEW])?/i;

  var result = {};

  var m1, m2, decDeg1, decDeg2, dmsString2;

  m1 = dmsString.match(dmsRe);

  if(!m1) throw 'Could not parse string';

  // If dmsString starts with a hemisphere letter, then the regex can also capture the
  // hemisphere letter for the second coordinate pair if also in the string
  if(m1[1]) {
    m1[6] = undefined;
    dmsString2 = dmsString.substr(m1[0].length - 1).trim();
  } else {
    dmsString2 = dmsString.substr(m1[0].length).trim();
  }

  decDeg1 = decDegFromMatch(m1);

  m2 = dmsString2.match(dmsRe);

  decDeg2 = m2 ? decDegFromMatch(m2) : {};

  if(typeof decDeg1.latLon === 'undefined') {
    if(!isNaN(decDeg1.decDeg) && isNaN(decDeg2.decDeg)) {
      // If we only have one coordinate but we have no hemisphere value,
      // just return the decDeg number
      return decDeg1.decDeg;
    } else if(!isNaN(decDeg1.decDeg) && !isNaN(decDeg2.decDeg)) {
      // If no hemisphere letter but we have two coordinates,
      // infer that the first is lat, the second lon
      decDeg1.latLon = 'lat';
      decDeg2.latLon = 'lon';
    } else {
      throw 'Could not parse string';
    }
  }

  // If we parsed the first coordinate as lat or lon, then assume the second is the other
  if(typeof decDeg2.latLon === 'undefined') {
    decDeg2.latLon = decDeg1.latLon === 'lat' ? 'lon' : 'lat';
  }

  result[decDeg1.latLon] = decDeg1.decDeg;
  result[decDeg2.latLon] = decDeg2.decDeg;

  return result;
}

function decDegFromMatch(m) {
  var signIndex = {
    '-': -1,
    N: 1,
    S: -1,
    E: 1,
    W: -1
  };

  var latLonIndex = {
    N: 'lat',
    S: 'lat',
    E: 'lon',
    W: 'lon'
  };
  var degrees, minutes, seconds, sign, latLon;

  sign = signIndex[m[2]] || signIndex[m[1]] || signIndex[m[6]] || 1;
  degrees = Number(m[3]);
  minutes = m[4] ? Number(m[4]) : 0;
  seconds = m[5] ? Number(m[5]) : 0;
  latLon = latLonIndex[m[1]] || latLonIndex[m[6]];

  if(!inRange(degrees, 0, 180)) throw 'Degrees out of range';
  if(!inRange(minutes, 0, 60)) throw 'Minutes out of range';
  if(!inRange(seconds, 0, 60)) throw 'Seconds out of range';

  return {
    decDeg: sign * (degrees + minutes / 60 + seconds / 3600),
    latLon: latLon
  };
}

function inRange(value, a, b) {
  return value >= a && value <= b;
}
