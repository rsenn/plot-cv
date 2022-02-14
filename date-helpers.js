export function parseDate(str) {
  let d,
    m = /^(0?[1-9]|1[0-2])\D(0?[1-9]|[12]\d|3[01])\D(\d\d\d\d)/.exec(str);

  if(m) {
    d = new Date(0);
    //console.log('m', m.slice(1).map(n => +n));
    d.setMonth(m[1] - 1);
    d.setDate(+m[2]);
    d.setFullYear(+m[3]);
  }

  return d ?? m;
}

export function dateToObject(d) {
  return {
    timezoneOffset: d.getTimezoneOffset(),
    time: d.getTime(),
    year: d.getYear(),
    fullYear: d.getFullYear(),
    month: d.getMonth(),
    date: d.getDate(),
    hours: d.getHours(),
    minutes: d.getMinutes(),
    seconds: d.getSeconds(),
    milliseconds: d.getMilliseconds(),
    day: d.getDay()
  };
}
