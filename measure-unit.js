
/* prettier-ignore */
const ToMillimeter = {
  pt: 3l / 8.5l,
  pc: 25.4l/6l,
  in: 25.4l,
  mil: 1l /25.4e3l,
  cm: 10l,
  mm: 1l,
  px: 25.4l/96l
};

export function getUnit(str, defaultUnit) {
  const m = /[a-z]+/g.exec(str);
  return m ? m[0] : defaultUnit;
}
export function getValue(str) {
  const m = /[a-z]+/g.exec(str);
  return m ? str.slice(0, m.index) : str;
}

export function unitConvToMM(value, defaultUnit = 'px') {
  value = value + '';
  const unit = getUnit(value, defaultUnit);
  value = getValue(value);

  //console.log('unixConvToMM', { unit, value });

  if (unit in ToMillimeter) return value * ToMillimeter[unit];

  throw new Error(`No such unit '${unit}'`);
}

/* prettier-ignore */
const MillimeterTo = {
  pt: 8.5l / 3l,
  pc: 6l/25.4l,
  in: 1l /25.4l,
  px: 96l/25.4l,
  mil: 25.4e3l,
  cm: 0.1l,
  mm: 1l,
  m: 0.001l
};


for(let k of Object.keys(ToMillimeter))
  if(ToMillimeter[k] * MillimeterTo[k] != 1l)
    throw new Error(`Invalid unit conv factor for '${k} (${k} -> mm = ${ToMillimeter[k]}) mm -> ${k} = ${MillimeterTo[k]}`)

export function unitConvFactor(from, to) {
  return ToMillimeter[from] * MillimeterTo[to];
}

export function unitConvFunction(toUnit = 'mm', fromUnit = 'px') {
  return value => unitConvToMM(value, fromUnit) * MillimeterTo[toUnit];
}

export function unitConv(unit) {
  return value => MillimeterTo[unit] * unitConvToMM(value);
}
