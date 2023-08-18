import lune from 'lune';
import MoonPhase from 'moonphase-js';
import SolarCalc from 'solar-calc';
import the_moon from 'the-moon';

const m = new MoonPhase(new Date(2018, 9, 23));

//console.log(m.illum * 100 + ' %');

function moon() {
  let ret = the_moon.apply(the_moon, [...arguments]);
  //console.log('ret:', ret);
  return ret;
}

//Get today's moon

moon(); //🌗 (today's moon)
//Choose a different date

moon(2017); //🌑 (today in 2017)
moon(2017, 3, 14); //🌕 (a specific date)
//Choose from available formats (moon.FORMATS) default is icon

moon(2017, 3, 14, { format: 'code' }); //'full-moon'
moon({ format: 'name' }); //'Last Quarter Moon'

//SolarCalc(date,lat,long)
const gps = { lat: 46.9480896, lon: 7.4474401 };
let solar = new SolarCalc(new Date(), gps.lat, gps.lon);
//console.log('solar:', solar /* 2015-03-08T11:35:30.000Z */);
//console.log([SunCalc.getTimes(/*Date*/ new Date(), /*Number*/ gps.lat, /*Number*/ gps.lon), SunCalc.getMoonPosition(/*Date*/ new Date(), /*Number*/ gps.lat, /*Number*/ gps.lon), SunCalc.getMoonIllumination(/*Date*/ new Date()), SunCalc.getMoonTimes(/*Date*/ new Date(), /*Number*/ gps.lat, /*Number*/ gps.lon)]);

let current_phase = lune.phase();
//console.log(current_phase);