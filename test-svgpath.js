import { SVG } from './lib/dom.js';
import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 7, colors: true }
});

const d = 'M 193.54706,178.86683 163.80521,218.90155 116.21174,233.8085 68.945718,217.89373 40.061173,177.23615 40.591015,127.36556 70.332862,87.330839 117.92634,72.423889 165.19236,88.338658 194.0769,128.99624 Z';

const d2 = 'M6.13 26.94L16.33 4.5l4.887 25.657 13.16-26.689 5.545 1.948 14.276 4.896-18.561 8.397-7.08 15.744 30.796-4.765 8.73-18.562-1.895-3.904.087-.066';

const data = SVG.parsePath(d).str();
const data2 = SVG.parsePath(d2).str();

//console.log('test-svgpath:', { data, data2 });
