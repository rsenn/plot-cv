import Util from './lib/util.js';

function readFile(path) {
  let ret;
  try {
    ret = filesystem.readFile(path).toString();
  } catch(err) {}
  return ret;
}

function invertRanges(ranges, len) {
  let prev = 0;
  let ret = [];
  for(let { start, end } of ranges) {
    if(start > prev) ret.push({ start: prev, end: start });

    prev = end;
  }
  if(prev < len) ret.push({ start: prev, end: len });
  return ret;
}

function extractRanges(ranges, text) {
  let ret = [];
  for(let { start, end } of ranges) {
    ret.push({
      pos: lineColumn(start, text),
      code: text.substring(start, end),
      toString(filename) {
        return `${filename}:${this.pos.toString()}\n${Util.abbreviate(this.code, 100)}`;
      }
    });
  }
  return ret;
}

function lineColumn(pos, text) {
  let lines = text.substring(0, pos).split(/\n/g);
  let last = lines[lines.length - 1];
  return {
    line: lines.length,
    column: last.length,
    toString() {
      return `${this.line}:${this.column}`;
    }
  };
}

function processFile(arg, re) {
  let str = filesystem.readFile(arg).toString();
  let json = JSON.parse(str);
  //console.log('json:', json);
  re = typeof re == 'string' ? new RegExp(re) : /.*/;
  //if(!(json instanceof Array)) return 1;

  let scripts = json.map(({ url, ...item }) => [url.replace(/.*:\/\/[^/]*\//g, ''), item]).filter(([file]) => re.test(file));

  for(let [file, obj] of scripts) {
    let { ranges, text } = obj;
    try {
      let lines = text /* || readFile(file)*/
        .split(/\n/g);
      let inverted = invertRanges(ranges, text.length);
      let used = extractRanges(ranges, text);
      let unused = extractRanges(inverted, text);

      /* console.log('file:', file);
      //console.log('obj:', Object.keys(obj));
      //console.log('text.length:', text.length);
      //console.log('ranges:', ranges);
      //console.log('inverted:', inverted);*/
      //console.log('used:', used.map(u => u.toString(file).replace(/\n/g, '\\n')).join('\n'));
      //console.log('unused:', unused.map(u => u.toString(file)).join('\n\n'));
    } catch(err) {}
  }
}

function main(args) {
  const [file, expr] = args;

  if(!processFile(file, expr)) return 1;
  return 0;
}

Util.exit(main(Util.getArgs()));
