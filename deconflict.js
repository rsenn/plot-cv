import { ReadFile, WriteFile } from './io-helpers.js';

function deconflict(s) {
  const re = /<<<<<<< ([^\r\n]*)\r?\n|=======\r?\n|>>>>>>> ([^\r\n]*)\r?\n/gm;

  let gen = util.repeat(
    v => v != null,
    m => (m = re.exec(s)) && [m[0], re.lastIndex - m[0].length, re.lastIndex]
  );

  let pos = 0;
  let out = ['', ''];
  let tags = [];

  for(;;) {
    let { done, value } = gen.next();
    if(done) break;
    let [match, start, end] = value;
    match = match.trimEnd();
    console.log('match', match);
    if(match.length > 8) {
      let idx = />>>>>>>/y.test(match) | 0;
      tags[idx] = match.slice(8);
    }

    if(start > pos) {
      let t = s.substring(pos, start);
      if(/<<<<<<<|=======/y.test(match)) out[0] += t;
      if(/<<<<<<<|>>>>>>>/y.test(match)) out[1] += t;
    }
    pos = end;
  }
  return out.map((s, i) => [tags[i], s]);
  return tags.reduce((acc, tag, i) => ({ ...acc, [tag]: out[i] }), {});
}

function deconflictFile(file) {
  let s = ReadFile(file, 'utf-8');
  let [[name1, a], [name2, b]] = deconflict(s);

  WriteFile(file + '.1', a);
  WriteFile(file + '.2', b);
}