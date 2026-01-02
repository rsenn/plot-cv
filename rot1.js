const str = 'DU BIST SCHLAU';

const A = 'A'.codePointAt(0);
const NUM_LETTERS = 'Z'.codePointAt(0) - A + 1;

console.log(
  str
    .split('')
    .map(ch => {
      if(ch != ' ') ch = String.fromCodePoint(((ch.codePointAt(0) - A + 1) % NUM_LETTERS) + A);

      return ch;
    })
    .join(''),
);
