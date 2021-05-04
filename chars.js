let i = 0;
let a = [
  'a',
  'b',
  'c',
  'd',
  'e',
  'f',
  'g',
  'h',
  'i',
  'j',
  'k',
  'l',
  'm',
  'n',
  'o',
  'p',
  'q',
  'r',
  's',
  't',
  'v',
  'w',
  'y',
  'z'
];

for(let c of [
  'a',
  '\b',
  'c',
  'd',
  'e',
  '\f',
  'g',
  'h',
  'i',
  'j',
  'k',
  'l',
  'm',
  '\n',
  'o',
  'p',
  'q',
  '\r',
  's',
  '\t',
  '\v',
  'w',
  'y',
  'z'
]) {
  let code = c.charCodeAt(0);
  let char = a[i];

  //console.log(i, a[i]);
  if(!(code >= 97 && code <= 122)) {
    //console.log({char,c,code});
    //console.log(`case '${char}': return ${code};`);
    console.log(`case ${code}: return '${char}';`);
  }
  i++;
}
