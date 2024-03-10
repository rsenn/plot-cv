import { iterate, RETURN_VALUE_PATH } from 'deep';

export function* JSON_Iterator(obj, pred = (v, k) => true) {
  let prev = { depth: 0, path: [], type: valueType(obj) };
  let stack = [];
  let parent;
  let str = '';
  const nl = () => '\n' + ' '.repeat(stack.length * 2);
  /*const push = {
    array() {
      let obj;
      stack.push(obj=[']', parent]);
      parent = 'array';
      str += '[' + nl();
      return obj;
    },
    object() {
      let obj;
      stack.push(obj=['}', parent]);
      parent = 'object';
      str += '{' + nl();
      return obj;
    }
  };*/

  //ush[prev.type]();
  stack[0] = ['}', 'object'];

  for(let [value, path] of iterate(obj, pred, RETURN_VALUE_PATH)) {
    const depth = path.length;
    const key = path[depth - 1];
    const type = valueType(value);
    let c = common(path, prev.path);
    const descend = path.length - c;
    const ascend = prev.path.length - c;
    const delta = descend - ascend;
    const keys = { ascend: prev.path.slice(c), descend: path.slice(c) };

    //   while(stack.length > c) {
    for(let i = 0; i < ascend - 1; ++i) {
      let [tmp, p] = stack.pop();
      str += nl() + tmp;
      parent = p;
    }

    if(delta <= 0) str += ',' + nl();

    const isnum = n => !isNaN(+n);
    const m = [
      ['{', '}', 'object'],
      ['[', ']', 'array']
    ];

    for(let i = c; i < path.length; ++i) {
      let [open, close, name] = m[isnum(path[i]) | 0];

      str += open+nl();
      stack[ i] = [close, m[isnum(path[ i-1]) | 0][2], path[ i-1]];
      parent = name;

      //   push[isnum(path[c+i]) ? 'array' : 'object']()[2]=path[c+i];
    }

    if(/^[A-Za-z_$][A-Za-z_$0-9]*$/.test(key)) str += key + ':';
    else if(parent != 'array') str += JSON.stringify(key) + ':';

    /*switch(type) {
  case 'object':str+='{'+nl(); stack[depth]=['}', parent]; break;
  case 'array': str+='['+nl(); stack[depth]=[']', parent]; break;
case 'null':str += 'null'; break;
default: 
}*/
    if(type == 'object' || type == 'array') {} 
    else if(type == 'null' || value === null) str += 'null';
    else str += JSON.stringify(value).trim();

    if(str.length >= 2) {
      yield str;
      str = '';
    }

    console.log('JSON_Iterator', console.config({ compact: 3, depth: 4 }), {
      // delta: `-${descend}/+${ascend} = ${((delta > 0 ? '+' : '') + delta).padStart(2)} -> ${depth}`,
      type,
      depth,
      ascend,
      descend,
      delta,
      stackLen: stack.length,
      path: s(path),
      stack,
      c
    });

    prev = { depth, path, type };
  }
  while(stack.length) {
    let [tmp, p] = stack.pop();
    str += nl() + tmp;
  }
  if(str.length >= 0) yield str;

  function s(p) {
    return p.join('.');
  }
  function common(a, b) {
    let n = Math.min(a.length, b.length);
    //console.log('common', {n, a,b});
    for(let i = 0; i < n; i++) if(a[i] != b[i]) return i;
    return n;
  }
  function valueType(value) {
    return Array.isArray(value) ? 'array' : value !== null ? typeof value : 'null';
  }
}