import { iterate, RETURN_VALUE_PATH } from 'deep';

export function* JSON_Iterator(obj, pred = (v, k) => true) {
  let prev = { depth: 0, path: [], type: valueType(obj) };
  let stack = [];
  let parent;
  let str = '';
  const nl = () => '\n' + ' '.repeat(stack.length * 2);
  const push = {
    array: () => (stack.push([']', parent]), (parent = 'array'), (str += '[' + nl())),
    object: () => (stack.push(['}', parent]), (parent = 'object'), (str += '{' + nl()))
  };

  push[prev.type]();

  for(let [value, path] of iterate(obj, pred, RETURN_VALUE_PATH)) {
    const depth = path.length;
    const key = path[depth - 1];
    const type = valueType(value);
    let c = common(path, prev.path);
    let descend = prev.depth - c,
      ascend = depth - c;
    let d = descend ? -(descend - 1) : ascend;

    if(d < 0)
      for(let i = d; i < 0; i++) {
        let [tmp, p] = stack.pop();
        str += nl() + tmp;
        parent = p;
      }
    if(d <= 0) str += ',' + nl();

    if(/^[A-Za-z_$][A-Za-z_$0-9]*$/.test(key)) str += key + ':';
    else if(parent != 'array') str += JSON.stringify(key) + ':';

    if(push[type]) push[type]();
    else str += JSON.stringify(value).trim();

    if(str.length >= 2) {
      yield str;
      str = '';
    }

    prev = { depth, path, type };
  }
  while(stack.length) {
    let [tmp, p] = stack.pop();
    str += tmp;
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
    return Array.isArray(value) ? 'array' : typeof value;
  }
}
