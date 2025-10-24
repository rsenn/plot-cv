import { get } from 'deep';
import { Pointer } from 'pointer';

export function GetQualifier(ast, path) {
  const ptr = new Pointer(path);
  const a = [];
  for(const [p, n] of ptr.hier(p => [p, p.deref(ast)]).filter(([p, n]) => 'type' in n)) {
    //if('id' in n)
    a.push([n.type, n.id?.name ?? n.key?.name]);
  }

  return a;
}
