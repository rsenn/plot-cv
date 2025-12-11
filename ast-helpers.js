import { get, select, iterate, RETURN_PATH_VALUE, RETURN_VALUE_PATH, RETURN_VALUE, PATH_AS_POINTER, FILTER_HAS_KEY, TYPE_OBJECT } from 'deep';
import { Pointer } from 'pointer';
import { readFileSync } from 'fs';
import { isObject,isFunction, isString, isNumeric, types,weakMapper } from 'util';

export function GetQualifier(ast, path) {
  const ptr = new Pointer(path);
  const a = [];
  for(const [p, n] of ptr.hier(p => [p, p.deref(ast)]).filter(([p, n]) => 'type' in n)) {
    //if('id' in n)
    a.push([n.type, n.id?.name ?? n.key?.name]);
  }

  return a;
}

export function IterateNodes(root, pred) {
  return iterate(root, pred, RETURN_VALUE_PATH | FILTER_HAS_KEY, TYPE_OBJECT, ['type']);
}

export function* GetImportSpecifiers(root) {
  for(const [spec, path] of IterateNodes(root, e => e.type == 'ImportSpecifier')) yield [spec, new Pointer(path)];
}

export function IsInsideAny(ast, path, pred) {
  for(const [n, p] of WalkUp(ast, path)) if(pred(n, p, ast)) return true;
  return false;
}

export function ParentPath(path) {
  return new Pointer(path).slice(0, isNumeric(path[path.length - 1]) ? -2 : -1);
}


export function ParentNode(ast,path) {
  let i=0;
  for(let[n,p] of WalkUp(ast,path)) {
if(i) {
  if(isObject(n) && n.type) return [n,p];
}

++i;
  }
} 


let sourceFiles=weakMapper(source=> readFileSync(source, 'utf-8').trimEnd().split(/\n/g), new Map());


export function GetSourceLine(source, line) {
return sourceFiles(source)[line-1]; 
}

export function Predicate( pred) {
  if(isString(pred)) {
    const s = pred;
    pred = n => n.type == s;
  } else if(types.isRegExp(pred)) {
    const re = pred;
    pred = n => re.test(n.type);
  }

if(!isFunction(pred))
  pred=() => true;

return pred;
}


export function IsInside(ast, path, pred) {
  const [n,p] = ParentNode(ast,path);
  return Predicate(pred)(n, p, ast);
}

export function* WalkUp(ast, path) {
  const n = path.length;
  for(let i = n; i >= 0; i--) {
    const p = path.slice(0, i);
    yield [get(ast, p), p];
  }
}

export function* WalkDown(ast, path) {
  const n = path.length;
  for(let i = 0; i <= n; i++) {
    const p = path.slice(0, i);
    yield [get(ast, p), p];
  }
}

export function* GetNodeTypes(ast, path) {
  for(const [n, p] of WalkDown(ast, path)) if(isObject(n) && n.type) yield [n.type, p];
}

export function GetImportIdentifiers(root) {
  if(!types.isGenerator(root)) root = GetImportSpecifiers(root);

const r={};
  for(const [spec, path] of root) r[spec.local.name]= new Pointer(path);

return r;
}

export function* GetNonImportIdentifiers(root) {
  for(const [id, path] of IterateNodes(root, e => e.type == 'Identifier')) {
    if(IsInsideAny(root, path, e => /Import.*Decl/.test(e.type))) continue;

if(path[path.length-1] == 'property') continue;

    yield [id.name, new Pointer(path)];

  }
}
