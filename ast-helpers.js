import { get, select, iterate, RETURN_PATH_VALUE, RETURN_VALUE_PATH, RETURN_VALUE, RETURN_PATH, PATH_AS_POINTER, FILTER_HAS_KEY, TYPE_OBJECT } from 'deep';
import { Pointer } from 'pointer';
import { readFileSync } from 'fs';
import { isObject, isFunction, isString, isNumeric, types, mapFunction, weakMapper } from 'util';

export const TreeSource = mapFunction(new WeakMap());
export const SourceLines = weakMapper(source => ('\n' + readFileSync(source, 'utf-8').trimEnd()).split(/\n/g), new Map());

export const NodeTree = mapFunction(new WeakMap());
export const NodePath = mapFunction(new WeakMap());

export function MakePointer(p) {
  return Pointer.isPointer(p) ? p : new Pointer(p);
}

export function GetNodeArguments(a) {
  if(Array.isArray(a) && Array.isArray(a[0]) && a[0].length == 2) a = [...a.shift()];

  let ast, ptr;

  if(a.length >= 1 && !a[1]) {
    const node = a.shift();

    ptr = NodePath(node);
    ast = NodeTree(node);
  }

  if(!ast && !ptr)
    if(a.length >= 2 && a[1]) {
      [ast, ptr] = a.splice(0, 2);
    }

  ast = NodeTree(ast) ?? ast;
  ptr = [...ptr];

  if(ast && ptr) return [ast, ptr];
}

export function GetQualifier(...args) {
  const a = [];

  for(const [n, p] of WalkDown(...args)) {
    const part = n.id?.name ?? n.key?.name;

    /*if(part)*/ a.push([part, n, p]);
  }

  return a;
}

export function* IterateNodes(root, pred) {
  for(const [n, p] of iterate(root, NodePredicate(pred), RETURN_VALUE_PATH | FILTER_HAS_KEY, TYPE_OBJECT, ['type'])) yield GetNodePath(root, p);
}

export function* GetImportSpecifiers(root) {
  for(const [spec, path] of IterateNodes(root, e => e.type == 'ImportSpecifier')) yield [spec, new Pointer(path)];
}

export function IsInsideAny(ast, path, pred) {
  for(const [n, p] of WalkUp(ast, path)) if(NodePredicate(pred)(n, p, ast)) return true;
  return false;
}

export function ParentPath(path) {
  return new Pointer(path).slice(0, isNumeric(path[path.length - 1]) ? -2 : -1);
}

export function GetParentNodePath(...args) {
  let i = 0;

  for(const [n, p] of WalkUp(...args)) if(i++) return [n, p];
}

export function SourceRange(source, range) {
  return SourceLines(source)
    .slice(1)
    .join('\n')
    .substring(...range);
}

export function GetNode(ast, path) {
  const node = get(ast, path);
  const ptr = NodePath(node) ?? new Pointer(path);
  NodeTree(node, ast);
  NodePath(node, ptr);
  return node;
}

export function GetNodePath(ast, path) {
  const node = get(ast, path);
  const ptr = NodePath(node) ?? new Pointer(path);
  NodeTree(node, ast);
  NodePath(node, ptr);
  return [node, ptr];
}

export function GetPath(ast, path) {
  const node = get(ast, path);
  const ptr = NodePath(node) ?? new Pointer(path);
  NodeTree(node, ast);
  NodePath(node, ptr);
  return ptr;
}

export function NodePredicate(pred) {
  if(isString(pred)) {
    const s = pred;
    pred = n => n.type == s;
  } else if(types.isRegExp(pred)) {
    const re = pred;
    pred = n => re.test(n.type);
  }
  if(!pred) pred = () => true;
  return pred;
}

export function IsInside(ast, path, pred) {
  const [n, p] = GetParentNodePath(ast, path);
  return NodePredicate(pred)(n, p, ast);
}

export function* WalkUp(...args) {
  const [ast, path] = GetNodeArguments(args);

  //path = [...path];
  const pred = NodePredicate(args.shift() ?? (n => !Array.isArray(n)));

  for(let i = path.length; i >= 0; i--) {
    const node_path = GetNodePath(ast, path.slice(0, i));
    if(pred(...node_path, ast)) yield node_path;
  }
}

export function* WalkDown(...args) {
  const [ast, path] = GetNodeArguments(args);

  const pred = NodePredicate(args.shift() ?? (n => !Array.isArray(n)));

  const { length } = path;

  for(let i = 0; i <= length; i++) {
    const node_path = GetNodePath(ast, path.slice(0, i));
    if(pred(...node_path, ast)) yield node_path;
  }
}

export function* GetNodeTypes(...args) {
  for(const [n, p] of WalkDown(...args)) if(isObject(n) && n.type) yield [n.type, p];
}

export function GetImportIdentifiers(root) {
  if(!types.isGenerator(root)) root = GetImportSpecifiers(root);

  const r = {};
  for(const [spec, path] of root) r[spec.local.name] = new Pointer(path);

  return r;
}

export function* GetNonImportIdentifiers(root) {
  for(const [id, path] of IterateNodes(root, 'Identifier')) {
    if(IsInsideAny(root, path, /Import.*Decl/)) continue;

    //    if(path[path.length - 1] == 'property') continue;

    yield [id.name, new Pointer(path)];
  }
}
