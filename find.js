export const find = (root, filter, path) => {
  let elementPath, k, v, ret;
  path = typeof path == 'string' ? path.split(/\.\//) : path;
  if (!path) path = [];
  if (filter(root, path)) ret = { path, value: root };
  else if (Util.isObject(root))
    for (k in root) {
      ret = select(root[k], filter, [...path, k]);
      if (ret) break;
    }
  return ret;
};
