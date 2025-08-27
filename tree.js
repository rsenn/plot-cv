const createArrayChildrenExtractor = childrenKey => {
  return node => node[childrenKey];
};
/**
 * create a flattener method that can flatten tree into node list
 * @param childrenExtractor
 */
export const createTreeFlattener = childrenExtractor => {
  const flattener = node => {
    const children = childrenExtractor(node);
    if(!children || !Array.isArray(children)) {
      return [node];
    }
    return children.map(flattener).reduce(
      (list, childList) => {
        return list.concat(childList);
      },
      [node],
    );
  };
  return flattener;
};
/**
 * create a flattener method that can flatten tree into node list, specialized version of childrenExtractor
 * @param childrenKey
 */
export function createArrayChildrenTreeFlattener(childrenKey) {
  return createTreeFlattener(createArrayChildrenExtractor(childrenKey));
}
/**
 * create a filter method that can generate a new tree with nodes filtered
 * @param filter
 * @param childrenExtractor
 * @param nodeCloner
 */
export function createTreeFilter(filter, childrenExtractor, nodeCloner) {
  const treeFilter = node => {
    if(!filter(node)) {
      return undefined;
    }
    const children = childrenExtractor(node);
    if(!children || !Array.isArray(children)) {
      // unchanged, just return node
      return node;
    }
    const childrenReplace = children.map(child => treeFilter(child)).filter(child => !!child);
    return nodeCloner(node, childrenReplace);
  };
  return treeFilter;
}
/**
 * create a filter method that can generate a new tree with nodes filtered, specialized version of createTreeFilter
 * @param filter
 * @param childrenKey
 */
export function createArrayChildrenTreeFilter(filter, childrenKey) {
  return createTreeFilter(filter, createArrayChildrenExtractor(childrenKey), (node, childrenReplace) => {
    return {
      ...node,
      [childrenKey]: childrenReplace,
    };
  });
}
/**
 * create a finder method that can find a node matches filter in breadth first order
 * @param filter
 * @param childrenExtractor
 */
export function createBreadthFirstTreeFinder(filter, childrenExtractor) {
  return root => {
    const queue = [root];
    while(queue.length > 0) {
      const node = queue.shift();
      if(filter(node)) {
        return node;
      }
      const children = childrenExtractor(node);
      if(children && Array.isArray(children)) {
        queue.push(...children);
      }
    }
    return undefined;
  };
}
/**
 * create a finder method that can find a node matches filter in breadth first order, specialized version of createBreadthFirstTreeFinder
 * @param filter
 * @param childrenKey
 */
export function createBreadthFirstArrayChildrenTreeFinder(filter, childrenKey) {
  return createBreadthFirstTreeFinder(filter, createArrayChildrenExtractor(childrenKey));
}
