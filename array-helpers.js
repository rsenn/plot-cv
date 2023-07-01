export function prioritySort(arr, predicates = []) {
  const matchPred = item => predicates.findIndex(p => p(item));
  return [...arr].sort((a, b) => matchPred(b) - matchPred(a));
}
