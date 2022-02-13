const blah = 'test';

export { blah as test };

'blah'.replace(/.*\./, '');

let obj = {
  className:
    typeof itemClass == 'function'
      ? itemClass(value)
      : classNames(itemClass || className + '-item', (name + '').replace(/.*\./, '')),
  active: i == active,
  onPush: pushHandler(i),
  label: name.replace(/.*\//, ''),
  tooltip: tooltip({ title, name, description, i, number, data, ...item }),
  name,
  description,
  ...item
};
