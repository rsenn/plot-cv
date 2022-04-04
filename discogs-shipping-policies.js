let g = globalThis;

g.setHandler = (
  fn = e => {
    const { type } = e;
    console.log(type, e);
    let price = getPrice();
    if(price) console.log('price', price.toFixed(22));
  }
) => findAll('select, input').forEach(e => ['change', 'keydown'].forEach(n => (e['on' + n] = fn)));

g.toArray = obj => (Array.isArray(obj) ? obj : [...obj]);
g.toObject = (arr, t = a => Object.fromEntries(a)) => {
  if(!Array.isArray(arr)) if (arr[Symbol.iterator]) arr = [...arr];
  if(Array.isArray(arr) /* && arr[0].length == 2*/) return t(arr);

  return arr;
};

g.defineEntries = entries => dest =>
  Object.defineProperties(
    dest,
    entries.reduce(
      (acc, [k, get, set]) => ({
        ...acc,
        [k]: { get, set: set ?? get, configurable: true }
      }),
      {}
    )
  );

g.toFn =
  (t = toArray) =>
  fn =>
  (...args) => {
    let r = fn(...args);
    return typeof r == 'object' && r && r[Symbol.iterator] ? t(r) : r;
  };

g.chainFn =
  (...fns) =>
  (...args) =>
    fns.reduce((acc, fn) => (acc.unshift(fn(...acc)), acc), args)[0];

g.chainFn1 =
  (...fns) =>
  (...args) =>
    fns.reduce((acc, fn) => (acc.unshift(fn(acc)), acc), args);

g.toArrayFn = fn => toFn(toArray)(fn);
g.toObjectFn = fn => toFn(toObject)(fn);
g.sliceFn =
  (fn, ...range) =>
  (...args) =>
    fn(...args).slice(...range);
g.trimFn = fn => s => (fn(s) + '').trim();

g.toEntries = toArrayFn(obj => (obj.entries ? obj.entries() : obj));

g.fromArrayLike = (t = (item, key, obj) => [key, item]) =>
  toArrayFn(function* (obj) {
    for(let i = 0; obj[i] !== undefined; i++) yield t(obj[i], i, obj);
  });

g.find = (query, root = document) => root.querySelector(query);
g.getElement = (query, root) => (typeof query == 'string' ? find(query, root) : query);
g.getElementFn = fn => e => fn(getElement(e));
g.findAll = toArrayFn((query, root = document) => root.querySelectorAll(query));
g.getterSetter =
  (...fns) =>
  (...args) =>
    (fns[args.length] ?? (() => {}))(...args);

g.up =
  (n = 1) =>
  node => {
    let i = 0;
    if(typeof n != 'function') {
      let x = n;
      n = (node, j) => j < x;
    }

    while(n(node, i++)) {
      if(!(node = node.parentElement)) break;
    }
    return node;
  };
g.propertiesFn = style => {
  let fn;
  fn = getterSetter(
    () => [...style],
    k => style.getPropertyValue(k),
    style.setPropertyValue ? (k, v) => style.setPropertyValue(k, v) : () => {}
  );
  return fn;
};
g.propertyFn = (style, property) => {
  let fn,
    prop = propertiesFn(style);
  fn = (...args) => prop(property, ...args);
  fn.element = style;
  fn.property = property;
  return fn;
};

// getStyle = e=> fromArrayLike((v,k,o) => [k, propertyFn(o)])(e => window.getComputedStyle(getElement(e)), (v,k,o) => [v, propertyFn(o)])))

g.getStyle = chainFn(getElement, e => window.getComputedStyle(e));
g.toStyle = getElementFn(element => {
  let fn = propertiesFn(element.style);

  return (...args) => (args.length >= 2 ? e.style.setProperty(...args) : fn(...args));
});

g.propertyProxyFn =
  getSetFunction =>
  (obj = {}) =>
    new Proxy(obj, {
      get: (target, prop, receiver) => getSetFunction(prop) ?? Reflect.get(target, prop, receiver),
      ownKeys: target => getSetFunction() ?? Reflect.ownKeys(target)
    });
g.propertyProxy = (elem, obj = {}) => chainFn1(toStyle, propertyProxyFn)(elem)(obj);

g.getCols = (row, pred = e => !e.disabled) => findAll('input, select', row).filter(pred);
g.getRows = (e = 'tr.range') =>
  findAll(e ?? 'div.h2_wrap > div > table > tbody > tr')
    .map(row => getCols(row, () => true))
    .filter(row => row.length);
g.numRows = () => getRows().length;

g.callIt =
  (t = a => a()) =>
  (fn, ...path) =>
    t(fn);
g.call = val => (isNaN(+val) ? val : +val);
g.toNumber = val => (isNaN(+val) ? val : +val);
g.toString = s => s + '';

g.getSet = (element, a = 'value') =>
  Array.isArray(element)
    ? element.map(e => getSet(e, a))
    : value => (value !== undefined ? element.setAttribute(a, value) : element.getAttribute(a));
g.recurse = (arr, fn, path = [], root) =>
  Array.isArray(arr)
    ? arr.map((item, key) => recurse(item, fn, path.concat([key]), root ?? arr))
    : fn(arr, path, root ?? arr);

g.getOptions = (element, t = a => a) =>
  [...getElement(element ?? find('select')).children].filter(e => /^option/i.test(e.tagName)).map(t);
g.makeExpr = (s, f = 'gi') => (typeof s == 'string' ? new RegExp(s, f) : s);
g.makeExprFn =
  fn =>
  (...args) =>
    fn(makeExpr(...args));

g.getCards = () => findAll('.horizontal-card');
g.getCardTexts = () => getCards().map(e => (e.innerText + '').trim());
g.getCardPos = () => getCards().indexOf(find('.horizontal-card.selected'));
g.getCard = pos => (pos !== undefined ? getCards()[pos] : find('.horizontal-card.selected'));
g.getCardText = pos => (getCard(pos).innerText + '').trim();

g.setCard = pos => findAll('.horizontal-card')[pos].click();
g.getCardIterator = function* () {
  let i = 0;
  for(let card of getCards()) {
    setCard(i++);
    yield card;
  }
};
g.nextOption = e =>
  e.dispatchEvent(
    new KeyboardEvent('keydown', {
      code: 'ArrowDown',
      keyCode: 40,
      srcElement: e,
      target: e,
      currentTarget: e
    })
  );
g.numOptions = e => e.options.length;
g.getOptionTexts = e => getOptions(e, a => a.innerText);
g.getOptionValues = e => getOptions(e, a => a.getAttribute('value'));
g.getOptionEntries = e => getOptions(e, a => [a.getAttribute('value'), (a.innerText + '').trim()]);
g.getOptionIterator = function* (e, t) {
  let i,
    n = numOptions(e);
  t ??= () => getSelectText(e);
  if(e.selectedIndex == 0) e.selectedIndex = -1;
  for(i = 0; i < n; i++) {
    e.selectedIndex = i;
    yield t(e.options[i]);
  }
};

g.getInputLabel = (e = find('input')) => e.nextElementSibling.innerText;
g.getInputValue = (e = find('input')) => e.value;
g.getName = e => e.name;
g.getSelectValue = (e = find('select')) => e.value;
g.getSelectPos = (e = find('select')) => findOption(e)(getSelectValue());
g.getSelectOption = (e = find('select')) => (e.options ? e.options[e.selectedIndex] : null);
g.getSelectText = (e = find('select')) => g.getSelectOption(e)?.innerText;
g.getValue = e => (/^select$/i.test(e.tagName) ? getSelectValue(e) : getInputValue(e));
g.getText = g.trimFn(e =>
  (({ select: getSelectText, input: getInputLabel }[e.tagName.toLowerCase()] ?? getCardText)(e))
); //  /^select$/i.test(e.tagName) ? getSelectText(e) : getInputLabel(e)

g.getSelections = e => getOptions(e, (t = (o, i) => [o.innerText, /*o.value,*/ o.selected]));
g.setSelection = (e, i) => (getElement(e).selectedIndex = i); //getSelections(e, o => [e,o.innerText,o.value,i]).findIndex(([e,...s]) => s.some(t => t+'' == i+''));
g.findOption = e => makeExprFn(expr => getOptionEntries(e).findIndex((a, i) => a.some(t => expr.test(t))));
g.filterOptions = e => makeExprFn(expr => getOptionEntries(e).filter((a, i) => a.some(t => expr.test(t))));

delete g.rows;

g.matchNumbers = toArrayFn(str => str.matchAll(/([-+]?[0-9.]+)/g));
g.parseNumber = v => {
  let [m] = matchNumbers(v + '');
  return toNumber(...m);
};
g.getPriceText = (price = find('.price-row-price > span')) => (price?.innerText ?? '').trim();

g.getPrice = (price = find('.price-row-price > span')) => parseNumber(g.getPriceText(price));

g.getCountry = () => findAll('select, input').map(getText).concat([getPriceText()]) /*.map(t => (t+'').strip())*/;

g.get = (obj, path) => path.reduce((acc, key) => acc[key], obj);
g.setTo = (parent, key, value) => (parent[key] = value);
g.defineTo = (parent, key, ...fns) =>
  Object.defineProperty(parent, key, {
    get: fns[0],
    set: fns[1] ?? fns[0],
    configurable: true
  });

g.set = (obj, path, value) => setTo(get(obj, path.slice(0, -1)), path[path.length - 1], value);

g.getValues = () =>
  recurse(
    g.y,
    callIt(a => {
      let n = a();
      return ({ object: a => a, undefined: a => a }[typeof n] ?? toNumber)(n);
    })
  );
g.setValues = v => recurse(g.y, (item, path, root) => item(get(v, path) + ''));

g.defineGetter = (name, fn) =>
  Object.defineProperties(globalThis, {
    [name]: { get: fn, configurable: true }
  });
defineGetter('rows', getRows);
defineGetter('y', () => getSet(g.rows));
defineGetter('y', () => getSet(g.rows));
defineTo(globalThis, 'values', getValues, setValues);

g.removeRow = () => document.querySelector('td.range_remove > button').click();
g.addRow = () => document.querySelector('button.add_range').click();

g.setNumRows = n => {
  while(numRows() < n) addRow();
  while(numRows() > n) removeRow();
};

recurse(y, g => g());
