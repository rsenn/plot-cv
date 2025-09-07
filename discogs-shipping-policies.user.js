// ==UserScript==
// @name         Discogs shipping policies
// @namespace    http://transistorisiert.ch/
// @version      20250907
// @description  try to take over the world!
// @author       You
// @match        https://www.discogs.com/settings/shipping
// @icon         https://www.google.com/s2/favicons?sz=64&domain=discogs.com
// @grant        none
// @downloadURL  https://github.com/rsenn/plot-cv/raw/refs/heads/main/discogs-shipping-policies.user.js
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/discogs-shipping-policies.user.js
// ==/UserScript==

let g = {};
globalThis.discogs = g;

setHandler = (
  fn = e => {
    const { type } = e;
    console.log(type, e);
    let price = getPrice();
    if(price) console.log('price', price.toFixed(22));
  }
) => findAll('select, input').forEach(e => ['change', 'keydown'].forEach(n => (e['on' + n] = fn)));

toArray = obj => (Array.isArray(obj) ? obj : [...obj]);
toObject = (arr, t = a => Object.fromEntries(a)) => {
  if(!Array.isArray(arr)) if (arr[Symbol.iterator]) arr = [...arr];
  if(Array.isArray(arr) /* && arr[0].length == 2*/) return t(arr);

  return arr;
};

defineEntries = entries => dest =>
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

toFn =
  (t = toArray) =>
  fn =>
  (...args) => {
    let r = fn(...args);
    return typeof r == 'object' && r && r[Symbol.iterator] ? t(r) : r;
  };

chainFn =
  (...fns) =>
  (...args) =>
    fns.reduce((acc, fn) => (acc.unshift(fn(...acc)), acc), args)[0];

chainFn1 =
  (...fns) =>
  (...args) =>
    fns.reduce((acc, fn) => (acc.unshift(fn(acc)), acc), args);

toArrayFn = fn => toFn(toArray)(fn);
toObjectFn = fn => toFn(toObject)(fn);
sliceFn =
  (fn, ...range) =>
  (...args) =>
    fn(...args).slice(...range);
trimFn = fn => s => (fn(s) + '').trim();

toEntries = toArrayFn(obj => (obj.entries ? obj.entries() : obj));

fromArrayLike = (t = (item, key, obj) => [key, item]) =>
  toArrayFn(function* (obj) {
    for(let i = 0; obj[i] !== undefined; i++) yield t(obj[i], i, obj);
  });

find = (query, root = document) => root.querySelector(query);
getElement = (query, root) => (typeof query == 'string' ? find(query, root) : query);
getElementFn = fn => e => fn(getElement(e));
findAll = toArrayFn((query, root = document) => root.querySelectorAll(query));
getterSetter =
  (...fns) =>
  (...args) =>
    (fns[args.length] ?? (() => {}))(...args);

up =
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
propertiesFn = style => {
  let fn;
  fn = getterSetter(
    () => [...style],
    k => style.getPropertyValue(k),
    style.setPropertyValue ? (k, v) => style.setPropertyValue(k, v) : () => {}
  );
  return fn;
};
propertyFn = (style, property) => {
  let fn,
    prop = propertiesFn(style);
  fn = (...args) => prop(property, ...args);
  fn.element = style;
  fn.property = property;
  return fn;
};

// getStyle = e=> fromArrayLike((v,k,o) => [k, propertyFn(o)])(e => window.getComputedStyle(getElement(e)), (v,k,o) => [v, propertyFn(o)])))

getStyle = chainFn(getElement, e => window.getComputedStyle(e));
toStyle = getElementFn(element => {
  let fn = propertiesFn(element.style);

  return (...args) => (args.length >= 2 ? e.style.setProperty(...args) : fn(...args));
});

propertyProxyFn =
  getSetFunction =>
  (obj = {}) =>
    new Proxy(obj, {
      get: (target, prop, receiver) => getSetFunction(prop) ?? Reflect.get(target, prop, receiver),
      ownKeys: target => getSetFunction() ?? Reflect.ownKeys(target)
    });
propertyProxy = (elem, obj = {}) => chainFn1(toStyle, propertyProxyFn)(elem)(obj);

getCols = (row, pred = e => !e.disabled) => findAll('input, select', row).filter(pred);
getRows = (e = 'tr.range') =>
  findAll(e ?? 'div.h2_wrap > div > table > tbody > tr')
    .map(row => getCols(row, () => true))
    .filter(row => row.length);
numRows = () => getRows().length;

callIt =
  (t = a => a()) =>
  (fn, ...path) =>
    t(fn);
call = val => (isNaN(+val) ? val : +val);
toNumber = val => (isNaN(+val) ? val : +val);
toString = s => s + '';

getSet = (element, a = 'value') => (Array.isArray(element) ? element.map(e => getSet(e, a)) : value => (value !== undefined ? element.setAttribute(a, value) : element.getAttribute(a)));
recurse = (arr, fn, path = [], root) => (Array.isArray(arr) ? arr.map((item, key) => recurse(item, fn, path.concat([key]), root ?? arr)) : fn(arr, path, root ?? arr));

getOptions = (element, t = a => a) => [...getElement(element ?? find('select')).children].filter(e => /^option/i.test(e.tagName)).map(t);
makeExpr = (s, f = 'gi') => (typeof s == 'string' ? new RegExp(s, f) : s);
makeExprFn =
  fn =>
  (...args) =>
    fn(makeExpr(...args));

getCards = () => findAll('.horizontal-card');
getCardTexts = () => getCards().map(e => (e.innerText + '').trim());
getCardPos = () => getCards().indexOf(find('.horizontal-card.selected'));
getCard = pos => (pos !== undefined ? getCards()[pos] : find('.horizontal-card.selected'));
getCardText = pos => (getCard(pos).innerText + '').trim();

setCard = pos => findAll('.horizontal-card')[pos].click();
getCardIterator = function* () {
  let i = 0;
  for(let card of getCards()) {
    setCard(i++);
    yield card;
  }
};
nextOption = e =>
  e.dispatchEvent(
    new KeyboardEvent('keydown', {
      code: 'ArrowDown',
      keyCode: 40,
      srcElement: e,
      target: e,
      currentTarget: e
    })
  );
numOptions = e => e.options.length;
getOptionTexts = e => getOptions(e, a => a.innerText);
getOptionValues = e => getOptions(e, a => a.getAttribute('value'));
getOptionEntries = e => getOptions(e, a => [a.getAttribute('value'), (a.innerText + '').trim()]);
getOptionIterator = function* (e, t) {
  let i,
    n = numOptions(e);
  t ??= () => getSelectText(e);
  if(e.selectedIndex == 0) e.selectedIndex = -1;
  for(i = 0; i < n; i++) {
    e.selectedIndex = i;
    yield t(e.options[i]);
  }
};

getInputLabel = (e = find('input')) => e.nextElementSiblininnerText;
getInputValue = (e = find('input')) => e.value;
getName = e => e.name;
getSelectValue = (e = find('select')) => e.value;
getSelectPos = (e = find('select')) => findOption(e)(getSelectValue());
getSelectOption = (e = find('select')) => (e.options ? e.options[e.selectedIndex] : null);
getSelectText = (e = find('select')) => getSelectOption(e)?.innerText;
getValue = e => (/^select$/i.test(e.tagName) ? /* getElementsByTagName('')*/ SelectValue(e) : getInputValue(e));
getText = trimFn(e => (({ select: getSelectText, input: getInputLabel }[e.tagName.toLowerCase()] ?? getCardText)(e))); //  /^select$/i.test(e.tagName) ? getSelectText(e) : getInputLabel(e)

getSelections = e => getOptions(e, (t = (o, i) => [o.innerText, /*o.value,*/ o.selected]));
setSelection = (e, i) => (getElement(e).selectedIndex = i); //getSelections(e, o => [e,o.innerText,o.value,i]).findIndex(([e,...s]) => s.some(t => t+'' == i+''));
findOption = e => makeExprFn(expr => getOptionEntries(e).findIndex((a, i) => a.some(t => expr.test(t))));
filterOptions = e => makeExprFn(expr => getOptionEntries(e).filter((a, i) => a.some(t => expr.test(t))));

delete rows;

matchNumbers = toArrayFn(str => str.matchAll(/([-+]?[0-9.]+)/g));
parseNumber = v => {
  let [m] = matchNumbers(v + '');
  return toNumber(...m);
};
getPriceText = (price = find('.price-row-price > span')) => (price?.innerText ?? '').trim();

getPrice = (price = find('.price-row-price > span')) => parseNumber(getPriceText(price));

getCountry = () => findAll('select, input').map(getText).concat([getPriceText()]) /*.map(t => (t+'').strip())*/;

get = (obj, path) => path.reduce((acc, key) => acc[key], obj);
setTo = (parent, key, value) => (parent[key] = value);
defineTo = (parent, key, ...fns) =>
  Object.defineProperty(parent, key, {
    get: fns[0],
    set: fns[1] ?? fns[0],
    configurable: true
  });

set = (obj, path, value) => setTo(get(obj, path.slice(0, -1)), path[path.length - 1], value);

getValues = () =>
  recurse(
    y,
    callIt(a => {
      let n = a();
      return ({ object: a => a, undefined: a => a }[typeof n] ?? toNumber)(n);
    })
  );
setValues = v => recurse(y, (item, path, root) => item(get(v, path) + ''));

defineGetter = (name, fn) =>
  Object.defineProperties(globalThis, {
    [name]: { get: fn, configurable: true }
  });
defineGetter('rows', getRows);
defineGetter('y', () => getSet(rows));
defineTo(globalThis, 'values', getValues, setValues);

removeRow = () => document.querySelector('td.range_remove > button').click();
addRow = () => document.querySelector('button.add_range').click();

setNumRows = n => {
  while(numRows() < n) addRow();
  while(numRows() > n) removeRow();
};

recurse(y, g => g());
