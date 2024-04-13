const Q = s => document.querySelector(s);
const QA = s => ((s = document.querySelectorAll(s)), s ? [...s] : []);

const OM = (o, fn) => Object.entries(o).map(([name, value]) => fn(name, value));

const stack = (globalThis.stack = ['home']);
let layers = {};
let scrollTop = 0;

/*window.addEventListener('load', () => {
  layers = globalThis.layers = QA('body > div')
    .slice(0)
    .reduce((o, e) => {
      const id = e.getAttribute('data-name');
      o[id] = e;
      return o;
    }, {});

  QA('div#header > div').forEach(e => {
    const name = e.getAttribute('data-name');
    if(name)
      e.addEventListener('click', e => {
        if(name in layers) SelectPage(name);
      });
  });
});*/

function SelectPage(name) {
  OM(layers, (id, layer) => layer.style.setProperty('display', id == name ? (id == 'home' ? 'flex' : 'block') : 'none'));

  window.location.hash = name;
  stack.splice(1, stack.length - 1, name);
}

function PopPage() {
  stack.pop();
  let name = stack[stack.length - 1];

  OM(layers, (id, layer) => layer.style.setProperty('display', id == name ? (id == 'home' ? 'flex' : 'block') : 'none'));
  //if(name == 'home') window.location = window.location.href.replace(/#.*/g, ''); else
  window.location.hash = name == 'home' ? '' : name;
}

function HidePage() {
  let name = stack[stack.length - 1];

  scrollTop = window.scrollY ?? document.body.scrollTop;
  layers[name].style.setProperty('display', 'none');
}

function ShowPage() {
  let name = stack[stack.length - 1];

  layers[name].style.setProperty('display', 'block');
  //document.body.scrollTop = scrollTop;
  window.scrollTo(0, scrollTop);
}

function CreateElement(tag, attributes = {}, parent = document.body) {
  let e = document.createElement(tag);

  for(let attr in attributes) e.setAttribute(attr, attributes[attr]);

  if(parent) parent.appendChild(e);

  return e;
}

function ShowPhoto(element) {
  const { width, height, src } = element;

  // HidePage();

  let overlay = CreateElement('div', { class: 'overlay' });

  let photo = CreateElement('img', { src, style: width > height ? 'width: 100%' : 'height: 100%' }, overlay);
  let button = CreateElement('img', { src: 'static/img/schliessen.svg', class: 'close', style: `cursor: pointer;` }, overlay);

  button.addEventListener('click', e => {
    document.body.removeChild(overlay);
    ShowPage();
  });

  console.log('ShowPhoto', element);
}

Object.assign(globalThis, { Q, QA, SelectPage, PopPage, HidePage, CreateElement, ShowPhoto });
