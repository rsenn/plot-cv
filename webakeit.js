const Q = (s) => document.querySelector(s);
const QA = (s) => ((s = document.querySelectorAll(s)), s ? [...s] : []);

const OM = (o,fn) => Object.entries(o).map(([name,value]) => fn(name,value));

Object.assign(globalThis, { Q, QA });




window.addEventListener('load', () => {
  const layers = globalThis.layers=QA('body > div').slice(0).reduce((o, e) => {
    const id = e.getAttribute('data-name');
    o[id] = e;
    return o;
  }, {});


  QA('div#header > div').forEach(e => {
    const name = e.getAttribute('data-name');

    e.addEventListener('click', e => {

      if(name in layers)  
    OM(layers, (id,layer) => {
      console.log('OM', {id,layer})
      layer.style.setProperty('display', id == name ? 'block' : 'none');
    });

      window.location.hash=name;


    });
  });



});
