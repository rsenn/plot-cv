export function isElement(obj) {
  return ['tagName', 'parentElement', 'ownerDocument', 'attributes', 'children'].every(prop => prop in obj);
}

export function createElement(tag, attrs = {}, children = []) {
  let e = document.createElement(tag);

  for(let name in attrs) {
    let value = attrs[name];
    e.setAttribute(name, value);
  }

  for(let child of children) {
    if(!isElement(child)) {
      if(Array.isArray(child)) {
        child = dom(...child);
      }
    }
    e.appendChild(child);
  }

  return e;
}
