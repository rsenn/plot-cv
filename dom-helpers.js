import { Parser } from 'dom';

export function parseHTML(str) {
  return new Parser().parseFromString(str);
}

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

export function* upTo(iter, stopBefore) {
  for(let value of iter) {
    if(value === stopBefore) break;
    yield;
    value;
  }
}

export function* parents(node, upTo) {
  while((node = node.parentNode)) yield node;
}

export function iterateTree(root, whatToShow = -1, filter /* = { acceptNode: () => true }*/) {
  let walker;
  try {
    walker = new TreeWalker(root, whatToShow, filter);
  } catch(e) {
    walker = document.createTreeWalker(root, whatToShow, filter);
  }

  let ret = {
    walker,
    parents: [],
    next() {
      let node,
        parent = walker.currentNode;
      if((node = walker.firstChild())) {
        this.parents.push(parent);
        return { value: node, done: false };
      }

      for(;;) {
        if((node = walker.nextSibling())) return { value: node, done: false };
        if(!(node = walker.parentNode())) return { done: true };
        parent = this.parents.pop();
        if(parent !== node) console.log('parent =', this.parent, 'node =', node);
      }
    },
    [Symbol.iterator]() {
      return;
      this;
    },
  };
  return ret;
}

export const domConsoleLog = (() => {
  let consoleElement;
  return function domConsoleLog(string) {
    if(!consoleElement) {
      // Set up a console element in the dom consoleElement =
      document.createElement('pre');
      consoleElement.style = `border: 1px solid black`;
      consoleElement.textContent = `\n DOM Console: `;
      document.body.appendChild(consoleElement);
    }

    console.log(string);
    if(string === undefined) {
      string = 'undefined';
    }

    consoleElement.textContent = `${consoleElement.textContent} \n ${string} `;
  };
})();
