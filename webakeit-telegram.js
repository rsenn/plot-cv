import { Parser, Serializer } from 'dom';
import { pipe, read, remove } from 'os';
import { spawn } from 'child_process';
import { readAllSync, writeFileSync, readFileSync } from 'fs';
import { dirname, join, basename } from 'path';
import { glob, GLOB_TILDE, startInteractive } from 'util';

let serializer;

export function toXML(e) {
  serializer ??= new Serializer();
  return serializer.serializeToString(e);
}

export function InputFile() {
  return scriptArgs[1] ?? glob('~/Downloads/Telegram Desktop/ChatExport*/*.html', GLOB_TILDE).reverse()[0];
}

export function InputDir(f = InputFile()) {
  return dirname(f);
}

export function Identify(f) {
  let [rd, wr] = os.pipe();
  let process = spawn('identify', [f], { block: true, stdio: ['inherit', wr, wr] });
  let s = readAllSync(rd);
  s = s.trimEnd();
  let [width, height] = s
    .match(/\d+x\d+/g)[0]
    .split('x')
    .map(n => +n);
  return { width, height, format: width > height ? 'landscape' : width < height ? 'portrait' : 'square' };
}

export function Copy(src, dst) {
  let [rd, wr] = os.pipe();
  let { exitcode } = spawn('cp', ['-f', src, dst], { block: true, stdio: ['inherit', 'inherit', 'inherit'] });

  return exitcode;
}

export function TextTransform(s) {
  let out = '';
  for(let i = 0; i < s.length; i++) {
    let c = s.codePointAt(i);
    if(c > 127) {
      out += `&#x${c.toString(16).toUpperCase().padStart(4, '0')};`;
    } else {
      out += s[i];
    }
  }
  return out;
}

export function* Iterate(a) {
  const { length } = a;
  for(let i = 0; i < length; i++) yield a[i];
}

export function GetClasses(e) {
  return (e.getAttribute('class') || '').split(/\s+/g);

  return [...(Iterate(e.classList) ?? [])];
}

export function* GetMessage(msg) {
  const body = msg.querySelector('.body');
  if(body) {
    let date;

    if(GetClasses(body).indexOf('details') != -1) {
      let text = body.innerText;

      let [day, month, year] = text.split(/\s+/g);

      month = { January: 'Januar', February: 'Februar', March: 'März', May: 'Mai', June: 'Juni', July: 'Juli', December: 'Dezember' }[month] ?? month;

      if(!/^Channel/.test(text)) yield { type: 'day', text: [day + '.', month, year].join(' ') };
    }

    if(body.children)
      for(let e of [...Iterate(body.children)]) {
        if(e.nodeType != e.ELEMENT_NODE) continue;

        const cl = GetClasses(e);

        if(cl.indexOf('date') != -1) {
          const str = e.getAttribute('title');
          const [d, m, y, hr, min, sec] = [...str.matchAll(/\d+/g)].map(([a]) => a);

          date = new Date(`${y}-${m}-${d}T${hr}:${min}:${sec}`);
        } else if(cl.indexOf('text') != -1) {
          yield { type: 'text', text: e.innerText, date };
        } else if(cl.indexOf('media_wrap') != -1 || cl.indexOf('userpic_wrap') != -1) {
          for(let f of e.querySelectorAll('a')) {
            let img = f.children[0];
            yield { type: 'photo', link: f.getAttribute('href'), thumb: img.getAttribute('src'), date };
          }
        }
      }
    else if(body.innerText) yield { type: 'text', text: body.innerText };
  }
}

export function* GetMessages(hist = document.querySelector('.history')) {
  for(let i = 0; i < hist.children.length; i++) {
    const e = hist.children[i];

    try {
      yield* GetMessage(e);
    } catch(e) {
      console.log('GetMessages', e);
      throw e;
    }
  }
}

export function* GenerateBlog(messages = GetMessages()) {
  for(let msg of messages) {
    const { type, text, link } = msg;

    switch (type) {
      case 'day':
        yield { tagName: 'h4', children: [TextTransform(text)] };
        break;
      case 'text':
        let t = TextTransform(text);

        yield { tagName: 'p', children: [t] };
        break;
      case 'photo':
        let out = join('static/img', basename(link));
        Copy(join(InputDir(), link), out);

        const { width, height, format } = Identify(out);

        yield { tagName: 'img', attributes: { src: out, class: format == 'landscape' ? 'landscape' : 'bywidth', onclick: 'ShowPhoto(this);' } };
        break;
    }
  }
}

export function OutputBlog(gen = GenerateBlog(GetMessages(document.querySelector('.history')))) {
  return {
    tagName: '?xml',
    attributes: {
      version: '1.0',
      encoding: 'utf-8'
    },
    children: [
      {
        tagName: '!DOCTYPE html'
      },
      {
        tagName: 'html',
        children: [
          {
            tagName: 'head',
            children: [
              {
                tagName: 'title',
                children: ['webakeit']
              },
              {
                tagName: 'link',
                attributes: {
                  href: 'static/fonts/LibreCaslonText-Regular.css',
                  rel: 'stylesheet'
                }
              },
              {
                tagName: 'link',
                attributes: {
                  href: 'static/css/webakeit.css',
                  rel: 'stylesheet'
                }
              },

              {
                tagName: 'script',
                attributes: {
                  type: 'module',
                  src: './webakeit.js'
                },
                children: ['']
              }
            ]
          },
          {
            tagName: 'body',
            attributes: {
              class: 'center'
            },
            children: [
              {
                tagName: 'div',
                attributes: {
                  'data-name': 'blog',
                  class: 'blog'
                },
                children: [
                  {
                    tagName: 'a',
                    attributes: { href: 'index.html' },
                    children: [
                      {
                        tagName: 'img',
                        attributes: {
                          class: 'close',
                          src: 'static/img/kaefer1-klein.png',
                          border: '0'
                        }
                      }
                    ]
                  },
                  ...gen,
                  {
                    tagName: 'div',
                    attributes: { class: 'back' },
                    children: [
                      {
                        tagName: 'a',
                        attributes: { href: 'index.html', class: 'back' },
                        children: [
                          {
                            tagName: 'img',
                            attributes: {
                              class: 'back',
                              src: 'static/img/kaefer2-klein.png',
                              border: '0'
                            }
                          },

                          'Zurück'
                        ]
                      }
                    ]
                  }
                ]
              }
            ]
          }
        ]
      }
    ]
  };
}

console.log('scriptArgs', scriptArgs);
console.log('__filename', __filename);

if(__filename.match(scriptArgs[0])) {
  main();
}

function main() {
  let file = InputFile();
  console.log('file', file);

  const p = new Parser();
  globalThis.document = p.parseFromString(readFileSync(file));
  console.log('document', document);

  let e = document.querySelector('.history');
  Object.assign(globalThis, { file, p, e, toXML, InputDir, GetMessages, GetMessage, GenerateBlog, OutputBlog, Copy, TextTransform, GetClasses });

  try {
    console.log('document.querySelector(".history")', e);
    const msgs = [...GetMessages(e)];
    console.log('msgs', msgs);

    const gen = GenerateBlog(msgs);

    let a = [...gen];
    console.log('a', a);

    remove('blog.html');
    writeFileSync('blog.html', toXML(OutputBlog(a)));
  } catch(e) {
    console.log('exception', e);
    startInteractive();
  }
}
