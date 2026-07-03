import { Parser, Serializer, Node } from './lib/dom.js';
import { writeFileSync, existsSync, constants, copyFileSync, closeSync, openSync, readSync, statSync, writeSync, unlinkSync } from 'fs';
import { Console } from 'console';
import { dirname, join, basename, relative } from 'path';
import { glob, GLOB_TILDE, define, nonenumerable, mapWrapper } from './lib/util.js';
import htmlentities from './lib/htmlentities.js';
import { inspect } from './lib/inspect.js';
import { spawnSync, spawn } from 'child_process';
import { popen } from 'std';
import extendGenerator from 'extendGenerator';

extendGenerator();

const { DEBUG } = process.env;

const Message2Element = mapWrapper(new WeakMap());

const ExcludeList = [955, 1022];

const PhotoProperties = {
  [808]: { rotate: -90 },
  [877]: { rotate: 180 },
  [878]: { rotate: -90 },
};

export function toXML(e) {
  return new Serializer().serializeToString(e);
}

export function InputFileName() {
  return process.argv[2] || glob('~/Downloads/Telegram Desktop/ChatExport*/*.html', GLOB_TILDE).reverse()[0];
}

export function InputDir(f = InputFileName()) {
  return dirname(f);
}

export function ExecAndGetOutput(prog, ...params) {
  const f = popen(prog + params.map(p => ` '${p}'`).join(''), 'r');
  const s= f.readAsString();
  f.close();
  return s;

  /*const { stdout } = spawnSync(prog, params, { stdio: ['inherit', 'pipe', 'pipe'], encoding: 'utf-8' });
  return stdout;*/
}

export function Identify(f) {
  const s = ExecAndGetOutput('identify', f);

  if(typeof s != 'string') throw new Error(`Identify(): s is not a string ${inspect(s)}`);

  const [width, height] = s
    .match(/\d+x\d+/g)[0]
    .split('x')
    .map(n => +n);

  return {
    width,
    height,
    format: width > height ? 'landscape' : width < height ? 'portrait' : 'square',
  };
}

export function Copy(src, dest) {
  try {
    unlinkSync(dest);
  } catch(e) {}

  if(DEBUG) console.log(`Copying '${relative(src)}' to '${dest}'.`);

  return copyFileSync(src, dest, constants.COPYFILE_FICLONE);
}

export function Convert(src, dest, options = []) {
  try {
    unlinkSync(dest);
  } catch(e) {}

  if(DEBUG) console.log(`Converting '${relative(src)}' to '${dest}'.`);

  const cmd = ['convert-im6', src, ...options, dest];
  const child = spawn(cmd);

  child.wait();

  if(!(child.exited && child.exitcode == 0)) console.error(`Command '${cmd.join(' ')}' failed.`);
}

export function TextTransform(s) {
  return s
    .split(/\s+/g)
    .map(w => htmlentities.decode(w))
    .map(w => htmlentities.encode(w))
    .join(' ');

  let out = '';

  for(let i = 0; i < s.length; i++) {
    const c = s.codePointAt(i);

    out += c > 127 ? `&#x${c.toString(16).toUpperCase().padStart(4, '0')};` : s[i];
  }

  return out;
}

export function* Iterate(a, prog) {
  const { length } = a;

  if(prog) prog(0, length);

  for(let i = 0; i < length; i++) {
    yield a[i];

    if(prog) prog(i + 1, length);
  }
}

export function GetClasses(e) {
  return (e.getAttribute('class') || '').split(/\s+/g);

  return [...(Iterate(e.classList) ?? [])];
}

export function ProcessText(e, date) {
  let child,
    text = '';
  const r = [],
    it = Iterate(e.children);

  while((child = it.next().value)) {
    if(isListItem()) {
      Yield();

      do {
        if(child.tagName == 'br') {
          text += '\n';
          child = it.next().value;

          if(!isListItem()) break;
        }

        AddText();
      } while((child = it.next().value));

      Yield();

      continue;
    }

    AddText();
  }

  Yield();

  function isListItem(s = child.textContent) {
    return /\u2022/.test(s);
  }

  function AddText(s = child.textContent) {
    text += (text != '' ? ' ' : '') + s;
  }

  function Yield() {
    if(text != '') {
      r.push({ type: 'text', text, date });
      text = '';
    }
  }

  return r;
}

export function* GetMessage(msg) {
  let body = msg.querySelector('.body');

  const tmp = body.querySelector('.forwarded.body');

  if(tmp) body = tmp;

  if(body.querySelector('.reply_to')) return;

  if(body) {
    let date;

    if(GetClasses(body).indexOf('details') != -1) {
      const text = body.innerText;
      let [day, month, year] = text.split(/\s+/g);

      month =
        {
          January: 'Januar',
          February: 'Februar',
          March: 'März',
          May: 'Mai',
          June: 'Juni',
          July: 'Juli',
          October: 'Oktober',
          December: 'Dezember',
        }[month] ?? month;

      if(!/^Channel/.test(text)) yield { type: 'day', text: [day + '.', month, year].join(' ') };
    }

    if(body.children) {
      for(const e of [...Iterate(body.children)]) {
        if(e.nodeType != e.ELEMENT_NODE) continue;

        const cl = GetClasses(e);

        if(cl.indexOf('date') != -1) {
          const str = e.getAttribute('title');
          const [d, m, y, hr, min, sec] = [...str.matchAll(/\d+/g)].map(([a]) => a);

          date = new Date(`${y}-${m}-${d}T${hr}:${min}:${sec}`);
          continue;
        }

        if(cl.indexOf('text') != -1) {
          const r = ProcessText(e, date);

          if(r.length > 1) {
            console.log('e', e.parentNode.parentNode);
            console.log('r', r);
          }

          yield* r;
        } else if(cl.indexOf('media_wrap') != -1 || cl.indexOf('userpic_wrap') != -1) {
          for(const f of e.querySelectorAll('a')) {
            const img = f.children[0];
            const thumb = img.getAttribute('src');
            const link = f.getAttribute('href');

            yield { type: 'photo', link, thumb, date };
          }
        }
      }
    } else if(body.innerText) {
      yield { type: 'text', text: body.innerText };
    }
  }
}

export function* LogGenerator(gen, fn = arg => console.log(arg)) {
  let i = 0;

  for(const elem of gen) {
    fn(elem, i++);
    yield elem;
  }
}

export function GetId(e) {
  const [m] = e.getAttribute('id')?.match(/\d+/g) ?? [];

  if(m) return +m;
}

export function* GetMessages(hist) {
  if(!hist) throw new Error('');

  const { err } = std;

  function Progress(i, n) {
    err.puts(`\rEintrag ${i}/${n} (${((i * 100) / n).toFixed(1)}%)\x1b[K` + (i == n ? '\n' : ''));
    err.flush();
  }

  for(const e of Iterate(hist, Progress)) {
    const id = GetId(e);

    if(ExcludeList.includes(id)) continue;

    try {
      yield* GetMessage(e) /*.map(m => (Message2Element(m, e), m))*/;
    } catch(error) {
      console.log('GetMessages', e);
      throw error;
    }
  }
}

export function AccumulateDays(messages) {
  let days = {},
    day;

  for(const msg of messages) {
    if(msg.type == 'day') {
      day = msg.text;
      continue;
    }

    days[day] ??= [];
    days[day].push(msg);
  }

  return Object.entries(days);
}

export function* GenerateDays(days) {
  for(const [day, messages] of days) {
    yield { type: 'day', text: day };
    yield* messages;
  }
}

function FormatDate(d) {
  return d
    ?.toJSON()
    ?.replaceAll(/([TZ]|\.\d\d\d\s*)/g, ' ')
    ?.trimEnd();
}

export function* GenerateBlog(messages /* = GetMessages()*/) {
  try {
    let prevMsg,
      i = 0;

    for(const msg of messages) {
      const { type, text, link, thumb, date } = msg;

      if(type == 'day' && prevMsg && prevMsg.type == 'day') continue;
      prevMsg = msg;

      const o = { type };

      if(date) o.date = FormatDate(date);
      if(text) o.text = text;
      if(link) o.link = basename(link);

      if(DEBUG) console.log(`Generate blog message #${i}`, inspect(o, { colors: true, compact: true, breakLength: Infinity }));

      switch (type) {
        case 'day':
          const s = TextTransform(text);
          const [d, m, y] = text.split(/[. \s]+/g);

          yield { tagName: 'a', attributes: { name: `${y}${m}${d}` }, children: [''] };
          yield { tagName: 'h4', children: [TextTransform(text)] };
          break;

        case 'text':
          const lines = text.split(/[\n\r]+/g).map(l => TextTransform(l));

          const children = lines.reduce((a, l) => (Array.isArray(a) ? [...a, { tagName: 'br' }, l] : [l]), undefined);

          yield { tagName: 'p', children };
          break;

        case 'photo':
          const id = +link.replace(/.*photo_(\d+)@.*/g, '$1');
          const input = [link, thumb].map(p => join(InputDir(), p));
          const output = [link, thumb].map(p => join('static/photos', basename(p)));

          const isNew = output.every(p => !existsSync(p));

          if(isNew) console.log(`New photo '${output[0]}'`);

          if(id in PhotoProperties && PhotoProperties[id].rotate) {
            const { rotate } = PhotoProperties[id];

            if(rotate % 180 >= 90) format = { landscape: 'portrait', portrait: 'landscape' }[format];

            const options = ['-rotate', rotate + '', '-quality', '99'];

            Convert(input[0], output[0], options);
            Convert(input[1], output[1], options);
          } else {
            Copy(input[0], output[0]);
            Copy(input[1], output[1]);
          }

          const { width, height, format } = Identify(output[1]);

          yield {
            tagName: 'img',
            attributes: {
              'data-file': output[0],
              'data-src': output[1],
              class: format == 'landscape' ? 'landscape' : 'bywidth',
              onclick: 'ShowPhoto(this);',
              width,
              height,
            },
          };

          break;
      }

      ++i;
    }
  } catch(e) {
    console.log('GenerateBlog', e);
    throw e;
  }
}

export function OutputBlog(iterator) {
  return {
    tagName: '?xml',
    attributes: {
      version: '1.0',
      encoding: 'utf-8',
    },
    children: [
      {
        tagName: '!DOCTYPE html',
      },
      {
        tagName: 'html',
        attributes: {},
        children: [
          {
            tagName: 'head',
            attributes: {},
            children: [
              { tagName: 'title', attributes: {}, children: ['webakeit - Blog'] },
              { tagName: 'link', attributes: { href: 'static/fonts/LibreCaslonText-Regular.css', rel: 'stylesheet' } },
              { tagName: 'link', attributes: { href: 'static/css/webakeit.css', rel: 'stylesheet' } },
              { tagName: 'link', attributes: { rel: 'icon', type: 'image/x-icon', href: 'favicon.ico' } },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:title',
                  content: 'webakeit',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:locale',
                  content: 'de_CH',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:logo',
                  content: 'http://www.webakeit.ch/static/img/globi-bg-preview.svg',
                  size: '2000x2050',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:description',
                  content: 'Brot aus dem Holzbackofen ab Herbst 2025',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:type',
                  content: 'website',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:url',
                  content: 'https://www.webakeit.ch/',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:image',
                  content: 'http://www.webakeit.ch/static/img/globi-bg-preview.jpg',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:image:secure_url',
                  content: 'https://www.webakeit.ch/static/img/globi-bg-preview.jpg',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:image:type',
                  content: 'image/jpeg',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:image:alt',
                  content: 'Brot aus dem Holzbackofen ab Herbst 2025',
                },
              },
              {
                tagName: 'meta',
                attributes: {
                  property: 'og:image',
                  content: 'https://www.webakeit.ch/static/img/globi-bg-preview300.jpg',
                },
              },
              { tagName: 'script', attributes: { type: 'module', src: './webakeit.js' }, children: ['\n'] },
              {
                tagName: 'meta',
                attributes: {
                  name: 'description',
                  content: 'Webakeit backt Dein Brot im historischen Holzofen in Guggisberg von 1771. Die Brote werden via Versand, an Märkten oder vor Ort angeboten.',
                },
              },
            ],
          },
          {
            tagName: 'body',
            attributes: {},
            children: [
              {
                tagName: 'div',
                attributes: { 'data-name': 'blog', class: 'blog' },
                children: [
                  { tagName: 'div', attributes: { id: 'header' }, children: [''] },
                  {
                    tagName: 'div',
                    attributes: { id: 'content' },
                    children: [
                      {
                        tagName: 'div',
                        attributes: { id: 'text' },
                        children: [
                          ...[...iterator].reduce((a, e) => {
                            if(a.length && e.tagName != 'a') {
                              a[a.length - 1].children.push(e);
                            } else {
                              a.push({ tagName: 'div', children: [e] });
                            }
                            return a;
                          }, []),
                          {
                            tagName: 'div',
                            attributes: {
                              class: 'back-container',
                            },
                            children: [
                              {
                                tagName: 'a',
                                attributes: {
                                  href: 'index.html',
                                },
                                children: [
                                  'Zur&uuml;ck',
                                  {
                                    tagName: 'img',
                                    attributes: { class: 'back-img', src: 'static/img/globi-fg-kaefer1.svg' },
                                  },
                                ],
                              },
                            ],
                          },
                        ],
                      },
                    ],
                  },
                ],
              },
            ],
          },
        ],
      },
    ],
  };
}

main(...scriptArgs.slice(1));

function main(...args) {
  const { stdout, stderr } = process;

  globalThis.console = new Console({ stdout, stderr, inspectOptions: { depth: 4, compact: false } });

  const file = InputFileName();

  const document = new Parser().parseFromFile(file);

  const historyElement = document.querySelector('.history');

  Object.assign(globalThis, {
    file,
    document,
    historyElement,
    toXML,
    InputDir,
    GetId,
    GetMessages,
    GetMessage,
    GenerateBlog,
    OutputBlog,
    Copy,
    TextTransform,
    GetClasses,
    AccumulateDays,
    GenerateDays,
    Message2Element,
  });

  try {
    //const gen = GetMessages(historyElement.children);
    //if(DEBUG) gen = LogGenerator(gen, (msg, i) => console.log(`Getting message #${i}`, inspect(msg, { compact: true, breakLength: Infinity, colors: true })));

    const messages = [...GetMessages(historyElement.children)];
    const days = AccumulateDays(messages).reverse();

    Object.assign(globalThis, { messages, days });

    if(existsSync('blog.html')) unlinkSync('blog.html');

    const iterator = GenerateBlog(GenerateDays(days));

    const output = toXML(OutputBlog([...iterator])).replaceAll(/([^>"])\b(\w+:\/\/[^\s<"]+)\b([^<"])/g, (match, p1, p2, p3) => {
      let text = p2;

      if(/\.(pdf|docx?)$/i.test(p2))
        if(p2.length > 30) {
          text = text.replace(/.*\//g, '');
          p1 += '<br />';
          p3 = '<br />' + p3;
        }

      return `${p1}<a href="${p2}">${text}</a>${p3}`;
    });

    writeFileSync('blog.html', output);
  } catch(error) {
    console.log('exception', error);
    throw error;
  }
}
