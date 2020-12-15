import Util from './lib/util.js';

export class Message {
  static SENDER_ID = 0x2609;
  static RECIPIENT_ID = 0x2690;
  static TYPE_ID = 0x2606; //0xfffc;

  constructor(...args) {
    //console.log('new Message(', ...args, ')');

    if(args.length == 1) {
      Object.assign(this, typeof args[0] == 'string' ? Message.decode(args[0]) : Util.filterOutMembers(args[0], v => v == undefined));
    } else {
      const [body, origin, recipient, type] = args;
      if(origin !== undefined) this.origin = origin;
      if(recipient !== undefined) this.recipient = recipient;
      if(type !== undefined) this.type = type;
      if(body !== undefined) this.body = body;
    }
    const { origin, recipient, type, body } = this;

    //console.log('new Message', Util.filterOutMembers({ origin, recipient, type, body }, v => v == undefined));
    //console.log('new Message',  Util.abbreviate(this.encode(),80));
  }

  static isId(char) {
    const { RECIPIENT_ID, SENDER_ID, TYPE_ID } = Message;
    const num = typeof char == 'string' ? char.codePointAt(0) : char;

    return [SENDER_ID, RECIPIENT_ID, TYPE_ID].some(code => code == num);
  }

  static decode(m) {
    if(Message.isId(m[0])) {
      const o = {
        origin: getPrefix(Message.SENDER_ID),
        recipient: getPrefix(Message.RECIPIENT_ID),
        type: getPrefix(Message.TYPE_ID)
      };

      function getPrefix(code) {
        let r, i;
        if(m.codePointAt(0) == code) {
          i = m.lastIndexOf(String.fromCodePoint(code));
          r = m.substring(1, i);
          m = m.substring(i + 1);
        }
        return r;
      }
      return o;
    }
    let o = {};
    let len;
    if(m[0] == ':') {
      len = m.indexOf(' ');
      if(len == -1) len = m.length;
      o.origin = m.substring(1, len);
      m = m.substring(len + 1);
    }
    len = m.indexOf(' ');
    if(len == -1) len = m.length;
    o.type = m.substring(0, len);
    m = m.substring(len + 1);

    len = m.indexOf(' ');
    if(len != -1) {
      o.recipient = m.substring(0, len);
      m = m.substring(len + 1);
    }

    if(m.length) o.body = decodeBody(m);

    return o;
  }

  get data() {
    // console.log(`Message data: `, this);
    let data = this.encode();
    //  console.log(`Message data: '${data}'`);
    return data;
  }

  encode(plain = true) {
    const { body, recipient, origin, type, isJSON } = this;
    //console.log('Message.encode',{body,recipient,origin, type});
    let r = encodeBody(body) || '';

    const prepend = plain ? str => (r = str + (r != '' ? ' ' : '') + r) : str => (r = str + r);

    const insertField = plain ? (str, code) => prepend((code == Message.SENDER_ID ? ':' : '') + str) : (str, code) => prepend((r = String.fromCodePoint(code) + str + String.fromCodePoint(code)));

    if(recipient) r = insertField(recipient, Message.RECIPIENT_ID);
    r = insertField(type || 'x', Message.TYPE_ID);
    if(origin) r = insertField(origin, Message.SENDER_ID);

    return r;
  }

  get json() {
    const { body } = this;
    let r = Util.tryCatch(() => JSON.parse(body),
      obj => obj,
      () => null
    );

    return r;
  }

  [Symbol.for('nodejs.util.inspect.custom')]() {
    return this[Symbol.toStringTag]();
  }
  [Symbol.toStringTag]() {
    const { origin, recipient, type, body } = this;
    return 'new Message', Util.filterOutMembers({ origin, recipient, type, body }, v => v == undefined);
  }
}

function decodeBody(str) {
  if(str[0] == ':') {
    //console.debug('decodeBody', str);
    return JSON.parse(unescape(str.substring(1)));
  }
  return str;
}

function encodeBody(body) {
  if(typeof body == 'string') return body;
  if(body == undefined) return undefined;

  //console.debug('encodeBody', JSON.stringify(body));
  return ':' + /*escape*/ JSON.stringify(body);
}
export default Message;
