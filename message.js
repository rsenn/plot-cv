import Util from './lib/util.js';

export class Message {
  static SENDER_ID = 0x2609;
  static RECIPIENT_ID = 0x2690;
  static TYPE_ID = 0x2606; //0xfffc;

  constructor(msg, origin, recipient, type) {
    let obj = this instanceof Message ? this : Object.setPrototypeOf({}, Message.prototype);

    if(typeof msg != 'string') {
      msg = Util.tryCatch(
        (msg) => JSON.stringify(msg),
        (json, msg) => json,
        (err, msg) => msg,
        msg
      );
    } else {
      const fields = {
        origin: getPrefix(Message.SENDER_ID),
        recipient: getPrefix(Message.RECIPIENT_ID),
        type: getPrefix(Message.TYPE_ID)
      };

      origin = origin || fields.origin;
      recipient = recipient || fields.recipient;
      type = type || fields.type;
    }

    if(origin) obj.origin = origin;
    if(recipient) obj.recipient = recipient;
    if(type) obj.type = type;

    function getPrefix(code) {
      let r, i;
      if(msg.codePointAt(0) == code) {
        i = msg.lastIndexOf(String.fromCodePoint(code));
        r = msg.substring(1, i);
        msg = msg.substring(i + 1);
      }
      return r;
    }
    msg = decodeURIComponent(msg);

    obj.body = msg;

    //console.log('Message', obj);
    return obj;
  }

  get data() {
    const { body, recipient, origin, type } = this;
    let r = encodeURIComponent(body);
    if(type) r = insertField(type, Message.TYPE_ID);
    if(recipient) r = insertField(recipient, Message.RECIPIENT_ID);
    if(origin) r = insertField(origin, Message.SENDER_ID);

    function insertField(str, code) {
      return String.fromCodePoint(code) + str + String.fromCodePoint(code) + r;
    }
    return r;
  }

  get json() {
    const { body } = this;
    let r = Util.tryCatch(
      () => JSON.parse(body),
      (obj) => obj,
      () => null
    );

    return r;
  }

  [Symbol.for('nodejs.util.inspect.custom')]() {
    const { body, recipient, origin, type } = this;
    let ret = {};

    if(this.json) ret.body = this.json;
    else ret.body = body;
    if(recipient) ret.recipient = recipient;
    if(origin) ret.origin = origin;
    if(type) ret.origin = type;
    return ret;
  }
  [Symbol.toStringTag]() {
    return this[Symbol.for('nodejs.util.inspect.custom')]();
  }
}

export default Message;
