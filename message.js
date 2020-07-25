export class Message {
  constructor(msg, sender, recipient) {
    let obj = this instanceof Message ? this : Object.setPrototypeOf({}, Message.prototype);

    sender = getPrefix(Message.SENDER_ID) || sender;
    recipient = getPrefix(Message.RECIPIENT_ID) || recipient;

    if(sender) obj.sender = sender;
    if(recipient) obj.recipient = recipient;

    function getPrefix(code) {
      let r, i;
      if(msg.codePointAt(0) == code) {
        i = msg.indexOf(String.fromCodePoint(code), 1);
        r = msg.substring(1, i);
        msg = msg.substring(i + 1);
      }
      return r;
    }

    obj.msg = msg;
    //console.log('Message', obj);

    return obj;
  }

  static SENDER_ID = 0x27bf;
  static RECIPIENT_ID = 0x26aa;

  get data() {
    const { msg, recipient, sender } = this;
    let r = msg;

    if(recipient) r = insertField(recipient, Message.RECIPIENT_ID);
    if(sender) r = insertField(sender, Message.SENDER_ID);

    function insertField(str, code) {
      return String.fromCodePoint(code) + str + String.fromCodePoint(code) + r;
    }

    return r;
  }
  get body() {
    return this.msg;
  }
}

export default Message;
