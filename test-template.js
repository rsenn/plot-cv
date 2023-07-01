const name = 'blah';
console.log(`loading "${name}"...`);
const value = 42;
const word = `simple`;
const simple = `this is a ${word}    ${value * 3} template`;
const start = `${word} ${value * 3}`;

const pushHandler = async state => {
  await load(name);
};
