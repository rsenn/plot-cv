import { GPIO } from 'gpio';

function main(...args) {
  const gpio = new GPIO();
  console.log('gpio.initPin', gpio.initPin);

  gpio.initPin(0, GPIO.OUTPUT);
  gpio.setPin(0, 1);

  gpio.initPin(1, GPIO.INPUT);
  const value = gpio.getPin(1);
  console.log('Pin #1 value:', value);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(error);
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}