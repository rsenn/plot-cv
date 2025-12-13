import { LogLevel, SocketDebugClient } from './node-debugprotocol-client/dist/index.js';

async function main(...args) {
  // create a client instance
  const client = new SocketDebugClient({
    port: 6666,
    host: 'localhost',
    logLevel: LogLevel.Verbose,
    loggerName: 'My Debug Adapter Client'
  });

  // connect
  await client.connectAdapter();

  // initialize first
  let v = await client.variables({ variablesReference: 0 });

  console.log('variables:', v);

  // tell the debug adapter to attach to a debuggee which is already running somewhere
  // SpecificAttachArguments has to extend DebugProtocol.AttachRequestArguments
  await client.attach(
    /*<SpecificAttachArguments>*/ {
      // ...
    }
  );

  // set some breakpoints
  await client.setBreakpoints({
    breakpoints: [
      { line: 1337 },
      { line: 42 }
      // ...
    ],
    source: {
      path: 'test-renderer.js'
    }
  });

  // listen to events such as "stopped"
  const unsubscribable = client.onStopped(async stoppedEvent => {
    if(stoppedEvent.reason === 'breakpoint') {
      // we hit a breakpoint!

      // do some debugging

      // continue all threads
      await client.continue({ threadId: 0 });
    }
  });

  // send 'configuration done' (in some debuggers this will trigger 'continue' if attach was awaited)
  await client.configurationDone();

  // ...

  // Event subscriptions can be unsubscribed
  unsubscribable.unsubscribe();

  // disconnect the from adapter when done
  client.disconnectAdapter();
}

main(...scriptArgs.slice());