import fetch from 'isomorphic-fetch';
import gql from './lib/nanographql.js';
import { Console } from 'console';
let query = gql`
  query MyQuery {
    users {
      id
      email
      username
      last_seen
    }
  }
`;

async function main() {
  globalThis.console = new Console({
    stdout: process.stdout,
    stderr: process.stderr,
    inspectOptions: {
      colors: true,
      depth: Infinity,
      customInspect: true,
      compact: 2,
      multiline: true
    }
  });

  try {
    let body = query();
    console.log('body', body);
    let res = await fetch('http://wild-beauty.herokuapp.com/v1/graphql', {
      body,
      method: 'POST'
    });
    let json = await res.json();
    console.log('json', json);
  } catch(err) {
    console.error(err);
  }
}

main();