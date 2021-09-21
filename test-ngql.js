import gql from './lib/nanographql.js';

var query = gql`
  query ($name: String!) {
    movie(name: $name) {
      releaseDate
    }
  }
`;

try {
  var res = await fetch('/query', {
    body: query({ name: 'Back to the Future' }),
    method: 'POST'
  });
  var json = res.json();
  console.log(json);
} catch(err) {
  console.error(err);
}
