import { PQconnectStart, PQerrorMessage, PQexec, PQgetResult } from './psql.js';
function main(...args) {
  let pq = PQconnectStart('User ID=roman;Password=r4eHuJ;Host=localhost;Port=5432;Database=roman;Pooling=true;Min Pool Size=0;Max Pool Size=100;Connection Lifetime=0;');
  console.log('pq', pq);

  let result = PQexec(pq, 'SELECT * FROM test');
  console.log('result', result);
  result = PQgetResult(pq);
  console.log('result', result);

  let error = PQerrorMessage(pq);
  console.log('error', error);
}

main(...scriptArgs.slice(1));