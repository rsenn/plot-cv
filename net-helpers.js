import { fetch } from 'fetch';

export async function httpGet(url, opts = {}) {
  return await fetch(url, { tls: { key: 'localhost.key', cert: 'localhost.crt', ca: 'ca.crt', rejectUnauthorized: false }, h2: false, ...opts });
}
