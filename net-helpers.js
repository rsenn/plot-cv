import { fetch } from 'fetch';
import { define } from 'util';

export async function httpGet(url, opts = {}) {
  const resp = await fetch(url, { tls: { key: 'localhost.key', cert: 'localhost.crt', /*ca: 'ca.crt',*/ rejectUnauthorized: false }, h2: false, ...opts });

  return await resp.text();
}
