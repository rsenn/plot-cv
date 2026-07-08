/*
 * http-client.js
 * --------------
 * Synchronous HTTPS client abstraction with two interchangeable backends:
 *
 *   - 'curl' : shells out to /usr/bin/curl via child_process
 *   - 'lws'  : native HTTPS client via qjs-lws (LWSContext)
 *
 * Both backends share the same request/response shape, so calling code
 * does not need to know which is active.
 *
 * Supports mutual TLS (client cert + private key) and custom CA bundles
 * via PEM files on both backends.
 *
 * Usage:
 *
 *   import { HttpClient } from './http-client.js';
 *
 *   const c = new HttpClient({
 *     backend: 'lws',                   // or 'curl'
 *     cert:    '/etc/ssl/client.pem',   // optional client cert
 *     key:     '/etc/ssl/client.key',   // optional client private key
 *     ca:      '/etc/ssl/ca.pem',       // optional CA bundle
 *     verify:  true,                    // verify server cert (default true)
 *     timeout: 30000,                   // request timeout in ms
 *   });
 *
 *   const r = c.request({
 *     method:  'POST',
 *     url:     'https://api.example.com/v1/foo',
 *     headers: { 'Content-Type': 'application/json' },
 *     body:    JSON.stringify({ hello: 'world' }),
 *   });
 *
 *   // r === { status: 200, headers: { ... }, body: '...' }
 */

import * as fs from 'fs';
import * as path from 'path';
import { execSync } from 'child_process';
import { LWSContext } from 'lws.so';

//----------------------------------------------------------------------------
// URL parsing — small and deliberate; the WHATWG URL is overkill here.
//----------------------------------------------------------------------------
function parseUrl(u) {
  const m = /^(https?):\/\/([^\/:?#]+)(?::(\d+))?(\/[^?#]*)?(\?[^#]*)?/.exec(u);
  if(!m) throw new Error('invalid URL: ' + u);
  const scheme = m[1];
  return {
    scheme,
    host: m[2],
    port: m[3] ? parseInt(m[3], 10) : scheme === 'https' ? 443 : 80,
    path: (m[4] || '/') + (m[5] || ''),
    secure: scheme === 'https',
  };
}

//----------------------------------------------------------------------------
// Build the response object shape — both backends produce this.
//----------------------------------------------------------------------------
function makeResponse(status, headers, body) {
  // Normalize header names to lowercase for predictable access.
  const norm = {};
  for(const k of Object.keys(headers || {}))
    norm[k.toLowerCase()] = headers[k];
  return { status, headers: norm, body };
}

//----------------------------------------------------------------------------
// HttpClient — the abstraction
//----------------------------------------------------------------------------
export class HttpClient {
  constructor(opts = {}) {
    this.backend = opts.backend || 'curl';
    this.cert = opts.cert || null;
    this.key = opts.key || null;
    this.ca = opts.ca || null;
    this.verify = opts.verify !== false;
    this.timeout = opts.timeout || 30000;

    // Validate cert/key files exist if specified (cheap sanity check)
    for(const [name, p] of [
      ['cert', this.cert],
      ['key', this.key],
      ['ca', this.ca],
    ]) {
      if(p && !fs.existsSync(p))
        throw new Error(`${name} PEM file not found: ${p}`);
    }

    if(this.backend === 'lws') this._initLws();
  }

  request(req) {
    if(!req.url) throw new Error('request.url is required');

    if(this.backend === 'curl') return this._curlRequest(req);
    if(this.backend === 'lws') return this._lwsRequest(req);
    throw new Error('unknown backend: ' + this.backend);
  }

  // Close any backend-held resources (LWS context, etc.)
  close() {
    if(this._ctx && typeof this._ctx.destroy === 'function')
      this._ctx.destroy();
    this._ctx = null;
  }

  //----------------------------------------------------------------------
  // CURL BACKEND
  //----------------------------------------------------------------------
  _curlRequest(req) {
    const headerFile = `/tmp/qjs-http-${Date.now()}-h.txt`;
    const bodyFile = req.body ? `/tmp/qjs-http-${Date.now()}-b.bin` : null;

    if(bodyFile) {
      // Write the body to a temp file so we don't have to shell-escape it.
      fs.writeFileSync(bodyFile, req.body);
    }

    const args = [
      'curl',
      '-sS',
      '-o',
      '-', // body to stdout
      '-D',
      shellQuote(headerFile), // headers to file
      '--max-time',
      String(Math.ceil(this.timeout / 1000)),
      '-X',
      shellQuote(req.method || 'GET'),
    ];

    if(!this.verify) args.push('-k');

    if(this.ca) args.push('--cacert', shellQuote(this.ca));
    if(this.cert) args.push('--cert', shellQuote(this.cert));
    if(this.key) args.push('--key', shellQuote(this.key));

    for(const [name, value] of Object.entries(req.headers || {}))
      args.push('-H', shellQuote(`${name}: ${value}`));

    if(bodyFile) args.push('--data-binary', shellQuote('@' + bodyFile));

    args.push(shellQuote(req.url));

    let stdout = '';
    let runError = null;
    try {
      const r = execSync(args.join(' '));
      stdout = r.stdout || '';
    } catch(e) {
      runError = e;
    }

    // Parse headers file
    let status = 0;
    let respHeaders = {};
    if(fs.existsSync(headerFile)) {
      const hraw = fs.readFileSync(headerFile, 'utf8');
      ({ status, headers: respHeaders } = parseHttpHeaders(hraw));
      try {
        fs.unlinkSync(headerFile);
      } catch(_) {}
    }
    if(bodyFile) {
      try {
        fs.unlinkSync(bodyFile);
      } catch(_) {}
    }

    if(runError && status === 0)
      throw new Error('curl failed: ' + runError.message);

    return makeResponse(status, respHeaders, stdout);
  }

  //----------------------------------------------------------------------
  // LWS BACKEND
  //
  // NOTE: the following uses what I believe to be the qjs-lws client API
  // surface. Method names you may need to verify against your binding:
  //
  //   - new LWSContext({ ... }) accepting these SSL-related option keys:
  //         clientSslCert, clientSslKey, clientSslCa, sslVerify
  //   - ctx.connect({ ... }) -> wsi, OR ctx.clientConnect({ ... }),
  //         OR new LWSClient(ctx, { ... })
  //   - ctx.service(ms) to pump events for up to `ms` ms
  //   - wsi.write(data) to send body bytes (during APPEND/WRITEABLE)
  //
  // The reason-code constants below are from libwebsockets and *should*
  // be exposed as integers on the namespace; I've inlined the numeric
  // values from upstream lws as a fallback.
  //----------------------------------------------------------------------
  _initLws() {
    // One context per HttpClient — reused across requests.
    const ctxOpts = {
      // No listen port — this is a client-only context.
      // (Some lws bindings require port: -1 or omitting the field.)
      vhostName: 'client',
      protocols: [{ name: 'http', callback: () => 0 }], // placeholder

      // SSL/TLS material — adjust key names to your binding if needed.
      clientSslCert: this.cert || undefined,
      clientSslKey: this.key || undefined,
      clientSslCa: this.ca || undefined,
      sslVerify: this.verify,
    };
    this._ctx = new LWSContext(ctxOpts);
  }

  _lwsRequest(req) {
    const u = parseUrl(req.url);

    // libwebsockets HTTP-client callback reason codes (numeric values
    // are from upstream lws's libwebsockets.h):
    const ESTABLISHED_CLIENT_HTTP = 44;
    const CLIENT_HTTP_WRITEABLE = 57; // for sending body
    const RECEIVE_CLIENT_HTTP_READ = 48;
    const COMPLETED_CLIENT_HTTP = 47;
    const CLIENT_CONNECTION_ERROR = 1;
    const CLIENT_APPEND_HANDSHAKE_HEADER = 49;

    let status = 0;
    let respHeaders = {};
    const chunks = [];
    let done = false;
    let errorMsg = null;
    let writeOffset = 0;
    const bodyBytes = req.body
      ? typeof req.body === 'string'
        ? new TextEncoder().encode(req.body)
        : new Uint8Array(req.body)
      : null;

    const headersToSend = Object.assign({}, req.headers || {});
    if(bodyBytes && !findHeaderCI(headersToSend, 'content-length'))
      headersToSend['Content-Length'] = String(bodyBytes.length);

    const connectOpts = {
      address: u.host,
      port: u.port,
      path: u.path,
      host: u.host,
      origin: u.host,
      method: req.method || 'GET',
      ssl: u.secure,
      protocol: 'http',

      callback: (wsi, reason, user, data) => {
        switch (reason) {
          case CLIENT_CONNECTION_ERROR:
            errorMsg = typeof data === 'string' ? data : 'connection error';
            done = true;
            return -1;

          case CLIENT_APPEND_HANDSHAKE_HEADER:
            // Some lws bindings let you add headers here; many bindings
            // accept them up front via connectOpts.headers instead. We
            // populate both for safety.
            if(wsi && typeof wsi.appendHeader === 'function') {
              for(const [k, v] of Object.entries(headersToSend))
                wsi.appendHeader(k, v);
            }
            return 0;

          case ESTABLISHED_CLIENT_HTTP:
            // `data` (if provided by your binding) carries response
            // status + headers. Otherwise fetch via wsi accessors.
            if(data && typeof data === 'object') {
              status = data.status || data.statusCode || 0;
              respHeaders = data.headers || {};
            } else if(wsi) {
              if(typeof wsi.responseStatus === 'function')
                status = wsi.responseStatus();
              if(typeof wsi.responseHeaders === 'function')
                respHeaders = wsi.responseHeaders();
            }
            // If we have a body to send, ask to be made writeable.
            if(
              bodyBytes &&
              wsi &&
              typeof wsi.callbackOnWritable === 'function'
            )
              wsi.callbackOnWritable();
            return 0;

          case CLIENT_HTTP_WRITEABLE:
            if(bodyBytes && writeOffset < bodyBytes.length) {
              const chunk = bodyBytes.subarray(writeOffset);
              const n = wsi.write(chunk);
              writeOffset += typeof n === 'number' ? n : chunk.length;
              if(
                writeOffset < bodyBytes.length &&
                typeof wsi.callbackOnWritable === 'function'
              )
                wsi.callbackOnWritable();
            }
            return 0;

          case RECEIVE_CLIENT_HTTP_READ:
            if(data) {
              if(typeof data === 'string') chunks.push(data);
              else chunks.push(new Uint8Array(data));
            }
            return 0;

          case COMPLETED_CLIENT_HTTP:
            done = true;
            return 0;
        }
        return 0;
      },

      // Pre-emptively pass headers, in case the binding prefers them here:
      headers: headersToSend,
    };

    // Initiate the client connection. The exact method name depends on
    // the qjs-lws build — try the common ones in order.
    let wsi;
    if(typeof this._ctx.connect === 'function')
      wsi = this._ctx.connect(connectOpts);
    else if(typeof this._ctx.clientConnect === 'function')
      wsi = this._ctx.clientConnect(connectOpts);
    else throw new Error('qjs-lws: no client-connect method on LWSContext');

    // Pump the event loop until the request finishes or times out.
    const deadline = Date.now() + this.timeout;
    while(!done && Date.now() < deadline) {
      if(typeof this._ctx.service === 'function') {
        // Service for up to 100 ms per spin; lws returns when an event
        // fires or the timeout elapses.
        this._ctx.service(100);
      } else {
        // Fallback: assume the binding fires callbacks asynchronously
        // and yield. (You'll know if this branch is wrong.)
        break;
      }
    }

    if(errorMsg) throw new Error('HTTPS request failed: ' + errorMsg);
    if(!done)
      throw new Error('HTTPS request timed out after ' + this.timeout + 'ms');

    const bodyStr = chunksToString(chunks);
    return makeResponse(status, respHeaders, bodyStr);
  }
}

//----------------------------------------------------------------------------
// Header parsing — for the curl backend's `-D headers.txt` output.
//----------------------------------------------------------------------------
function parseHttpHeaders(raw) {
  // curl writes one status line + headers per response, separated by blank
  // lines. With redirects there may be multiple blocks; the *last* block
  // is the final response.
  const blocks = raw.split(/\r?\n\r?\n/).filter(b => b.trim().length > 0);
  const last = blocks[blocks.length - 1];
  const lines = last.split(/\r?\n/);

  let status = 0;
  const headers = {};
  const m = /^HTTP\/[\d.]+\s+(\d+)/.exec(lines[0] || '');
  if(m) status = parseInt(m[1], 10);

  for(let i = 1; i < lines.length; i++) {
    const idx = lines[i].indexOf(':');
    if(idx < 0) continue;
    const k = lines[i].slice(0, idx).trim();
    const v = lines[i].slice(idx + 1).trim();
    headers[k] = v;
  }
  return { status, headers };
}

//----------------------------------------------------------------------------
// Small utilities
//----------------------------------------------------------------------------
function shellQuote(s) {
  // Single-quote, escaping embedded single quotes.
  return "'" + String(s).replace(/'/g, "'\\''") + "'";
}

function findHeaderCI(headers, name) {
  const target = name.toLowerCase();
  for(const k of Object.keys(headers))
    if(k.toLowerCase() === target) return k;
  return null;
}

function chunksToString(chunks) {
  if(chunks.length === 0) return '';
  if(typeof chunks[0] === 'string') return chunks.join('');

  // Concatenate Uint8Arrays then decode as UTF-8.
  let total = 0;
  for(const c of chunks) total += c.length;
  const out = new Uint8Array(total);
  let off = 0;
  for(const c of chunks) {
    out.set(c, off);
    off += c.length;
  }
  return new TextDecoder().decode(out);
}

//----------------------------------------------------------------------------
// Convenience wrappers — feel free to extend
//----------------------------------------------------------------------------
HttpClient.prototype.get = function(url, headers) {
  return this.request({ method: 'GET', url, headers });
};

HttpClient.prototype.post = function(url, body, headers) {
  return this.request({ method: 'POST', url, body, headers });
};

HttpClient.prototype.postJson = function(url, obj, headers) {
  const h = Object.assign(
    { 'Content-Type': 'application/json' },
    headers || {},
  );
  return this.request({
    method: 'POST',
    url,
    headers: h,
    body: JSON.stringify(obj),
  });
};

HttpClient.prototype.postForm = function(url, fields, headers) {
  const body = Object.keys(fields)
    .map(k => encodeURIComponent(k) + '=' + encodeURIComponent(fields[k]))
    .join('&');
  const h = Object.assign(
    { 'Content-Type': 'application/x-www-form-urlencoded' },
    headers || {},
  );
  return this.request({ method: 'POST', url, headers: h, body });
};
