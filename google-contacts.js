#!/usr/bin/env qjs
/*
 * google-contacts.js
 * ------------------
 * Retrieves all Google contacts sorted by last-modified time (descending),
 * so the most recently created/edited contact appears first.
 *
 * Useful when you remember roughly *when* you added a contact but not who.
 *
 * Architecture:
 *   - HTTPS calls go through the HttpClient abstraction (./http-client.js).
 *     Switch the BACKEND constant below between 'curl' and 'lws' freely.
 *   - The local OAuth2 redirect listener uses qjs-lws directly (server side),
 *     since that's the part the abstraction does not cover.
 *
 * Setup (one-time):
 *   a) https://console.cloud.google.com -> create / pick a project
 *   b) Enable the "People API" for that project
 *   c) APIs & Services -> Credentials -> Create OAuth client ID
 *      Application type: "Desktop app"
 *   d) Save them to ~/.google-contacts-creds.json:
 *        { "client_id": "...", "client_secret": "..." }
 *
 * Usage:
 *   qjs google-contacts.js                       # list all contacts, newest first
 *   qjs google-contacts.js --json                # raw JSON dump
 *   qjs google-contacts.js --since 2026-01       # only contacts newer than prefix
 *   qjs google-contacts.js --reauth              # force a fresh OAuth flow
 *   qjs google-contacts.js --backend lws         # use qjs-lws for HTTPS
 *   qjs google-contacts.js --backend curl        # use curl for HTTPS (default)
 */

import * as fs from 'fs';
import * as path from 'path';
import { spawn } from 'child_process';
import { LWSContext } from 'lws.so';
import { HttpClient } from './http-client.js';

//----------------------------------------------------------------------------
// Config & paths
//----------------------------------------------------------------------------
const HOME = path.gethome();
const CREDS_PATH = path.join(HOME, '.google-contacts-creds.json');
const TOKENS_PATH = path.join(HOME, '.google-contacts-tokens.json');

const REDIRECT_PORT = 8080;
const REDIRECT_URI = `http://localhost:${REDIRECT_PORT}/callback`;
const SCOPE = 'https://www.googleapis.com/auth/contacts.readonly';

// libwebsockets server-side callback reason for plain HTTP requests
const LWS_CALLBACK_HTTP = 12;

//----------------------------------------------------------------------------
// Tiny helpers
//----------------------------------------------------------------------------
function readJson(p) {
  return JSON.parse(fs.readFileSync(p, 'utf8'));
}
function writeJson(p, o) {
  fs.writeFileSync(p, JSON.stringify(o, null, 2) + '\n');
}

function die(msg) {
  console.log('ERROR:', msg);
  throw new Error('__exit__');
}

// Open a URL in the user's default browser, best effort.
function openBrowser(url) {
  for(const cmd of ['xdg-open', 'open', 'sensible-browser', 'firefox']) {
    try {
      spawn(cmd, [url], { detached: true });
      return true;
    } catch(_) {}
  }
  return false;
}

//----------------------------------------------------------------------------
// Google OAuth2 — using HttpClient for the HTTPS side
//----------------------------------------------------------------------------
function doOAuthFlow(http, creds) {
  const authParams = {
    client_id: creds.client_id,
    redirect_uri: REDIRECT_URI,
    response_type: 'code',
    scope: SCOPE,
    access_type: 'offline',
    prompt: 'consent',
  };
  const authUrl =
    'https://accounts.google.com/o/oauth2/v2/auth?' +
    Object.keys(authParams)
      .map(k => encodeURIComponent(k) + '=' + encodeURIComponent(authParams[k]))
      .join('&');

  console.log('Opening browser for Google authorization...');
  console.log('If it does not open, visit this URL manually:\n');
  console.log('  ' + authUrl + '\n');
  openBrowser(authUrl);

  const code = waitForAuthCode();
  console.log('Got authorization code, exchanging for tokens...');

  const r = http.postForm('https://oauth2.googleapis.com/token', {
    code,
    client_id: creds.client_id,
    client_secret: creds.client_secret,
    redirect_uri: REDIRECT_URI,
    grant_type: 'authorization_code',
  });

  if(r.status !== 200) die(`token exchange returned HTTP ${r.status}: ${r.body}`);

  const tokens = JSON.parse(r.body);
  if(!tokens.access_token) die('token exchange response missing access_token: ' + r.body);

  // Store absolute expiry so we can refresh proactively
  tokens.expires_at = Math.floor(Date.now() / 1000) + (tokens.expires_in || 3600) - 60;
  writeJson(TOKENS_PATH, tokens);
  console.log('Tokens cached to', TOKENS_PATH);
  return tokens;
}

function refreshTokens(http, creds, refreshToken) {
  const r = http.postForm('https://oauth2.googleapis.com/token', {
    client_id: creds.client_id,
    client_secret: creds.client_secret,
    refresh_token: refreshToken,
    grant_type: 'refresh_token',
  });
  if(r.status !== 200) return null;
  try {
    return JSON.parse(r.body);
  } catch(_) {
    return null;
  }
}

function getAccessToken(http, creds, forceReauth) {
  if(forceReauth || !fs.existsSync(TOKENS_PATH)) return doOAuthFlow(http, creds).access_token;

  const tokens = readJson(TOKENS_PATH);
  const now = Math.floor(Date.now() / 1000);

  if(tokens.access_token && tokens.expires_at && now < tokens.expires_at) return tokens.access_token;

  if(tokens.refresh_token) {
    console.log('Refreshing access token...');
    const fresh = refreshTokens(http, creds, tokens.refresh_token);
    if(fresh && fresh.access_token) {
      tokens.access_token = fresh.access_token;
      tokens.expires_at = now + (fresh.expires_in || 3600) - 60;
      writeJson(TOKENS_PATH, tokens);
      return tokens.access_token;
    }
    console.log('Refresh failed, falling back to full reauth.');
  }

  return doOAuthFlow(http, creds).access_token;
}

//----------------------------------------------------------------------------
// Loopback redirect listener — qjs-lws as server
//----------------------------------------------------------------------------
function waitForAuthCode() {
  let receivedCode = null;
  let receivedError = null;

  const ctx = new LWSContext({
    port: REDIRECT_PORT,
    vhostName: 'localhost',
    protocols: [
      {
        name: 'http',
        callback(wsi, reason, user, buf) {
          if(reason !== LWS_CALLBACK_HTTP) return 0;

          const uri = typeof buf === 'string' ? buf : '/';
          const params = parseQuery(uri);

          if(params.code) {
            receivedCode = params.code;
            respond(
              wsi,
              200,
              '<html><body style="font-family:sans-serif;padding:2em">' + '<h2>Authorization received.</h2>' + '<p>You can close this tab and return to the terminal.</p>' + '</body></html>',
            );
          } else if(params.error) {
            receivedError = params.error;
            respond(wsi, 400, '<html><body>Authorization failed: ' + params.error + '</body></html>');
          } else {
            respond(wsi, 404, 'not found');
          }
          return 0;
        },
      },
    ],
  });

  // Wait for the callback to capture the code. If qjs-lws services events
  // on its own thread/loop, this just spins until the callback fires.
  while(receivedCode === null && receivedError === null) {
    if(typeof ctx.service === 'function') ctx.service(100);
  }

  if(typeof ctx.destroy === 'function') ctx.destroy();

  if(receivedError) die('OAuth error: ' + receivedError);
  return receivedCode;
}

function parseQuery(uri) {
  const q = uri.indexOf('?');
  if(q < 0) return {};
  const out = {};
  for(const pair of uri.slice(q + 1).split('&')) {
    const [k, v] = pair.split('=');
    out[decodeURIComponent(k)] = decodeURIComponent(v || '');
  }
  return out;
}

function respond(wsi, status, body) {
  const headers = `HTTP/1.1 ${status} ${status === 200 ? 'OK' : 'Error'}\r\n` + `Content-Type: text/html; charset=utf-8\r\n` + `Content-Length: ${body.length}\r\n` + `Connection: close\r\n\r\n`;
  if(typeof wsi.writeRaw === 'function') wsi.writeRaw(headers + body);
  else if(typeof wsi.write === 'function') wsi.write(headers + body);
  else if(typeof wsi.respond === 'function') wsi.respond(status, body, 'text/html');
}

//----------------------------------------------------------------------------
// People API — using HttpClient for HTTPS
//----------------------------------------------------------------------------
function fetchAllContacts(http, accessToken) {
  const fields = ['names', 'phoneNumbers', 'emailAddresses', 'organizations', 'metadata', 'biographies'].join(',');

  const out = [];
  let pageToken = null;
  let page = 0;

  do {
    const params = [`personFields=${fields}`, 'pageSize=1000', 'sortOrder=LAST_MODIFIED_DESCENDING'];
    if(pageToken) params.push('pageToken=' + encodeURIComponent(pageToken));

    const url = 'https://people.googleapis.com/v1/people/me/connections?' + params.join('&');
    const r = http.get(url, { Authorization: `Bearer ${accessToken}` });

    if(r.status !== 200) die(`People API returned HTTP ${r.status}: ${r.body}`);

    const data = JSON.parse(r.body);
    if(data.connections) out.push(...data.connections);

    page++;
    if(data.totalItems && page === 1) console.log(`(server reports ${data.totalItems} total contacts)`);

    pageToken = data.nextPageToken || null;
  } while(pageToken);

  return out;
}

//----------------------------------------------------------------------------
// Contact summarization
//----------------------------------------------------------------------------
function summarize(c) {
  const name = c.names?.[0]?.displayName || '(no name)';
  const phones = (c.phoneNumbers || []).map(p => p.value).join(', ');
  const emails = (c.emailAddresses || []).map(e => e.value).join(', ');
  const org = c.organizations?.[0]?.name || '';
  const note = c.biographies?.[0]?.value?.replace(/\s+/g, ' ').slice(0, 80) || '';

  let earliest = null;
  for(const s of c.metadata?.sources || []) {
    if(s.updateTime && (!earliest || s.updateTime < earliest)) earliest = s.updateTime;
  }
  return { name, phones, emails, org, note, updateTime: earliest };
}

//----------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------
function parseArgs(argv) {
  const args = { backend: 'curl', wantJson: false, forceReauth: false, since: null };
  for(let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if(a === '--json') args.wantJson = true;
    else if(a === '--reauth') args.forceReauth = true;
    else if(a === '--since') args.since = argv[++i];
    else if(a === '--backend') args.backend = argv[++i];
    else if(a === '-h' || a === '--help') {
      args.help = true;
    }
  }
  return args;
}

function main() {
  const args = parseArgs(scriptArgs.slice(1));

  if(args.help) {
    console.log('Usage: qjs google-contacts.js [--backend curl|lws] [--since YYYY-MM] [--json] [--reauth]');
    return;
  }

  if(!fs.existsSync(CREDS_PATH)) {
    console.log(`Missing OAuth credentials file: ${CREDS_PATH}`);
    console.log('See the comment at the top of this script for setup steps.');
    return;
  }

  // Google "Desktop app" credentials wrap the fields under "installed";
  // web-app credentials wrap them under "web". Unwrap either shape.
  let creds = readJson(CREDS_PATH);
  if(creds.installed) creds = creds.installed;
  else if(creds.web) creds = creds.web;

  if(!creds.client_id || !creds.client_secret) die(`credentials file ${CREDS_PATH} is missing client_id or client_secret`);

  const http = new HttpClient({ backend: args.backend, verify: true, timeout: 30000 });

  console.log(`(HTTPS backend: ${args.backend})`);

  try {
    const token = getAccessToken(http, creds, args.forceReauth);

    console.log('Fetching contacts...');
    const raw = fetchAllContacts(http, token);
    console.log(`Fetched ${raw.length} contacts.`);

    let summaries = raw.map(summarize);
    summaries.sort((a, b) => (b.updateTime || '').localeCompare(a.updateTime || ''));

    if(args.since) {
      summaries = summaries.filter(s => s.updateTime && s.updateTime >= args.since);
      console.log(`Filtered to ${summaries.length} contacts since ${args.since}.`);
    }

    if(args.wantJson) {
      console.log(JSON.stringify(summaries, null, 2));
      return;
    }

    for(const s of summaries) {
      const t = s.updateTime || '(no timestamp)';
      console.log(`[${t}]  ${s.name}`);
      if(s.phones) console.log(`             phone:  ${s.phones}`);
      if(s.emails) console.log(`             email:  ${s.emails}`);
      if(s.org) console.log(`             org:    ${s.org}`);
      if(s.note) console.log(`             note:   ${s.note}`);
    }
  } finally {
    http.close();
  }
}

try {
  main();
} catch(e) {
  if(String(e.message).indexOf('__exit__') < 0) console.log('Fatal:', e.message);
}
