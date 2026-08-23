/**
 * Text-embedding client + a semantic-search mixin.
 *
 * `OllamaClient` is a minimal provider client, built the same way as
 * quickjs/qjs-lws/examples/ollama-repl/lib/{ollama,openai}-client.js -
 * a persistent, pipelined `LWSContext` connection via the `httpClient`
 * protocol adapter (`quickjs/qjs-lws/lib/lws/protocols.js`), instead of
 * the shared `fetch()` (`quickjs/qjs-lws/lib/fetch.js`), for the same
 * reason those two document in their own header comments: lws's
 * built-in 15s wsi timeout is too short for a local model that has to
 * cold-load first, and `fetch()` has no way to override it. Stripped
 * down to what embeddings need though - one JSON POST, one JSON
 * response, no streaming, no tool calls, no chat message conversion.
 *
 * `Embeddings(Base)` is a mixin: it returns a class, extending `Base`,
 * that adds `embed()` (turns text into vectors - "sentence transforming")
 * plus a minimal in-memory `add()`/`search()` semantic index on top of
 * it (cosine similarity, no persistence, no vector database - just an
 * array, for small/medium in-process corpora). `Base` only has to
 * implement `_embedRaw(texts: string[]): Promise<number[][]>` - that's
 * the one piece that differs per provider (OllamaClient below, or your
 * own, e.g. an OpenAI-compatible `/embeddings` endpoint).
 *
 * Example:
 *   class SemanticSearch extends Embeddings(OllamaClient) {}
 *
 *   const search = new SemanticSearch({ model: 'nomic-embed-text' });
 *   await search.add(['a cat sat on the mat', 'stocks fell sharply today']);
 *   const hits = await search.search('kitten on a rug');
 *   search.destroy();
 *
 * Used directly (no semantic index, just embed()):
 *   const E = Embeddings(OllamaClient);
 *   const emb = new E({ model: 'nomic-embed-text' });
 *   const vector = await emb.embed('hello world');
 */
import createContext from 'lws/context.js';
import { httpClient } from 'lws/protocols.js';
import { LCCSCF_PIPELINE } from 'lws.so';

/* Ollama models with an "embedding" capability, as of this writing - for
   discoverability/autocomplete, not enforced anywhere below. */
export const OLLAMA_EMBEDDING_MODELS = [
  'bge-large',
  'bge-m3',
  'embeddinggemma',
  'granite-embedding',
  'mxbai-embed-large',
  'nomic-embed-text',
  'nomic-embed-text-v2-moe',
  'qwen3-embedding',
  'qwen3-embedding:0.6b',
  'snowflake-arctic-embed',
  'snowflake-arctic-embed2',
];

const DEFAULT_TIMEOUT_SECS = 5 * 60;

/* Mirrors OllamaClient's own IDLE_RECONNECT_MS (ollama-client.js) and the
   BUGS entry it documents (ollama-pipelined-connection-hangs-after-idle-gap):
   a pipelined connection left idle this long may already be dead without
   lws having noticed, so it's replaced with a fresh one rather than reused. */
const IDLE_RECONNECT_MS = 30_000;

/** Shared connect+await-JSON-response plumbing, used by both clients below. */
class JsonPostClient {
  #ctx;
  #adapter;
  #settled = new Map(); // req -> {resolve, reject}
  #timeoutMs;
  #lastActivityAt = null;

  constructor(timeoutSecs = DEFAULT_TIMEOUT_SECS) {
    this.#timeoutMs = timeoutSecs * 1000;
    this.#adapter = httpClient((req, resp) => this.#take(req)?.resolve(resp), { error: (req, err) => this.#reject(req, err) });
    this.#ctx = createContext({ protocols: [{ name: 'http', ...this.#adapter }], timeoutSecs });
  }

  #take(req) {
    const record = this.#settled.get(req);
    this.#settled.delete(req);
    return record;
  }

  #reject(req, err) {
    const reason = new Error(`request failed: ${err.message}`);
    if(req) {
      this.#take(req)?.reject(reason);
      return;
    }
    for(const { reject } of this.#settled.values()) reject(reason);
    this.#settled.clear();
  }

  async post(url, headers, payload) {
    if(this.#lastActivityAt != null && Date.now() - this.#lastActivityAt > IDLE_RECONNECT_MS) {
      this.#ctx.destroy();
      this.#ctx = createContext({ protocols: [{ name: 'http', ...this.#adapter }], timeoutSecs: this.#timeoutMs / 1000 });
    }

    const { req, wsi } = await this.#adapter.connect(this.#ctx, url, {
      method: 'POST',
      headers: { 'content-type': 'application/json', ...headers },
      body: JSON.stringify(payload),
      sslConnection: LCCSCF_PIPELINE,
    });

    const resp = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.#take(req);
        wsi?.close();
        reject(new Error(`no response after ${(this.#timeoutMs / 1000).toFixed(0)}s`));
      }, this.#timeoutMs);

      this.#settled.set(req, {
        resolve: v => {
          clearTimeout(timer);
          resolve(v);
        },
        reject: e => {
          clearTimeout(timer);
          reject(e);
        },
      });
    });

    this.#lastActivityAt = Date.now();

    if(resp.status < 200 || resp.status >= 300) throw new Error(`HTTP ${resp.status}: ${await resp.text().catch(() => '')}`);

    return resp.json();
  }

  destroy() {
    this.#ctx.destroy();
  }
}

/**
 * Local Ollama server, `POST /api/embed`. Provides `_embedRaw()` for
 * `Embeddings()` below - use it as `new (Embeddings(OllamaClient))(opts)`
 * or `class X extends Embeddings(OllamaClient) {}`, not directly.
 */
export class OllamaClient {
  #client;
  #url;
  model;

  /**
   * @param {object} opts
   * @param {string} [opts.host]        Ollama server hostname (default "localhost")
   * @param {number} [opts.port]        Ollama server port (default 11434)
   * @param {string} opts.model         embedding model name, e.g. "nomic-embed-text"
   * @param {number} [opts.timeoutSecs] how long a request may wait before giving up
   */
  constructor({ host = 'localhost', port = 11434, model, timeoutSecs = DEFAULT_TIMEOUT_SECS } = {}) {
    if(!model) throw new Error('OllamaClient: model is required');
    this.model = model;
    this.#url = `http://${host}:${port}/api/embed`;
    this.#client = new JsonPostClient(timeoutSecs);
  }

  /** @param {string[]} texts @returns {Promise<number[][]>} one vector per text, in order */
  async _embedRaw(texts) {
    const data = await this.#client.post(this.#url, {}, { model: this.model, input: texts });
    if(!Array.isArray(data.embeddings)) throw new Error(`unexpected Ollama response: ${JSON.stringify(data)}`);
    return data.embeddings;
  }

  destroy() {
    this.#client.destroy();
  }
}

/** Cosine similarity between two equal-length vectors, in [-1, 1] (1 = identical direction). */
export function cosineSimilarity(a, b) {
  let dot = 0,
    na = 0,
    nb = 0;
  for(let i = 0; i < a.length; i++) {
    dot += a[i] * b[i];
    na += a[i] * a[i];
    nb += b[i] * b[i];
  }
  return dot / (Math.sqrt(na) * Math.sqrt(nb));
}

/**
 * Mixin: adds `embed()` and a minimal in-memory semantic index
 * (`add()`/`search()`/`size`) to `Base` (OllamaClient, or anything else
 * implementing `_embedRaw(texts: string[])`, see above).
 *
 * @param {Function} Base
 * @returns {Function} a class extending Base
 */
export function Embeddings(Base) {
  return class extends Base {
    #entries = []; // { text, vector, meta }

    /**
     * @param {string|string[]} input
     * @returns {Promise<number[]|number[][]>} one vector for a string
     *   input, one vector per element (in order) for an array input
     */
    async embed(input) {
      const batch = Array.isArray(input) ? input : [input];
      const vectors = await this._embedRaw(batch);
      return Array.isArray(input) ? vectors : vectors[0];
    }

    /**
     * Embeds and adds one or more texts to this instance's semantic index.
     * @param {string|string[]} texts
     * @param {*} [meta] attached to every entry added by this call, and
     *   returned alongside matches from search()
     */
    async add(texts, meta) {
      const list = Array.isArray(texts) ? texts : [texts];
      const vectors = await this._embedRaw(list);
      list.forEach((text, i) => this.#entries.push({ text, vector: vectors[i], meta }));
    }

    /**
     * @param {string} query
     * @param {number} [k] max results (default 5)
     * @returns {Promise<{text: string, score: number, meta: *}[]>} the k
     *   entries added via add() closest to `query`, most similar first
     */
    async search(query, k = 5) {
      const [vector] = await this._embedRaw([query]);
      return this.#entries
        .map(e => ({ text: e.text, score: cosineSimilarity(vector, e.vector), meta: e.meta }))
        .sort((a, b) => b.score - a.score)
        .slice(0, k);
    }

    get size() {
      return this.#entries.length;
    }
  };
}
