/**
 * Demo/smoke-test for embeddings-client.js against a local Ollama server.
 * Run: qjsm test-embeddings.js [model]
 */
import { OllamaClient, Embeddings } from './embeddings-client.js';

const model = scriptArgs[1] || 'bge-m3';

class SemanticSearch extends Embeddings(OllamaClient) {}

const CORPUS = [
  'The cat sat on the mat.',
  'Stocks fell sharply today amid inflation fears.',
  'A kitten curled up on the rug.',
  'The central bank raised interest rates.',
  'Dogs are loyal and playful companions.',
  'The recipe calls for two cups of flour and a pinch of salt.',
  'Quantum computers use qubits instead of classical bits.',
  'She baked a chocolate cake for the birthday party.',
];

const QUERIES = ['a small cat', 'financial markets and money', 'baking a dessert'];

async function main() {
  const search = new SemanticSearch({ model });

  console.log(`-- embed() sanity check (model: ${model}) --`);
  const vector = await search.embed('hello world');
  console.log(`single: ${Array.isArray(vector)} dims=${vector.length}`);

  console.log('\n-- indexing corpus --');
  await search.add(CORPUS);
  console.log(`indexed ${search.size} entries`);

  for(const query of QUERIES) {
    console.log(`\n-- search: "${query}" --`);
    const hits = await search.search(query, 3);
    for(const hit of hits) console.log(`  ${hit.score.toFixed(3)}  ${hit.text}`);
  }

  search.destroy();
}

main().catch(e => {
  console.log(`error: ${e.message}`);
  std.exit(1);
});
