import * as fs from 'fs';
import * as path from 'path';

/**
 * Splits a C or JavaScript source file into logical code chunks:
 * classes, functions, and a fallback global scope block.
 */
export function chunkSourceCode(filePath, content) {
  const ext = path.extname(filePath).toLowerCase();
  const chunks = [];

  if (['.js', '.mjs', '.ts'].includes(ext)) {
    chunks.push(...chunkJavaScript(filePath, content));
  } else if (['.c', '.h', '.cpp', '.hpp', '.cc', '.cxx'].includes(ext)) {
    chunks.push(...chunkCStyle(filePath, content));
  } else {
    // Fallback: Treat the whole file as a single block if type is unknown
    chunks.push({
      text: content,
      meta: { filePath, type: 'file', name: path.basename(filePath) }
    });
  }

  return chunks;
}

/**
 * Regex-based structural chunker for JavaScript/TypeScript
 */
function chunkJavaScript(filePath, content) {
  const chunks = [];
  
  // Match standard/arrow/async functions and class blocks roughly by signature and balanced braces
  // This lightweight heuristic scans for declarations like: class Name, function name(), or const name = (...) =>
  const pattern = /(?:class\s+([A-Za-z0-9_$]+))|(?:(?:async\s+)?function\s+([A-Za-z0-9_$]+))|(?:(?:const|let|var)\s+([A-Za-z0-9_$]+)\s*=\s*(?:async\s*)?\([^)]*\)\s*=>)/g;
  
  let match;
  let lastIndex = 0;

  while ((match = pattern.exec(content)) !== null) {
    const blockType = match[1] ? 'class' : 'function';
    const blockName = match[1] || match[2] || match[3] || 'anonymous';
    const startIndex = match.index;

    // Grab preceding lines as global/utility scope if there's a gap
    if (startIndex > lastIndex) {
      const globalSegment = content.slice(lastIndex, startIndex).trim();
      if (globalSegment.length > 30) { // Filter out purely whitespace/trivial syntax gaps
        chunks.push({
          text: globalSegment,
          meta: { filePath, type: 'global', name: 'global-scope' }
        });
      }
    }

    // Extract block body using brace counting
    const bodyEndIndex = extractBalancedBraceBlock(content, startIndex);
    const codeChunk = content.slice(startIndex, bodyEndIndex);

    chunks.push({
      text: codeChunk,
      meta: { filePath, type: blockType, name: blockName }
    });

    pattern.lastIndex = bodyEndIndex;
    lastIndex = bodyEndIndex;
  }

  // Push remaining tail content as global scope
  if (lastIndex < content.length) {
    const tail = content.slice(lastIndex).trim();
    if (tail.length > 0) {
      chunks.push({
        text: tail,
        meta: { filePath, type: 'global', name: 'global-tail' }
      });
    }
  }

  return chunks.length > 0 ? chunks : [{ text: content, meta: { filePath, type: 'file', name: 'entire-file' } }];
}

/**
 * Regex-based structural chunker for C / C++
 */
function chunkCStyle(filePath, content) {
  const chunks = [];
  // Matches typical C function definitions (e.g., "int main(int argc, char** argv) {")
  const pattern = /^[a-zA-Z_][\w\s\*]+\s+([a-zA-Z_]\w*)\s*\([^;]*\)\s*\{/gm;

  let match;
  let lastIndex = 0;

  while ((match = pattern.exec(content)) !== null) {
    const funcName = match[1];
    const startIndex = match.index;

    if (startIndex > lastIndex) {
      const globalSegment = content.slice(lastIndex, startIndex).trim();
      if (globalSegment.length > 30) {
        chunks.push({
          text: globalSegment,
          meta: { filePath, type: 'global', name: 'includes-and-globals' }
        });
      }
    }

    const bodyEndIndex = extractBalancedBraceBlock(content, startIndex);
    const codeChunk = content.slice(startIndex, bodyEndIndex);

    chunks.push({
      text: codeChunk,
      meta: { filePath, type: 'function', name: funcName }
    });

    pattern.lastIndex = bodyEndIndex;
    lastIndex = bodyEndIndex;
  }

  if (lastIndex < content.length) {
    const tail = content.slice(lastIndex).trim();
    if (tail.length > 0) {
      chunks.push({
        text: tail,
        meta: { filePath, type: 'global', name: 'global-tail' }
      });
    }
  }

  return chunks.length > 0 ? chunks : [{ text: content,meta: { filePath, type: 'file', name: 'entire-file' } }];
}

/**
 * Utility: Find closing brace index for a block starting at startIndex
 */
function extractBalancedBraceBlock(str, startIndex) {
  let braceCount = 0;
  let foundFirstBrace = false;

  for (let i = startIndex; i < str.length; i++) {
    if (str[i] === '{') {
      braceCount++;
      foundFirstBrace = true;
    } else if (str[i] === '}') {
      braceCount--;
    }
    if (foundFirstBrace && braceCount === 0) {
      return i + 1;
    }
  }
  // Fallback if brackets are unbalanced: take next 1000 chars or end of file
  return Math.min(str.length, startIndex + 1000);
}

/**
 * High-level helper: Recursively walks a directory, chunks code files, 
 * and feeds them directly into an Embeddings instance.
 */
export async function indexProjectSource(searchInstance, rootDir, extensions = ['.js', '.c', '.h', '.cpp']) {
  function walk(dir, fileList = []) {
    for (const file of fs.readdirSync(dir)) {
      const fullPath = path.join(dir, file);
      if (fs.statSync(fullPath).isDirectory()) {
        if (!['node_modules', '.git', 'build', 'dist'].includes(file)) {
          walk(fullPath, fileList);
        }
      } else if (extensions.includes(path.extname(file).toLowerCase())) {
        fileList.push(fullPath);
      }
    }
    return fileList;
  }

  const files = walk(rootDir);
  let totalChunks = 0;

  for (const filePath of files) {
    const content = fs.readFileSync(filePath, 'utf-8');
    const chunks = chunkSourceCode(filePath, content);

    for (const chunk of chunks) {
      // Feed each parsed function/class/scope block into your existing index
      await searchInstance.add(chunk.text, chunk.meta);
      totalChunks++;
    }
  }

  return { filesCount: files.length, chunksCount: totalChunks };
}
