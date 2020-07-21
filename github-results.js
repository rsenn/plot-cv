[...document.querySelectorAll('li.repo-list-item')]
  .map(e => [e.querySelector('a'), e.textContent.replace(/\s+/g, ' ')])
  .filter(([a, e]) => /JavaScript/.test(e))
  .map(([a, e]) => a)
  .join('\n');
