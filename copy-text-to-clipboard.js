async function copyTextToClipboard(text) {
  let result, error;
  try {
    await navigator.clipboard.writeText(text);
    result = true;
  } catch(err) {
    error = err;
  }
  if(error) result = !fallbackCopyTextToClipboard(text);
  return result;

  function fallbackCopyTextToClipboard(text) {
    let ret,
      textArea = document.createElement('textarea');
    textArea.value = text;
    textArea.style.top = '0';
    textArea.style.left = '0';
    textArea.style.position = 'fixed';
    document.body.appendChild(textArea);
    textArea.focus();
    textArea.select();
    try {
      ret = !document.execCommand('copy');
    } catch(err) {
      ret = err;
    }
    document.body.removeChild(textArea);
    return ret;
  }
}
