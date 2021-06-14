/**       
 * @function rl_abort        
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_abort', dlsym(libreadline, 'rl_abort'), null, 'int', 'int', 'int');
function rl_abort(count, key) {
  return call('rl_abort', count, key);
}

/**       
 * @function rl_add_defun    
 * 
 * @param    {String}        name
 * @param    {Number}        function
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_add_defun', dlsym(libreadline, 'rl_add_defun'), null, 'int', 'char *', 'void *', 'int');
function rl_add_defun(name, function, key) {
  return call('rl_add_defun', name, function, key);
}

/**       
 * @function rl_add_funmap_entry
 * 
 * @param    {String}        name
 * @param    {Number}        function
 * 
 * @return   {Number}        
 */
define('rl_add_funmap_entry', dlsym(libreadline, 'rl_add_funmap_entry'), null, 'int', 'char *', 'void *');
function rl_add_funmap_entry(name, function) {
  return call('rl_add_funmap_entry', name, function);
}

/**       
 * @function rl_add_undo     
 * 
 * @param    {Number}        what
 * @param    {Number}        start
 * @param    {Number}        end
 * @param    {String}        text
 * 
 * @return   {Number}        
 */
define('rl_add_undo', dlsym(libreadline, 'rl_add_undo'), null, 'void', 'int', 'int', 'int', 'char *');
function rl_add_undo(what, start, end, text) {
  return call('rl_add_undo', what, start, end, text);
}

/**       
 * @function rl_alphabetic   
 * 
 * @param    {Number}        c
 * 
 * @return   {Number}        
 */
define('rl_alphabetic', dlsym(libreadline, 'rl_alphabetic'), null, 'int', 'int');
function rl_alphabetic(c) {
  return call('rl_alphabetic', c);
}

/**       
 * @function rl_arrow_keys   
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_arrow_keys', dlsym(libreadline, 'rl_arrow_keys'), null, 'int', 'int', 'int');
function rl_arrow_keys(count, key) {
  return call('rl_arrow_keys', count, key);
}

/**       
 * @function rl_backward     
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward', dlsym(libreadline, 'rl_backward'), null, 'int', 'int', 'int');
function rl_backward(count, key) {
  return call('rl_backward', count, key);
}

/**       
 * @function rl_backward_byte
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_byte', dlsym(libreadline, 'rl_backward_byte'), null, 'int', 'int', 'int');
function rl_backward_byte(count, key) {
  return call('rl_backward_byte', count, key);
}

/**       
 * @function rl_backward_char
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_char', dlsym(libreadline, 'rl_backward_char'), null, 'int', 'int', 'int');
function rl_backward_char(count, key) {
  return call('rl_backward_char', count, key);
}

/**       
 * @function rl_backward_char_search
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_char_search', dlsym(libreadline, 'rl_backward_char_search'), null, 'int', 'int', 'int');
function rl_backward_char_search(count, key) {
  return call('rl_backward_char_search', count, key);
}
