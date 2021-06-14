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
/**       
 * @function rl_backward_kill_line
 * 
 * @param    {Number}        direction
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_kill_line', dlsym(libreadline, 'rl_backward_kill_line'), null, 'int', 'int', 'int');
function rl_backward_kill_line(direction, key) {
  return call('rl_backward_kill_line', direction, key);
}
/**       
 * @function rl_backward_kill_word
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_kill_word', dlsym(libreadline, 'rl_backward_kill_word'), null, 'int', 'int', 'int');
function rl_backward_kill_word(count, key) {
  return call('rl_backward_kill_word', count, key);
}
/**       
 * @function rl_backward_menu_complete
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_menu_complete', dlsym(libreadline, 'rl_backward_menu_complete'), null, 'int', 'int', 'int');
function rl_backward_menu_complete(count, key) {
  return call('rl_backward_menu_complete', count, key);
}
/**       
 * @function rl_backward_word
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_backward_word', dlsym(libreadline, 'rl_backward_word'), null, 'int', 'int', 'int');
function rl_backward_word(count, key) {
  return call('rl_backward_word', count, key);
}
/**       
 * @function rl_beg_of_line  
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_beg_of_line', dlsym(libreadline, 'rl_beg_of_line'), null, 'int', 'int', 'int');
function rl_beg_of_line(count, key) {
  return call('rl_beg_of_line', count, key);
}
/**       
 * @function rl_begin_undo_group
 * 
 * 
 * @return   {Number}        
 */
define('rl_begin_undo_group', dlsym(libreadline, 'rl_begin_undo_group'), null, 'int');
function rl_begin_undo_group() {
  return call('rl_begin_undo_group', );
}
/**       
 * @function rl_beginning_of_history
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_beginning_of_history', dlsym(libreadline, 'rl_beginning_of_history'), null, 'int', 'int', 'int');
function rl_beginning_of_history(count, key) {
  return call('rl_beginning_of_history', count, key);
}
/**       
 * @function rl_bind_key     
 * 
 * @param    {Number}        key
 * @param    {Number}        function
 * 
 * @return   {Number}        
 */
define('rl_bind_key', dlsym(libreadline, 'rl_bind_key'), null, 'int', 'int', 'void *');
function rl_bind_key(key, function) {
  return call('rl_bind_key', key, function);
}
/**       
 * @function rl_bind_key_if_unbound
 * 
 * @param    {Number}        key
 * @param    {Number}        default_func
 * 
 * @return   {Number}        
 */
define('rl_bind_key_if_unbound', dlsym(libreadline, 'rl_bind_key_if_unbound'), null, 'int', 'int', 'void *');
function rl_bind_key_if_unbound(key, default_func) {
  return call('rl_bind_key_if_unbound', key, default_func);
}
/**       
 * @function rl_bind_key_if_unbound_in_map
 * 
 * @param    {Number}        key
 * @param    {Number}        default_func
 * @param    {Number}        kmap
 * 
 * @return   {Number}        
 */
define('rl_bind_key_if_unbound_in_map', dlsym(libreadline, 'rl_bind_key_if_unbound_in_map'), null, 'int', 'int', 'void *', 'long');
function rl_bind_key_if_unbound_in_map(key, default_func, kmap) {
  return call('rl_bind_key_if_unbound_in_map', key, default_func, kmap);
}
/**       
 * @function rl_bind_key_in_map
 * 
 * @param    {Number}        key
 * @param    {Number}        function
 * @param    {Number}        map
 * 
 * @return   {Number}        
 */
define('rl_bind_key_in_map', dlsym(libreadline, 'rl_bind_key_in_map'), null, 'int', 'int', 'void *', 'long');
function rl_bind_key_in_map(key, function, map) {
  return call('rl_bind_key_in_map', key, function, map);
}
/**       
 * @function rl_bind_keyseq  
 * 
 * @param    {String}        keyseq
 * @param    {Number}        function
 * 
 * @return   {Number}        
 */
define('rl_bind_keyseq', dlsym(libreadline, 'rl_bind_keyseq'), null, 'int', 'char *', 'void *');
function rl_bind_keyseq(keyseq, function) {
  return call('rl_bind_keyseq', keyseq, function);
}
/**       
 * @function rl_bind_keyseq_if_unbound
 * 
 * @param    {String}        keyseq
 * @param    {Number}        default_func
 * 
 * @return   {Number}        
 */
define('rl_bind_keyseq_if_unbound', dlsym(libreadline, 'rl_bind_keyseq_if_unbound'), null, 'int', 'char *', 'void *');
function rl_bind_keyseq_if_unbound(keyseq, default_func) {
  return call('rl_bind_keyseq_if_unbound', keyseq, default_func);
}
/**       
 * @function rl_bind_keyseq_if_unbound_in_map
 * 
 * @param    {String}        keyseq
 * @param    {Number}        default_func
 * @param    {Number}        kmap
 * 
 * @return   {Number}        
 */
define('rl_bind_keyseq_if_unbound_in_map', dlsym(libreadline, 'rl_bind_keyseq_if_unbound_in_map'), null, 'int', 'char *', 'void *', 'long');
function rl_bind_keyseq_if_unbound_in_map(keyseq, default_func, kmap) {
  return call('rl_bind_keyseq_if_unbound_in_map', keyseq, default_func, kmap);
}
/**       
 * @function rl_bind_keyseq_in_map
 * 
 * @param    {String}        keyseq
 * @param    {Number}        function
 * @param    {Number}        map
 * 
 * @return   {Number}        
 */
define('rl_bind_keyseq_in_map', dlsym(libreadline, 'rl_bind_keyseq_in_map'), null, 'int', 'char *', 'void *', 'long');
function rl_bind_keyseq_in_map(keyseq, function, map) {
  return call('rl_bind_keyseq_in_map', keyseq, function, map);
}
/**       
 * @function rl_bracketed_paste_begin
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_bracketed_paste_begin', dlsym(libreadline, 'rl_bracketed_paste_begin'), null, 'int', 'int', 'int');
function rl_bracketed_paste_begin(count, key) {
  return call('rl_bracketed_paste_begin', count, key);
}
/**       
 * @function rl_call_last_kbd_macro
 * 
 * @param    {Number}        count
 * @param    {Number}        ignore
 * 
 * @return   {Number}        
 */
define('rl_call_last_kbd_macro', dlsym(libreadline, 'rl_call_last_kbd_macro'), null, 'int', 'int', 'int');
function rl_call_last_kbd_macro(count, ignore) {
  return call('rl_call_last_kbd_macro', count, ignore);
}
/**       
 * @function rl_callback_handler_install
 * 
 * @param    {String}        prompt
 * @param    {Number}        linefunc
 * 
 * @return   {Number}        
 */
define('rl_callback_handler_install', dlsym(libreadline, 'rl_callback_handler_install'), null, 'void', 'char *', 'void *');
function rl_callback_handler_install(prompt, linefunc) {
  return call('rl_callback_handler_install', prompt, linefunc);
}
/**       
 * @function rl_callback_handler_remove
 * 
 * 
 * @return   {Number}        
 */
define('rl_callback_handler_remove', dlsym(libreadline, 'rl_callback_handler_remove'), null, 'void');
function rl_callback_handler_remove() {
  return call('rl_callback_handler_remove', );
}
/**       
 * @function rl_callback_read_char
 * 
 * 
 * @return   {Number}        
 */
define('rl_callback_read_char', dlsym(libreadline, 'rl_callback_read_char'), null, 'void');
function rl_callback_read_char() {
  return call('rl_callback_read_char', );
}
/**       
 * @function rl_callback_sigcleanup
 * 
 * 
 * @return   {Number}        
 */
define('rl_callback_sigcleanup', dlsym(libreadline, 'rl_callback_sigcleanup'), null, 'void');
function rl_callback_sigcleanup() {
  return call('rl_callback_sigcleanup', );
}
/**       
 * @function rl_capitalize_word
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_capitalize_word', dlsym(libreadline, 'rl_capitalize_word'), null, 'int', 'int', 'int');
function rl_capitalize_word(count, key) {
  return call('rl_capitalize_word', count, key);
}
/**       
 * @function rl_char_search  
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_char_search', dlsym(libreadline, 'rl_char_search'), null, 'int', 'int', 'int');
function rl_char_search(count, key) {
  return call('rl_char_search', count, key);
}
/**       
 * @function rl_character_len
 * 
 * @param    {Number}        c
 * @param    {Number}        pos
 * 
 * @return   {Number}        
 */
define('rl_character_len', dlsym(libreadline, 'rl_character_len'), null, 'int', 'int', 'int');
function rl_character_len(c, pos) {
  return call('rl_character_len', c, pos);
}
/**       
 * @function rl_check_signals
 * 
 * 
 * @return   {Number}        
 */
define('rl_check_signals', dlsym(libreadline, 'rl_check_signals'), null, 'void');
function rl_check_signals() {
  return call('rl_check_signals', );
}
/**       
 * @function rl_cleanup_after_signal
 * 
 * 
 * @return   {Number}        
 */
define('rl_cleanup_after_signal', dlsym(libreadline, 'rl_cleanup_after_signal'), null, 'void');
function rl_cleanup_after_signal() {
  return call('rl_cleanup_after_signal', );
}
/**       
 * @function rl_clear_history
 * 
 * 
 * @return   {Number}        
 */
define('rl_clear_history', dlsym(libreadline, 'rl_clear_history'), null, 'void');
function rl_clear_history() {
  return call('rl_clear_history', );
}
/**       
 * @function rl_clear_message
 * 
 * 
 * @return   {Number}        
 */
define('rl_clear_message', dlsym(libreadline, 'rl_clear_message'), null, 'int');
function rl_clear_message() {
  return call('rl_clear_message', );
}
/**       
 * @function rl_clear_pending_input
 * 
 * 
 * @return   {Number}        
 */
define('rl_clear_pending_input', dlsym(libreadline, 'rl_clear_pending_input'), null, 'int');
function rl_clear_pending_input() {
  return call('rl_clear_pending_input', );
}
/**       
 * @function rl_clear_screen 
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_clear_screen', dlsym(libreadline, 'rl_clear_screen'), null, 'int', 'int', 'int');
function rl_clear_screen(count, key) {
  return call('rl_clear_screen', count, key);
}
/**       
 * @function rl_clear_signals
 * 
 * 
 * @return   {Number}        
 */
define('rl_clear_signals', dlsym(libreadline, 'rl_clear_signals'), null, 'int');
function rl_clear_signals() {
  return call('rl_clear_signals', );
}
/**       
 * @function rl_clear_visible_line
 * 
 * 
 * @return   {Number}        
 */
define('rl_clear_visible_line', dlsym(libreadline, 'rl_clear_visible_line'), null, 'int');
function rl_clear_visible_line() {
  return call('rl_clear_visible_line', );
}
/**       
 * @function rl_complete     
 * 
 * @param    {Number}        ignore
 * @param    {Number}        invoking_key
 * 
 * @return   {Number}        
 */
define('rl_complete', dlsym(libreadline, 'rl_complete'), null, 'int', 'int', 'int');
function rl_complete(ignore, invoking_key) {
  return call('rl_complete', ignore, invoking_key);
}
/**       
 * @function rl_complete_internal
 * 
 * @param    {Number}        what_to_do
 * 
 * @return   {Number}        
 */
define('rl_complete_internal', dlsym(libreadline, 'rl_complete_internal'), null, 'int', 'int');
function rl_complete_internal(what_to_do) {
  return call('rl_complete_internal', what_to_do);
}
/**       
 * @function rl_completion_matches
 * 
 * @param    {String}        text
 * @param    {Number}        entry_function
 * 
 * @return   {Number}        
 */
define('rl_completion_matches', dlsym(libreadline, 'rl_completion_matches'), null, 'void *', 'char *', 'void *');
function rl_completion_matches(text, entry_function) {
  return call('rl_completion_matches', text, entry_function);
}
/**       
 * @function rl_completion_mode
 * 
 * @param    {Number}        cfunc
 * 
 * @return   {Number}        
 */
define('rl_completion_mode', dlsym(libreadline, 'rl_completion_mode'), null, 'int', 'void *');
function rl_completion_mode(cfunc) {
  return call('rl_completion_mode', cfunc);
}
/**       
 * @function rl_copy_backward_word
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_copy_backward_word', dlsym(libreadline, 'rl_copy_backward_word'), null, 'int', 'int', 'int');
function rl_copy_backward_word(count, key) {
  return call('rl_copy_backward_word', count, key);
}
/**       
 * @function rl_copy_forward_word
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_copy_forward_word', dlsym(libreadline, 'rl_copy_forward_word'), null, 'int', 'int', 'int');
function rl_copy_forward_word(count, key) {
  return call('rl_copy_forward_word', count, key);
}
/**       
 * @function rl_copy_region_to_kill
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_copy_region_to_kill', dlsym(libreadline, 'rl_copy_region_to_kill'), null, 'int', 'int', 'int');
function rl_copy_region_to_kill(count, key) {
  return call('rl_copy_region_to_kill', count, key);
}
/**       
 * @function rl_copy_text    
 * 
 * @param    {Number}        from
 * @param    {Number}        to
 * 
 * @return   {String}        
 */
define('rl_copy_text', dlsym(libreadline, 'rl_copy_text'), null, 'char *', 'int', 'int');
function rl_copy_text(from, to) {
  return call('rl_copy_text', from, to);
}
/**       
 * @function rl_crlf         
 * 
 * 
 * @return   {Number}        
 */
define('rl_crlf', dlsym(libreadline, 'rl_crlf'), null, 'int');
function rl_crlf() {
  return call('rl_crlf', );
}
/**       
 * @function rl_delete       
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_delete', dlsym(libreadline, 'rl_delete'), null, 'int', 'int', 'int');
function rl_delete(count, key) {
  return call('rl_delete', count, key);
}
/**       
 * @function rl_delete_horizontal_space
 * 
 * @param    {Number}        count
 * @param    {Number}        ignore
 * 
 * @return   {Number}        
 */
define('rl_delete_horizontal_space', dlsym(libreadline, 'rl_delete_horizontal_space'), null, 'int', 'int', 'int');
function rl_delete_horizontal_space(count, ignore) {
  return call('rl_delete_horizontal_space', count, ignore);
}
/**       
 * @function rl_delete_or_show_completions
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_delete_or_show_completions', dlsym(libreadline, 'rl_delete_or_show_completions'), null, 'int', 'int', 'int');
function rl_delete_or_show_completions(count, key) {
  return call('rl_delete_or_show_completions', count, key);
}
/**       
 * @function rl_delete_text  
 * 
 * @param    {Number}        from
 * @param    {Number}        to
 * 
 * @return   {Number}        
 */
define('rl_delete_text', dlsym(libreadline, 'rl_delete_text'), null, 'int', 'int', 'int');
function rl_delete_text(from, to) {
  return call('rl_delete_text', from, to);
}
/**       
 * @function rl_deprep_terminal
 * 
 * 
 * @return   {Number}        
 */
define('rl_deprep_terminal', dlsym(libreadline, 'rl_deprep_terminal'), null, 'void');
function rl_deprep_terminal() {
  return call('rl_deprep_terminal', );
}
/**       
 * @function rl_digit_argument
 * 
 * @param    {Number}        ignore
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_digit_argument', dlsym(libreadline, 'rl_digit_argument'), null, 'int', 'int', 'int');
function rl_digit_argument(ignore, key) {
  return call('rl_digit_argument', ignore, key);
}
/**       
 * @function rl_ding         
 * 
 * 
 * @return   {Number}        
 */
define('rl_ding', dlsym(libreadline, 'rl_ding'), null, 'int');
function rl_ding() {
  return call('rl_ding', );
}
/**       
 * @function rl_discard_argument
 * 
 * 
 * @return   {Number}        
 */
define('rl_discard_argument', dlsym(libreadline, 'rl_discard_argument'), null, 'int');
function rl_discard_argument() {
  return call('rl_discard_argument', );
}
/**       
 * @function rl_discard_keymap
 * 
 * @param    {Number}        map
 * 
 * @return   {Number}        
 */
define('rl_discard_keymap', dlsym(libreadline, 'rl_discard_keymap'), null, 'void', 'long');
function rl_discard_keymap(map) {
  return call('rl_discard_keymap', map);
}
/**       
 * @function rl_discard_keymap
 * 
 * @param    {Number}        map
 * 
 * @return   {Number}        
 */
define('rl_discard_keymap', dlsym(libreadline, 'rl_discard_keymap'), null, 'void', 'long');
function rl_discard_keymap(map) {
  return call('rl_discard_keymap', map);
}
/**       
 * @function rl_display_match_list
 * 
 * @param    {Number}        matches
 * @param    {Number}        len
 * @param    {Number}        max
 * 
 * @return   {Number}        
 */
define('rl_display_match_list', dlsym(libreadline, 'rl_display_match_list'), null, 'void', 'void *', 'int', 'int');
function rl_display_match_list(matches, len, max) {
  return call('rl_display_match_list', matches, len, max);
}
/**       
 * @function rl_do_lowercase_version
 * 
 * @param    {Number}        ignore1
 * @param    {Number}        ignore2
 * 
 * @return   {Number}        
 */
define('rl_do_lowercase_version', dlsym(libreadline, 'rl_do_lowercase_version'), null, 'int', 'int', 'int');
function rl_do_lowercase_version(ignore1, ignore2) {
  return call('rl_do_lowercase_version', ignore1, ignore2);
}
/**       
 * @function rl_do_undo      
 * 
 * 
 * @return   {Number}        
 */
define('rl_do_undo', dlsym(libreadline, 'rl_do_undo'), null, 'int');
function rl_do_undo() {
  return call('rl_do_undo', );
}
/**       
 * @function rl_downcase_word
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_downcase_word', dlsym(libreadline, 'rl_downcase_word'), null, 'int', 'int', 'int');
function rl_downcase_word(count, key) {
  return call('rl_downcase_word', count, key);
}
/**       
 * @function rl_dump_functions
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_dump_functions', dlsym(libreadline, 'rl_dump_functions'), null, 'int', 'int', 'int');
function rl_dump_functions(count, key) {
  return call('rl_dump_functions', count, key);
}
/**       
 * @function rl_dump_macros  
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_dump_macros', dlsym(libreadline, 'rl_dump_macros'), null, 'int', 'int', 'int');
function rl_dump_macros(count, key) {
  return call('rl_dump_macros', count, key);
}
/**       
 * @function rl_dump_variables
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_dump_variables', dlsym(libreadline, 'rl_dump_variables'), null, 'int', 'int', 'int');
function rl_dump_variables(count, key) {
  return call('rl_dump_variables', count, key);
}
/**       
 * @function rl_echo_signal_char
 * 
 * @param    {Number}        sig
 * 
 * @return   {Number}        
 */
define('rl_echo_signal_char', dlsym(libreadline, 'rl_echo_signal_char'), null, 'void', 'int');
function rl_echo_signal_char(sig) {
  return call('rl_echo_signal_char', sig);
}
/**       
 * @function rl_emacs_editing_mode
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_emacs_editing_mode', dlsym(libreadline, 'rl_emacs_editing_mode'), null, 'int', 'int', 'int');
function rl_emacs_editing_mode(count, key) {
  return call('rl_emacs_editing_mode', count, key);
}
/**       
 * @function rl_empty_keymap 
 * 
 * @param    {Number}        keymap
 * 
 * @return   {Number}        
 */
define('rl_empty_keymap', dlsym(libreadline, 'rl_empty_keymap'), null, 'int', 'long');
function rl_empty_keymap(keymap) {
  return call('rl_empty_keymap', keymap);
}
/**       
 * @function rl_end_kbd_macro
 * 
 * @param    {Number}        count
 * @param    {Number}        ignore
 * 
 * @return   {Number}        
 */
define('rl_end_kbd_macro', dlsym(libreadline, 'rl_end_kbd_macro'), null, 'int', 'int', 'int');
function rl_end_kbd_macro(count, ignore) {
  return call('rl_end_kbd_macro', count, ignore);
}
/**       
 * @function rl_end_of_history
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_end_of_history', dlsym(libreadline, 'rl_end_of_history'), null, 'int', 'int', 'int');
function rl_end_of_history(count, key) {
  return call('rl_end_of_history', count, key);
}
/**       
 * @function rl_end_of_line  
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_end_of_line', dlsym(libreadline, 'rl_end_of_line'), null, 'int', 'int', 'int');
function rl_end_of_line(count, key) {
  return call('rl_end_of_line', count, key);
}
/**       
 * @function rl_end_undo_group
 * 
 * 
 * @return   {Number}        
 */
define('rl_end_undo_group', dlsym(libreadline, 'rl_end_undo_group'), null, 'int');
function rl_end_undo_group() {
  return call('rl_end_undo_group', );
}
/**       
 * @function rl_exchange_point_and_mark
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_exchange_point_and_mark', dlsym(libreadline, 'rl_exchange_point_and_mark'), null, 'int', 'int', 'int');
function rl_exchange_point_and_mark(count, key) {
  return call('rl_exchange_point_and_mark', count, key);
}
/**       
 * @function rl_execute_next 
 * 
 * @param    {Number}        c
 * 
 * @return   {Number}        
 */
define('rl_execute_next', dlsym(libreadline, 'rl_execute_next'), null, 'int', 'int');
function rl_execute_next(c) {
  return call('rl_execute_next', c);
}
/**       
 * @function rl_expand_prompt
 * 
 * @param    {String}        prompt
 * 
 * @return   {Number}        
 */
define('rl_expand_prompt', dlsym(libreadline, 'rl_expand_prompt'), null, 'int', 'char *');
function rl_expand_prompt(prompt) {
  return call('rl_expand_prompt', prompt);
}
/**       
 * @function rl_extend_line_buffer
 * 
 * @param    {Number}        len
 * 
 * @return   {Number}        
 */
define('rl_extend_line_buffer', dlsym(libreadline, 'rl_extend_line_buffer'), null, 'void', 'int');
function rl_extend_line_buffer(len) {
  return call('rl_extend_line_buffer', len);
}
/**       
 * @function rl_forced_update_display
 * 
 * 
 * @return   {Number}        
 */
define('rl_forced_update_display', dlsym(libreadline, 'rl_forced_update_display'), null, 'int');
function rl_forced_update_display() {
  return call('rl_forced_update_display', );
}
/**       
 * @function rl_forward      
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_forward', dlsym(libreadline, 'rl_forward'), null, 'int', 'int', 'int');
function rl_forward(count, key) {
  return call('rl_forward', count, key);
}
/**       
 * @function rl_forward_byte 
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_forward_byte', dlsym(libreadline, 'rl_forward_byte'), null, 'int', 'int', 'int');
function rl_forward_byte(count, key) {
  return call('rl_forward_byte', count, key);
}
/**       
 * @function rl_forward_char 
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_forward_char', dlsym(libreadline, 'rl_forward_char'), null, 'int', 'int', 'int');
function rl_forward_char(count, key) {
  return call('rl_forward_char', count, key);
}
/**       
 * @function rl_forward_search_history
 * 
 * @param    {Number}        sign
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_forward_search_history', dlsym(libreadline, 'rl_forward_search_history'), null, 'int', 'int', 'int');
function rl_forward_search_history(sign, key) {
  return call('rl_forward_search_history', sign, key);
}
/**       
 * @function rl_forward_word 
 * 
 * @param    {Number}        count
 * @param    {Number}        key
 * 
 * @return   {Number}        
 */
define('rl_forward_word', dlsym(libreadline, 'rl_forward_word'), null, 'int', 'int', 'int');
function rl_forward_word(count, key) {
  return call('rl_forward_word', count, key);
}
/**       
 * @function rl_free         
 * 
 * @param    {Number}        mem
 * 
 * @return   {Number}        
 */
define('rl_free', dlsym(libreadline, 'rl_free'), null, 'void', 'void *');
function rl_free(mem) {
  return call('rl_free', mem);
}
/**       
 * @function rl_free_keymap  
 * 
 * @param    {Number}        map
 * 
 * @return   {Number}        
 */
define('rl_free_keymap', dlsym(libreadline, 'rl_free_keymap'), null, 'void', 'long');
function rl_free_keymap(map) {
  return call('rl_free_keymap', map);
}
/**       
 * @function rl_free_line_state
 * 
 * 
 * @return   {Number}        
 */
define('rl_free_line_state', dlsym(libreadline, 'rl_free_line_state'), null, 'void');
function rl_free_line_state() {
  return call('rl_free_line_state', );
}
/**       
 * @function rl_free_undo_list
 * 
 * 
 * @return   {Number}        
 */
define('rl_free_undo_list', dlsym(libreadline, 'rl_free_undo_list'), null, 'void');
function rl_free_undo_list() {
  return call('rl_free_undo_list', );
}
/**       
 * @function rl_function_dumper
 * 
 * @param    {Number}        print_readably
 * 
 * @return   {Number}        
 */
define('rl_function_dumper', dlsym(libreadline, 'rl_function_dumper'), null, 'void', 'int');
function rl_function_dumper(print_readably) {
  return call('rl_function_dumper', print_readably);
}
/**       
 * @function rl_function_of_keyseq
 * 
 * @param    {String}        keyseq
 * @param    {Number}        map
 * @param    {Number}        type
 * 
 * @return   {Number}        
 */
define('rl_function_of_keyseq', dlsym(libreadline, 'rl_function_of_keyseq'), null, 'void *', 'char *', 'long', 'void *');
function rl_function_of_keyseq(keyseq, map, type) {
  return call('rl_function_of_keyseq', keyseq, map, type);
}
/**       
 * @function rl_function_of_keyseq_len
 * 
 * @param    {String}        keyseq
 * @param    {Number}        len
 * @param    {Number}        map
 * @param    {Number}        type
 * 
 * @return   {Number}        
 */
define('rl_function_of_keyseq_len', dlsym(libreadline, 'rl_function_of_keyseq_len'), null, 'void *', 'char *', 'size_t', 'long', 'void *');
function rl_function_of_keyseq_len(keyseq, len, map, type) {
  return call('rl_function_of_keyseq_len', keyseq, len, map, type);
}
/**       
 * @function rl_funmap_names 
 * 
 * 
 * @return   {Number}        
 */
define('rl_funmap_names', dlsym(libreadline, 'rl_funmap_names'), null, 'void *');
function rl_funmap_names() {
  return call('rl_funmap_names', );
}
/**       
 * @function rl_generic_bind 
 * 
 * @param    {Number}        type
 * @param    {String}        keyseq
 * @param    {String}        data
 * @param    {Number}        map
 * 
 * @return   {Number}        
 */
define('rl_generic_bind', dlsym(libreadline, 'rl_generic_bind'), null, 'int', 'int', 'char *', 'char *', 'long');
function rl_generic_bind(type, keyseq, data, map) {
  return call('rl_generic_bind', type, keyseq, data, map);
}
