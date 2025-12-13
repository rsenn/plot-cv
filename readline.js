import { call, define, dlopen, dlsym, RTLD_NOW } from 'ffi';

const libreadline = dlopen('libreadline.so.8', RTLD_NOW);

/**
 * @function readline
 *
 * @param    {String}        prompt
 *
 * @return   {String}
 */
define('readline', dlsym(libreadline, 'readline'), null, 'string', 'string');

export function readline(prompt) {
  return call('readline', prompt);
}

/**
 * @function abort
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_abort', dlsym(libreadline, 'rl_abort'), null, 'int', 'int', 'int');

export function abort(count, key) {
  return call('rl_abort', count, key);
}

/**
 * @function add_defun
 *
 * @param    {String}        name
 * @param    {Number}        func230
 *
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_add_defun', dlsym(libreadline, 'rl_add_defun'), null, 'int', 'char *', 'void *', 'int');
export function add_defun(name, func, key) {
  return call('rl_add_defun', name, func, key);
}

/**
 * @function add_funmap_entry
 *
 * @param    {String}        name
 * @param    {Number}        function
 *
 * @return   {Number}
 */
define('rl_add_funmap_entry', dlsym(libreadline, 'rl_add_funmap_entry'), null, 'int', 'char *', 'void *');

export function add_funmap_entry(name, func) {
  return call('rl_add_funmap_entry', name, func);
}

/**
 * @function add_undo
 *
 * @param    {Number}        what
 * @param    {Number}        start
 * @param    {Number}        end
 * @param    {String}        text
 *
 * @return   {Number}
 */
define('rl_add_undo', dlsym(libreadline, 'rl_add_undo'), null, 'void', 'int', 'int', 'int', 'char *');
export function add_undo(what, start, end, text) {
  return call('rl_add_undo', what, start, end, text);
}

/**
 * @function alphabetic
 *
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_alphabetic', dlsym(libreadline, 'rl_alphabetic'), null, 'int', 'int');
export function alphabetic(c) {
  return call('rl_alphabetic', c);
}

/**
 * @function arrow_keys
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_arrow_keys', dlsym(libreadline, 'rl_arrow_keys'), null, 'int', 'int', 'int');
export function arrow_keys(count, key) {
  return call('rl_arrow_keys', count, key);
}

/**
 * @function backward
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward', dlsym(libreadline, 'rl_backward'), null, 'int', 'int', 'int');
export function backward(count, key) {
  return call('rl_backward', count, key);
}

/**
 * @function backward_byte
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_byte', dlsym(libreadline, 'rl_backward_byte'), null, 'int', 'int', 'int');
export function backward_byte(count, key) {
  return call('rl_backward_byte', count, key);
}

/**
 * @function backwardChar
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_char', dlsym(libreadline, 'rl_backward_char'), null, 'int', 'int', 'int');
export function backwardChar(count, key) {
  return call('rl_backward_char', count, key);
}

/**
 * @function backward_char_search
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_char_search', dlsym(libreadline, 'rl_backward_char_search'), null, 'int', 'int', 'int');
export function backward_char_search(count, key) {
  return call('rl_backward_char_search', count, key);
}

/**
 * @function backwardKillLine
 *
 * @param    {Number}        direction
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_kill_line', dlsym(libreadline, 'rl_backward_kill_line'), null, 'int', 'int', 'int');
export function backwardKillLine(direction, key) {
  return call('rl_backward_kill_line', direction, key);
}

/**
 * @function backwardKillWord
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_kill_word', dlsym(libreadline, 'rl_backward_kill_word'), null, 'int', 'int', 'int');
export function backwardKillWord(count, key) {
  return call('rl_backward_kill_word', count, key);
}

/**
 * @function backward_menu_complete
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_menu_complete', dlsym(libreadline, 'rl_backward_menu_complete'), null, 'int', 'int', 'int');
export function backward_menu_complete(count, key) {
  return call('rl_backward_menu_complete', count, key);
}

/**
 * @function backwardWord
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_backward_word', dlsym(libreadline, 'rl_backward_word'), null, 'int', 'int', 'int');
export function backwardWord(count, key) {
  return call('rl_backward_word', count, key);
}

/**
 * @function beg_of_line
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_beg_of_line', dlsym(libreadline, 'rl_beg_of_line'), null, 'int', 'int', 'int');
export function beg_of_line(count, key) {
  return call('rl_beg_of_line', count, key);
}

/**
 * @function begin_undo_group
 *
 * @return   {Number}
 */
define('rl_begin_undo_group', dlsym(libreadline, 'rl_begin_undo_group'), null, 'int');
export function begin_undo_group() {
  return call('rl_begin_undo_group');
}

/**
 * @function beginning_of_history
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_beginning_of_history', dlsym(libreadline, 'rl_beginning_of_history'), null, 'int', 'int', 'int');
export function beginning_of_history(count, key) {
  return call('rl_beginning_of_history', count, key);
}

/**
 * @function bind_key
 *
 * @param    {Number}        key
 * @param    {Number}        function
 *
 * @return   {Number}
 */
define('rl_bind_key', dlsym(libreadline, 'rl_bind_key'), null, 'int', 'int', 'void *');
export function bind_key(key, func) {
  return call('rl_bind_key', key, func);
}

/**
 * @function bind_key_if_unbound
 *
 * @param    {Number}        key
 * @param    {Number}        default_func
 *
 * @return   {Number}
 */
define('rl_bind_key_if_unbound', dlsym(libreadline, 'rl_bind_key_if_unbound'), null, 'int', 'int', 'void *');
export function bind_key_if_unbound(key, default_func) {
  return call('rl_bind_key_if_unbound', key, default_func);
}

/**
 * @function bind_key_if_unbound_in_map
 *
 * @param    {Number}        key
 * @param    {Number}        default_func
 * @param    {Number}        kmap
 *
 * @return   {Number}
 */
define('rl_bind_key_if_unbound_in_map', dlsym(libreadline, 'rl_bind_key_if_unbound_in_map'), null, 'int', 'int', 'void *', 'long');
export function bind_key_if_unbound_in_map(key, default_func, kmap) {
  return call('rl_bind_key_if_unbound_in_map', key, default_func, kmap);
}

/**
 * @function bind_key_in_map
 *
 * @param    {Number}        key
 * @param    {Number}        function
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_bind_key_in_map', dlsym(libreadline, 'rl_bind_key_in_map'), null, 'int', 'int', 'void *', 'long');
export function bind_key_in_map(key, func, map) {
  return call('rl_bind_key_in_map', key, func, map);
}

/**
 * @function bind_keyseq
 *
 * @param    {String}        keyseq
 * @param    {Number}        function
 *
 * @return   {Number}
 */
define('rl_bind_keyseq', dlsym(libreadline, 'rl_bind_keyseq'), null, 'int', 'char *', 'void *');
export function bind_keyseq(keyseq, func) {
  return call('rl_bind_keyseq', keyseq, func);
}

/**
 * @function bind_keyseq_if_unbound
 *
 * @param    {String}        keyseq
 * @param    {Number}        default_func
 *
 * @return   {Number}
 */
define('rl_bind_keyseq_if_unbound', dlsym(libreadline, 'rl_bind_keyseq_if_unbound'), null, 'int', 'char *', 'void *');
export function bind_keyseq_if_unbound(keyseq, default_func) {
  return call('rl_bind_keyseq_if_unbound', keyseq, default_func);
}

/**
 * @function bind_keyseq_if_unbound_in_map
 *
 * @param    {String}        keyseq
 * @param    {Number}        default_func
 * @param    {Number}        kmap
 *
 * @return   {Number}
 */
define('rl_bind_keyseq_if_unbound_in_map', dlsym(libreadline, 'rl_bind_keyseq_if_unbound_in_map'), null, 'int', 'char *', 'void *', 'long');
export function bind_keyseq_if_unbound_in_map(keyseq, default_func, kmap) {
  return call('rl_bind_keyseq_if_unbound_in_map', keyseq, default_func, kmap);
}

/**
 * @function bind_keyseq_in_map
 *
 * @param    {String}        keyseq
 * @param    {Number}        function
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_bind_keyseq_in_map', dlsym(libreadline, 'rl_bind_keyseq_in_map'), null, 'int', 'char *', 'void *', 'long');
export function bind_keyseq_in_map(keyseq, func, map) {
  return call('rl_bind_keyseq_in_map', keyseq, func, map);
}

/**
 * @function bracketed_paste_begin
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_bracketed_paste_begin', dlsym(libreadline, 'rl_bracketed_paste_begin'), null, 'int', 'int', 'int');
export function bracketed_paste_begin(count, key) {
  return call('rl_bracketed_paste_begin', count, key);
}

/**
 * @function call_last_kbd_macro
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_call_last_kbd_macro', dlsym(libreadline, 'rl_call_last_kbd_macro'), null, 'int', 'int', 'int');
export function call_last_kbd_macro(count, ignore) {
  return call('rl_call_last_kbd_macro', count, ignore);
}

/**
 * @function callback_handler_install
 *
 * @param    {String}        prompt
 * @param    {Number}        linefunc
 *
 * @return   {Number}
 */
define('rl_callback_handler_install', dlsym(libreadline, 'rl_callback_handler_install'), null, 'void', 'char *', 'void *');
export function callback_handler_install(prompt, linefunc) {
  return call('rl_callback_handler_install', prompt, linefunc);
}

/**
 * @function callback_handler_remove
 *
 * @return   {Number}
 */
define('rl_callback_handler_remove', dlsym(libreadline, 'rl_callback_handler_remove'), null, 'void');
export function callback_handler_remove() {
  return call('rl_callback_handler_remove');
}

/**
 * @function callback_read_char
 *
 * @return   {Number}
 */
define('rl_callback_read_char', dlsym(libreadline, 'rl_callback_read_char'), null, 'void');
export function callback_read_char() {
  return call('rl_callback_read_char');
}

/**
 * @function callback_sigcleanup
 *
 * @return   {Number}
 */
define('rl_callback_sigcleanup', dlsym(libreadline, 'rl_callback_sigcleanup'), null, 'void');
export function callback_sigcleanup() {
  return call('rl_callback_sigcleanup');
}

/**
 * @function capitalize_word
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_capitalize_word', dlsym(libreadline, 'rl_capitalize_word'), null, 'int', 'int', 'int');
export function capitalize_word(count, key) {
  return call('rl_capitalize_word', count, key);
}

/**
 * @function char_search
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_char_search', dlsym(libreadline, 'rl_char_search'), null, 'int', 'int', 'int');
export function char_search(count, key) {
  return call('rl_char_search', count, key);
}

/**
 * @function character_len
 *
 * @param    {Number}        c
 * @param    {Number}        pos
 *
 * @return   {Number}
 */
define('rl_character_len', dlsym(libreadline, 'rl_character_len'), null, 'int', 'int', 'int');
export function character_len(c, pos) {
  return call('rl_character_len', c, pos);
}

/**
 * @function check_signals
 *
 * @return   {Number}
 */
define('rl_check_signals', dlsym(libreadline, 'rl_check_signals'), null, 'void');
export function check_signals() {
  return call('rl_check_signals');
}

/**
 * @function cleanup_after_signal
 *
 * @return   {Number}
 */
define('rl_cleanup_after_signal', dlsym(libreadline, 'rl_cleanup_after_signal'), null, 'void');
export function cleanup_after_signal() {
  return call('rl_cleanup_after_signal');
}

/**
 * @function clear_history
 *
 * @return   {Number}
 */
define('rl_clear_history', dlsym(libreadline, 'rl_clear_history'), null, 'void');
export function clear_history() {
  return call('rl_clear_history');
}

/**
 * @function clear_message
 *
 * @return   {Number}
 */
define('rl_clear_message', dlsym(libreadline, 'rl_clear_message'), null, 'int');
export function clear_message() {
  return call('rl_clear_message');
}

/**
 * @function clear_pending_input
 *
 * @return   {Number}
 */
define('rl_clear_pending_input', dlsym(libreadline, 'rl_clear_pending_input'), null, 'int');
export function clear_pending_input() {
  return call('rl_clear_pending_input');
}

/**
 * @function clear_screen
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_clear_screen', dlsym(libreadline, 'rl_clear_screen'), null, 'int', 'int', 'int');
export function clear_screen(count, key) {
  return call('rl_clear_screen', count, key);
}

/**
 * @function clear_signals
 *
 * @return   {Number}
 */
define('rl_clear_signals', dlsym(libreadline, 'rl_clear_signals'), null, 'int');
export function clear_signals() {
  return call('rl_clear_signals');
}

/**
 * @function clear_visible_line
 *
 * @return   {Number}
 */
define('rl_clear_visible_line', dlsym(libreadline, 'rl_clear_visible_line'), null, 'int');
export function clear_visible_line() {
  return call('rl_clear_visible_line');
}

/**
 * @function complete
 *
 * @param    {Number}        ignore
 * @param    {Number}        invoking_key
 *
 * @return   {Number}
 */
define('rl_complete', dlsym(libreadline, 'rl_complete'), null, 'int', 'int', 'int');
export function complete(ignore, invoking_key) {
  return call('rl_complete', ignore, invoking_key);
}

/**
 * @function complete_internal
 *
 * @param    {Number}        what_to_do
 *
 * @return   {Number}
 */
define('rl_complete_internal', dlsym(libreadline, 'rl_complete_internal'), null, 'int', 'int');
export function complete_internal(what_to_do) {
  return call('rl_complete_internal', what_to_do);
}

/**
 * @function completion_matches
 *
 * @param    {String}        text
 * @param    {Number}        entry_function
 *
 * @return   {Number}
 */
define('rl_completion_matches', dlsym(libreadline, 'rl_completion_matches'), null, 'void *', 'char *', 'void *');
export function completion_matches(text, entry_function) {
  return call('rl_completion_matches', text, entry_function);
}

/**
 * @function completion_mode
 *
 * @param    {Number}        cfunc
 *
 * @return   {Number}
 */
define('rl_completion_mode', dlsym(libreadline, 'rl_completion_mode'), null, 'int', 'void *');
export function completion_mode(cfunc) {
  return call('rl_completion_mode', cfunc);
}

/**
 * @function copy_backward_word
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_copy_backward_word', dlsym(libreadline, 'rl_copy_backward_word'), null, 'int', 'int', 'int');
export function copy_backward_word(count, key) {
  return call('rl_copy_backward_word', count, key);
}

/**
 * @function copy_forward_word
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_copy_forward_word', dlsym(libreadline, 'rl_copy_forward_word'), null, 'int', 'int', 'int');
export function copy_forward_word(count, key) {
  return call('rl_copy_forward_word', count, key);
}

/**
 * @function copy_region_to_kill
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_copy_region_to_kill', dlsym(libreadline, 'rl_copy_region_to_kill'), null, 'int', 'int', 'int');
export function copy_region_to_kill(count, key) {
  return call('rl_copy_region_to_kill', count, key);
}

/**
 * @function copy_text
 *
 * @param    {Number}        from
 * @param    {Number}        to
 *
 * @return   {String}
 */
define('rl_copy_text', dlsym(libreadline, 'rl_copy_text'), null, 'char *', 'int', 'int');
export function copy_text(from, to) {
  return call('rl_copy_text', from, to);
}

/**
 * @function crlf
 *
 * @return   {Number}
 */
define('rl_crlf', dlsym(libreadline, 'rl_crlf'), null, 'int');
export function crlf() {
  return call('rl_crlf');
}

/**
 * @function delete
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_delete', dlsym(libreadline, 'rl_delete'), null, 'int', 'int', 'int');
export function rl_delete(count, key) {
  return call('rl_delete', count, key);
}

/**
 * @function delete_horizontal_space
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_delete_horizontal_space', dlsym(libreadline, 'rl_delete_horizontal_space'), null, 'int', 'int', 'int');
export function delete_horizontal_space(count, ignore) {
  return call('rl_delete_horizontal_space', count, ignore);
}

/**
 * @function delete_or_show_completions
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_delete_or_show_completions', dlsym(libreadline, 'rl_delete_or_show_completions'), null, 'int', 'int', 'int');
export function delete_or_show_completions(count, key) {
  return call('rl_delete_or_show_completions', count, key);
}

/**
 * @function delete_text
 *
 * @param    {Number}        from
 * @param    {Number}        to
 *
 * @return   {Number}
 */
define('rl_delete_text', dlsym(libreadline, 'rl_delete_text'), null, 'int', 'int', 'int');
export function delete_text(from, to) {
  return call('rl_delete_text', from, to);
}

/**
 * @function deprep_terminal
 *
 * @return   {Number}
 */
define('rl_deprep_terminal', dlsym(libreadline, 'rl_deprep_terminal'), null, 'void');
export function deprep_terminal() {
  return call('rl_deprep_terminal');
}

/**
 * @function digit_argument
 *
 * @param    {Number}        ignore
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_digit_argument', dlsym(libreadline, 'rl_digit_argument'), null, 'int', 'int', 'int');
export function digit_argument(ignore, key) {
  return call('rl_digit_argument', ignore, key);
}

/**
 * @function ding
 *
 * @return   {Number}
 */
define('rl_ding', dlsym(libreadline, 'rl_ding'), null, 'int');
export function ding() {
  return call('rl_ding');
}

/**
 * @function discard_argument
 *
 * @return   {Number}
 */
define('rl_discard_argument', dlsym(libreadline, 'rl_discard_argument'), null, 'int');
export function discard_argument() {
  return call('rl_discard_argument');
}

/**
 * @function discard_keymap
 *
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_discard_keymap', dlsym(libreadline, 'rl_discard_keymap'), null, 'void', 'long');
export function discard_keymap(map) {
  return call('rl_discard_keymap', map);
}

/**
 * @function display_match_list
 *
 * @param    {Number}        matches
 * @param    {Number}        len
 * @param    {Number}        max
 *
 * @return   {Number}
 */
define('rl_display_match_list', dlsym(libreadline, 'rl_display_match_list'), null, 'void', 'void *', 'int', 'int');
export function display_match_list(matches, len, max) {
  return call('rl_display_match_list', matches, len, max);
}

/**
 * @function do_lowercase_version
 *
 * @param    {Number}        ignore1
 * @param    {Number}        ignore2
 *
 * @return   {Number}
 */
define('rl_do_lowercase_version', dlsym(libreadline, 'rl_do_lowercase_version'), null, 'int', 'int', 'int');
export function do_lowercase_version(ignore1, ignore2) {
  return call('rl_do_lowercase_version', ignore1, ignore2);
}

/**
 * @function do_undo
 *
 * @return   {Number}
 */
define('rl_do_undo', dlsym(libreadline, 'rl_do_undo'), null, 'int');
export function do_undo() {
  return call('rl_do_undo');
}

/**
 * @function downcaseWord
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_downcase_word', dlsym(libreadline, 'rl_downcase_word'), null, 'int', 'int', 'int');
export function downcaseWord(count, key) {
  return call('rl_downcase_word', count, key);
}

/**
 * @function dump_functions
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_dump_functions', dlsym(libreadline, 'rl_dump_functions'), null, 'int', 'int', 'int');
export function dump_functions(count, key) {
  return call('rl_dump_functions', count, key);
}

/**
 * @function dump_macros
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_dump_macros', dlsym(libreadline, 'rl_dump_macros'), null, 'int', 'int', 'int');
export function dump_macros(count, key) {
  return call('rl_dump_macros', count, key);
}

/**
 * @function dump_variables
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_dump_variables', dlsym(libreadline, 'rl_dump_variables'), null, 'int', 'int', 'int');
export function dump_variables(count, key) {
  return call('rl_dump_variables', count, key);
}

/**
 * @function echo_signal_char
 *
 * @param    {Number}        sig
 *
 * @return   {Number}
 */
define('rl_echo_signal_char', dlsym(libreadline, 'rl_echo_signal_char'), null, 'void', 'int');
export function echo_signal_char(sig) {
  return call('rl_echo_signal_char', sig);
}

/**
 * @function emacs_editing_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_emacs_editing_mode', dlsym(libreadline, 'rl_emacs_editing_mode'), null, 'int', 'int', 'int');
export function emacs_editing_mode(count, key) {
  return call('rl_emacs_editing_mode', count, key);
}

/**
 * @function empty_keymap
 *
 * @param    {Number}        keymap
 *
 * @return   {Number}
 */
define('rl_empty_keymap', dlsym(libreadline, 'rl_empty_keymap'), null, 'int', 'long');
export function empty_keymap(keymap) {
  return call('rl_empty_keymap', keymap);
}

/**
 * @function end_kbd_macro
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_end_kbd_macro', dlsym(libreadline, 'rl_end_kbd_macro'), null, 'int', 'int', 'int');
export function end_kbd_macro(count, ignore) {
  return call('rl_end_kbd_macro', count, ignore);
}

/**
 * @function end_of_history
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_end_of_history', dlsym(libreadline, 'rl_end_of_history'), null, 'int', 'int', 'int');
export function end_of_history(count, key) {
  return call('rl_end_of_history', count, key);
}

/**
 * @function endOfLine
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_end_of_line', dlsym(libreadline, 'rl_end_of_line'), null, 'int', 'int', 'int');
export function endOfLine(count, key) {
  return call('rl_end_of_line', count, key);
}

/**
 * @function end_undo_group
 *
 * @return   {Number}
 */
define('rl_end_undo_group', dlsym(libreadline, 'rl_end_undo_group'), null, 'int');
export function end_undo_group() {
  return call('rl_end_undo_group');
}

/**
 * @function exchange_point_and_mark
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_exchange_point_and_mark', dlsym(libreadline, 'rl_exchange_point_and_mark'), null, 'int', 'int', 'int');
export function exchange_point_and_mark(count, key) {
  return call('rl_exchange_point_and_mark', count, key);
}

/**
 * @function execute_next
 *
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_execute_next', dlsym(libreadline, 'rl_execute_next'), null, 'int', 'int');
export function execute_next(c) {
  return call('rl_execute_next', c);
}

/**
 * @function expand_prompt
 *
 * @param    {String}        prompt
 *
 * @return   {Number}
 */
define('rl_expand_prompt', dlsym(libreadline, 'rl_expand_prompt'), null, 'int', 'char *');
export function expand_prompt(prompt) {
  return call('rl_expand_prompt', prompt);
}

/**
 * @function extend_line_buffer
 *
 * @param    {Number}        len
 *
 * @return   {Number}
 */
define('rl_extend_line_buffer', dlsym(libreadline, 'rl_extend_line_buffer'), null, 'void', 'int');
export function extend_line_buffer(len) {
  return call('rl_extend_line_buffer', len);
}

/**
 * @function forced_update_display
 *
 * @return   {Number}
 */
define('rl_forced_update_display', dlsym(libreadline, 'rl_forced_update_display'), null, 'int');
export function forced_update_display() {
  return call('rl_forced_update_display');
}

/**
 * @function forward
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_forward', dlsym(libreadline, 'rl_forward'), null, 'int', 'int', 'int');
export function forward(count, key) {
  return call('rl_forward', count, key);
}

/**
 * @function forward_byte
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_forward_byte', dlsym(libreadline, 'rl_forward_byte'), null, 'int', 'int', 'int');
export function forward_byte(count, key) {
  return call('rl_forward_byte', count, key);
}

/**
 * @function forwardChar
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_forward_char', dlsym(libreadline, 'rl_forward_char'), null, 'int', 'int', 'int');
export function forwardChar(count, key) {
  return call('rl_forward_char', count, key);
}

/**
 * @function forward_search_history
 *
 * @param    {Number}        sign
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_forward_search_history', dlsym(libreadline, 'rl_forward_search_history'), null, 'int', 'int', 'int');
export function forward_search_history(sign, key) {
  return call('rl_forward_search_history', sign, key);
}

/**
 * @function forwardWord
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_forward_word', dlsym(libreadline, 'rl_forward_word'), null, 'int', 'int', 'int');
export function forwardWord(count, key) {
  return call('rl_forward_word', count, key);
}

/**
 * @function free
 *
 * @param    {Number}        mem
 *
 * @return   {Number}
 */
define('rl_free', dlsym(libreadline, 'rl_free'), null, 'void', 'void *');
export function free(mem) {
  return call('rl_free', mem);
}

/**
 * @function free_keymap
 *
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_free_keymap', dlsym(libreadline, 'rl_free_keymap'), null, 'void', 'long');
export function free_keymap(map) {
  return call('rl_free_keymap', map);
}

/**
 * @function free_line_state
 *
 * @return   {Number}
 */
define('rl_free_line_state', dlsym(libreadline, 'rl_free_line_state'), null, 'void');
export function free_line_state() {
  return call('rl_free_line_state');
}

/**
 * @function free_undo_list
 *
 * @return   {Number}
 */
define('rl_free_undo_list', dlsym(libreadline, 'rl_free_undo_list'), null, 'void');
export function free_undo_list() {
  return call('rl_free_undo_list');
}

/**
 * @function function_dumper
 *
 * @param    {Number}        print_readably
 *
 * @return   {Number}
 */
define('rl_function_dumper', dlsym(libreadline, 'rl_function_dumper'), null, 'void', 'int');
export function function_dumper(print_readably) {
  return call('rl_function_dumper', print_readably);
}

/**
 * @function function_of_keyseq
 *
 * @param    {String}        keyseq
 * @param    {Number}        map
 * @param    {Number}        type
 *
 * @return   {Number}
 */
define('rl_function_of_keyseq', dlsym(libreadline, 'rl_function_of_keyseq'), null, 'void *', 'char *', 'long', 'void *');
export function function_of_keyseq(keyseq, map, type) {
  return call('rl_function_of_keyseq', keyseq, map, type);
}

/**
 * @function function_of_keyseq_len
 *
 * @param    {String}        keyseq
 * @param    {Number}        len
 * @param    {Number}        map
 * @param    {Number}        type
 *
 * @return   {Number}
 */
define('rl_function_of_keyseq_len', dlsym(libreadline, 'rl_function_of_keyseq_len'), null, 'void *', 'char *', 'size_t', 'long', 'void *');
export function function_of_keyseq_len(keyseq, len, map, type) {
  return call('rl_function_of_keyseq_len', keyseq, len, map, type);
}

/**
 * @function funmap_names
 *
 * @return   {Number}
 */
define('rl_funmap_names', dlsym(libreadline, 'rl_funmap_names'), null, 'void *');
export function funmap_names() {
  return call('rl_funmap_names');
}

/**
 * @function generic_bind
 *
 * @param    {Number}        type
 * @param    {String}        keyseq
 * @param    {String}        data
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_generic_bind', dlsym(libreadline, 'rl_generic_bind'), null, 'int', 'int', 'char *', 'char *', 'long');
export function generic_bind(type, keyseq, data, map) {
  return call('rl_generic_bind', type, keyseq, data, map);
}

/**
 * @function get_keymap_name
 *
 * @param    {Number}        map
 *
 * @return   {String}
 */
define('rl_get_keymap_name', dlsym(libreadline, 'rl_get_keymap_name'), null, 'char *', 'long');
export function get_keymap_name(map) {
  return call('rl_get_keymap_name', map);
}

/**
 * @function get_keymap_name_from_edit_mode
 *
 * @return   {String}
 */
define('rl_get_keymap_name_from_edit_mode', dlsym(libreadline, 'rl_get_keymap_name_from_edit_mode'), null, 'char *');
export function get_keymap_name_from_edit_mode() {
  return call('rl_get_keymap_name_from_edit_mode');
}

/**
 * @function get_next_history
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_get_next_history', dlsym(libreadline, 'rl_get_next_history'), null, 'int', 'int', 'int');
export function get_next_history(count, key) {
  return call('rl_get_next_history', count, key);
}

/**
 * @function get_previous_history
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_get_previous_history', dlsym(libreadline, 'rl_get_previous_history'), null, 'int', 'int', 'int');
export function get_previous_history(count, key) {
  return call('rl_get_previous_history', count, key);
}

/**
 * @function get_screen_size
 *
 * @param    {Number}        rows
 * @param    {Number}        cols
 *
 * @return   {Number}
 */
define('rl_get_screen_size', dlsym(libreadline, 'rl_get_screen_size'), null, 'void', 'void *', 'void *');
export function get_screen_size(rows, cols) {
  return call('rl_get_screen_size', rows, cols);
}

/**
 * @function get_termcap
 *
 * @param    {String}        cap
 *
 * @return   {String}
 */
define('rl_get_termcap', dlsym(libreadline, 'rl_get_termcap'), null, 'char *', 'char *');
export function get_termcap(cap) {
  return call('rl_get_termcap', cap);
}

/**
 * @function getc
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('rl_getc', dlsym(libreadline, 'rl_getc'), null, 'int', 'void *');
export function getc(stream) {
  return call('rl_getc', stream);
}

/**
 * @function historySearchBackward
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_history_search_backward', dlsym(libreadline, 'rl_history_search_backward'), null, 'int', 'int', 'int');
export function historySearchBackward(count, ignore) {
  return call('rl_history_search_backward', count, ignore);
}

/**
 * @function historySearchForward
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_history_search_forward', dlsym(libreadline, 'rl_history_search_forward'), null, 'int', 'int', 'int');
export function historySearchForward(count, ignore) {
  return call('rl_history_search_forward', count, ignore);
}

/**
 * @function history_substr_search_backward
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_history_substr_search_backward', dlsym(libreadline, 'rl_history_substr_search_backward'), null, 'int', 'int', 'int');
export function history_substr_search_backward(count, ignore) {
  return call('rl_history_substr_search_backward', count, ignore);
}

/**
 * @function history_substr_search_forward
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_history_substr_search_forward', dlsym(libreadline, 'rl_history_substr_search_forward'), null, 'int', 'int', 'int');
export function history_substr_search_forward(count, ignore) {
  return call('rl_history_substr_search_forward', count, ignore);
}

/**
 * @function initialize
 *
 * @return   {Number}
 */
define('rl_initialize', dlsym(libreadline, 'rl_initialize'), null, 'int');
export function initialize() {
  return call('rl_initialize');
}

/**
 * @function initialize_funmap
 *
 * @return   {Number}
 */
define('rl_initialize_funmap', dlsym(libreadline, 'rl_initialize_funmap'), null, 'void');
export function initialize_funmap() {
  return call('rl_initialize_funmap');
}

/**
 * @function insert
 *
 * @param    {Number}        count
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_insert', dlsym(libreadline, 'rl_insert'), null, 'int', 'int', 'int');
export function insert(count, c) {
  return call('rl_insert', count, c);
}

/**
 * @function insert_close
 *
 * @param    {Number}        count
 * @param    {Number}        invoking_key
 *
 * @return   {Number}
 */
define('rl_insert_close', dlsym(libreadline, 'rl_insert_close'), null, 'int', 'int', 'int');
export function insert_close(count, invoking_key) {
  return call('rl_insert_close', count, invoking_key);
}

/**
 * @function insert_comment
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_insert_comment', dlsym(libreadline, 'rl_insert_comment'), null, 'int', 'int', 'int');
export function insert_comment(count, key) {
  return call('rl_insert_comment', count, key);
}

/**
 * @function insert_completions
 *
 * @param    {Number}        ignore
 * @param    {Number}        invoking_key
 *
 * @return   {Number}
 */
define('rl_insert_completions', dlsym(libreadline, 'rl_insert_completions'), null, 'int', 'int', 'int');
export function insert_completions(ignore, invoking_key) {
  return call('rl_insert_completions', ignore, invoking_key);
}

/**
 * @function insert_text
 *
 * @param    {String}        string
 *
 * @return   {Number}
 */
define('rl_insert_text', dlsym(libreadline, 'rl_insert_text'), null, 'int', 'char *');
export function insert_text(string) {
  return call('rl_insert_text', string);
}

/**
 * @function invoking_keyseqs
 *
 * @param    {Number}        function
 *
 * @return   {Number}
 */
define('rl_invoking_keyseqs', dlsym(libreadline, 'rl_invoking_keyseqs'), null, 'void *', 'void *');
export function invoking_keyseqs(func) {
  return call('rl_invoking_keyseqs', func);
}

/**
 * @function invoking_keyseqs_in_map
 *
 * @param    {Number}        function
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_invoking_keyseqs_in_map', dlsym(libreadline, 'rl_invoking_keyseqs_in_map'), null, 'void *', 'void *', 'long');
export function invoking_keyseqs_in_map(func, map) {
  return call('rl_invoking_keyseqs_in_map', func, map);
}

/**
 * @function kill_full_line
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_kill_full_line', dlsym(libreadline, 'rl_kill_full_line'), null, 'int', 'int', 'int');
export function kill_full_line(count, key) {
  return call('rl_kill_full_line', count, key);
}

/**
 * @function killLine
 *
 * @param    {Number}        direction
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_kill_line', dlsym(libreadline, 'rl_kill_line'), null, 'int', 'int', 'int');
export function killLine(direction, key) {
  return call('rl_kill_line', direction, key);
}

/**
 * @function killRegion
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_kill_region', dlsym(libreadline, 'rl_kill_region'), null, 'int', 'int', 'int');
export function killRegion(count, key) {
  return call('rl_kill_region', count, key);
}

/**
 * @function kill_text
 *
 * @param    {Number}        from
 * @param    {Number}        to
 *
 * @return   {Number}
 */
define('rl_kill_text', dlsym(libreadline, 'rl_kill_text'), null, 'int', 'int', 'int');
export function kill_text(from, to) {
  return call('rl_kill_text', from, to);
}

/**
 * @function killWord
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_kill_word', dlsym(libreadline, 'rl_kill_word'), null, 'int', 'int', 'int');
export function killWord(count, key) {
  return call('rl_kill_word', count, key);
}

/**
 * @function list_funmap_names
 *
 * @return   {Number}
 */
define('rl_list_funmap_names', dlsym(libreadline, 'rl_list_funmap_names'), null, 'void');
export function list_funmap_names() {
  return call('rl_list_funmap_names');
}

/**
 * @function macro_bind
 *
 * @param    {String}        keyseq
 * @param    {String}        macro
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_macro_bind', dlsym(libreadline, 'rl_macro_bind'), null, 'int', 'char *', 'char *', 'long');
export function macro_bind(keyseq, macro, map) {
  return call('rl_macro_bind', keyseq, macro, map);
}

/**
 * @function macro_dumper
 *
 * @param    {Number}        print_readably
 *
 * @return   {Number}
 */
define('rl_macro_dumper', dlsym(libreadline, 'rl_macro_dumper'), null, 'void', 'int');
export function macro_dumper(print_readably) {
  return call('rl_macro_dumper', print_readably);
}

/**
 * @function maybe_replace_line
 *
 * @return   {Number}
 */
define('rl_maybe_replace_line', dlsym(libreadline, 'rl_maybe_replace_line'), null, 'int');
export function maybe_replace_line() {
  return call('rl_maybe_replace_line');
}

/**
 * @function maybe_save_line
 *
 * @return   {Number}
 */
define('rl_maybe_save_line', dlsym(libreadline, 'rl_maybe_save_line'), null, 'int');
export function maybe_save_line() {
  return call('rl_maybe_save_line');
}

/**
 * @function maybe_unsave_line
 *
 * @return   {Number}
 */
define('rl_maybe_unsave_line', dlsym(libreadline, 'rl_maybe_unsave_line'), null, 'int');
export function maybe_unsave_line() {
  return call('rl_maybe_unsave_line');
}

/**
 * @function menu_complete
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_menu_complete', dlsym(libreadline, 'rl_menu_complete'), null, 'int', 'int', 'int');
export function menu_complete(count, ignore) {
  return call('rl_menu_complete', count, ignore);
}

/**
 * @function message
 *
 * @param    {String}        format
 * @param    {Number}        arg1
 * @param    {Number}        arg2
 *
 * @return   {Number}
 */
define('rl_message', dlsym(libreadline, 'rl_message'), null, 'int', 'char *', 'int', 'int');
export function message(format, arg1, arg2) {
  return call('rl_message', format, arg1, arg2);
}

/**
 * @function modifying
 *
 * @param    {Number}        start
 * @param    {Number}        end
 *
 * @return   {Number}
 */
define('rl_modifying', dlsym(libreadline, 'rl_modifying'), null, 'int', 'int', 'int');
export function modifying(start, end) {
  return call('rl_modifying', start, end);
}

/**
 * @function named_function
 *
 * @param    {String}        string
 *
 * @return   {Number}
 */
define('rl_named_function', dlsym(libreadline, 'rl_named_function'), null, 'void *', 'char *');
export function named_function(string) {
  return call('rl_named_function', string);
}

/**
 * @function newline
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_newline', dlsym(libreadline, 'rl_newline'), null, 'int', 'int', 'int');
export function newline(count, key) {
  return call('rl_newline', count, key);
}

/**
 * @function next_screen_line
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_next_screen_line', dlsym(libreadline, 'rl_next_screen_line'), null, 'int', 'int', 'int');
export function next_screen_line(count, key) {
  return call('rl_next_screen_line', count, key);
}

/**
 * @function noninc_forward_search
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_noninc_forward_search', dlsym(libreadline, 'rl_noninc_forward_search'), null, 'int', 'int', 'int');
export function noninc_forward_search(count, key) {
  return call('rl_noninc_forward_search', count, key);
}

/**
 * @function noninc_forward_search_again
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_noninc_forward_search_again', dlsym(libreadline, 'rl_noninc_forward_search_again'), null, 'int', 'int', 'int');
export function noninc_forward_search_again(count, key) {
  return call('rl_noninc_forward_search_again', count, key);
}

/**
 * @function noninc_reverse_search
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_noninc_reverse_search', dlsym(libreadline, 'rl_noninc_reverse_search'), null, 'int', 'int', 'int');
export function noninc_reverse_search(count, key) {
  return call('rl_noninc_reverse_search', count, key);
}

/**
 * @function noninc_reverse_search_again
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_noninc_reverse_search_again', dlsym(libreadline, 'rl_noninc_reverse_search_again'), null, 'int', 'int', 'int');
export function noninc_reverse_search_again(count, key) {
  return call('rl_noninc_reverse_search_again', count, key);
}

/**
 * @function old_menu_complete
 *
 * @param    {Number}        count
 * @param    {Number}        invoking_key
 *
 * @return   {Number}
 */
define('rl_old_menu_complete', dlsym(libreadline, 'rl_old_menu_complete'), null, 'int', 'int', 'int');
export function old_menu_complete(count, invoking_key) {
  return call('rl_old_menu_complete', count, invoking_key);
}

/**
 * @function on_new_line
 *
 * @return   {Number}
 */
define('rl_on_new_line', dlsym(libreadline, 'rl_on_new_line'), null, 'int');
export function on_new_line() {
  return call('rl_on_new_line');
}

/**
 * @function on_new_line_with_prompt
 *
 * @return   {Number}
 */
define('rl_on_new_line_with_prompt', dlsym(libreadline, 'rl_on_new_line_with_prompt'), null, 'int');
export function on_new_line_with_prompt() {
  return call('rl_on_new_line_with_prompt');
}

/**
 * @function overwrite_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_overwrite_mode', dlsym(libreadline, 'rl_overwrite_mode'), null, 'int', 'int', 'int');
export function overwrite_mode(count, key) {
  return call('rl_overwrite_mode', count, key);
}

/**
 * @function parse_and_bind
 *
 * @param    {String}        string
 *
 * @return   {Number}
 */
define('rl_parse_and_bind', dlsym(libreadline, 'rl_parse_and_bind'), null, 'int', 'char *');
export function parse_and_bind(string) {
  return call('rl_parse_and_bind', string);
}

/**
 * @function pending_signal
 *
 * @return   {Number}
 */
define('rl_pending_signal', dlsym(libreadline, 'rl_pending_signal'), null, 'int');
export function pending_signal() {
  return call('rl_pending_signal');
}

/**
 * @function possible_completions
 *
 * @param    {Number}        ignore
 * @param    {Number}        invoking_key
 *
 * @return   {Number}
 */
define('rl_possible_completions', dlsym(libreadline, 'rl_possible_completions'), null, 'int', 'int', 'int');
export function possible_completions(ignore, invoking_key) {
  return call('rl_possible_completions', ignore, invoking_key);
}

/**
 * @function prep_terminal
 *
 * @param    {Number}        meta_flag
 *
 * @return   {Number}
 */
define('rl_prep_terminal', dlsym(libreadline, 'rl_prep_terminal'), null, 'void', 'int');
export function prep_terminal(meta_flag) {
  return call('rl_prep_terminal', meta_flag);
}

/**
 * @function previous_screen_line
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_previous_screen_line', dlsym(libreadline, 'rl_previous_screen_line'), null, 'int', 'int', 'int');
export function previous_screen_line(count, key) {
  return call('rl_previous_screen_line', count, key);
}

/**
 * @function print_last_kbd_macro
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_print_last_kbd_macro', dlsym(libreadline, 'rl_print_last_kbd_macro'), null, 'int', 'int', 'int');
export function print_last_kbd_macro(count, ignore) {
  return call('rl_print_last_kbd_macro', count, ignore);
}

/**
 * @function push_macro_input
 *
 * @param    {String}        macro
 *
 * @return   {Number}
 */
define('rl_push_macro_input', dlsym(libreadline, 'rl_push_macro_input'), null, 'void', 'char *');
export function push_macro_input(macro) {
  return call('rl_push_macro_input', macro);
}

/**
 * @function quotedInsert
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_quoted_insert', dlsym(libreadline, 'rl_quoted_insert'), null, 'int', 'int', 'int');
export function quotedInsert(count, key) {
  return call('rl_quoted_insert', count, key);
}

/**
 * @function re_read_init_file
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_re_read_init_file', dlsym(libreadline, 'rl_re_read_init_file'), null, 'int', 'int', 'int');
export function re_read_init_file(count, ignore) {
  return call('rl_re_read_init_file', count, ignore);
}

/**
 * @function read_init_file
 *
 * @param    {String}        filename
 *
 * @return   {Number}
 */
define('rl_read_init_file', dlsym(libreadline, 'rl_read_init_file'), null, 'int', 'char *');
export function read_init_file(filename) {
  return call('rl_read_init_file', filename);
}

/**
 * @function read_key
 *
 * @return   {Number}
 */
define('rl_read_key', dlsym(libreadline, 'rl_read_key'), null, 'int');
export function read_key() {
  return call('rl_read_key');
}

/**
 * @function redisplay
 *
 * @return   {Number}
 */
define('rl_redisplay', dlsym(libreadline, 'rl_redisplay'), null, 'void');
export function redisplay() {
  return call('rl_redisplay');
}

/**
 * @function redraw_prompt_last_line
 *
 * @return   {Number}
 */
define('rl_redraw_prompt_last_line', dlsym(libreadline, 'rl_redraw_prompt_last_line'), null, 'void');
export function redraw_prompt_last_line() {
  return call('rl_redraw_prompt_last_line');
}

/**
 * @function refresh_line
 *
 * @param    {Number}        ignore1
 * @param    {Number}        ignore2
 *
 * @return   {Number}
 */
define('rl_refresh_line', dlsym(libreadline, 'rl_refresh_line'), null, 'int', 'int', 'int');
export function refresh_line(ignore1, ignore2) {
  return call('rl_refresh_line', ignore1, ignore2);
}

/**
 * @function replace_line
 *
 * @param    {String}        text
 * @param    {Number}        clear_undo
 *
 * @return   {Number}
 */
define('rl_replace_line', dlsym(libreadline, 'rl_replace_line'), null, 'void', 'char *', 'int');
export function replace_line(text, clear_undo) {
  return call('rl_replace_line', text, clear_undo);
}

/**
 * @function reset_after_signal
 *
 * @return   {Number}
 */
define('rl_reset_after_signal', dlsym(libreadline, 'rl_reset_after_signal'), null, 'void');
export function reset_after_signal() {
  return call('rl_reset_after_signal');
}

/**
 * @function reset_line_state
 *
 * @return   {Number}
 */
define('rl_reset_line_state', dlsym(libreadline, 'rl_reset_line_state'), null, 'int');
export function reset_line_state() {
  return call('rl_reset_line_state');
}

/**
 * @function reset_screen_size
 *
 * @return   {Number}
 */
define('rl_reset_screen_size', dlsym(libreadline, 'rl_reset_screen_size'), null, 'void');
export function reset_screen_size() {
  return call('rl_reset_screen_size');
}

/**
 * @function reset_terminal
 *
 * @param    {String}        terminal_name
 *
 * @return   {Number}
 */
define('rl_reset_terminal', dlsym(libreadline, 'rl_reset_terminal'), null, 'int', 'char *');
export function reset_terminal(terminal_name) {
  return call('rl_reset_terminal', terminal_name);
}

/**
 * @function resize_terminal
 *
 * @return   {Number}
 */
define('rl_resize_terminal', dlsym(libreadline, 'rl_resize_terminal'), null, 'void');
export function resize_terminal() {
  return call('rl_resize_terminal');
}

/**
 * @function restart_output
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_restart_output', dlsym(libreadline, 'rl_restart_output'), null, 'int', 'int', 'int');
export function restart_output(count, key) {
  return call('rl_restart_output', count, key);
}

/**
 * @function restore_prompt
 *
 * @return   {Number}
 */
define('rl_restore_prompt', dlsym(libreadline, 'rl_restore_prompt'), null, 'void');
export function restore_prompt() {
  return call('rl_restore_prompt');
}

/**
 * @function reverse_search_history
 *
 * @param    {Number}        sign
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_reverse_search_history', dlsym(libreadline, 'rl_reverse_search_history'), null, 'int', 'int', 'int');
export function reverse_search_history(sign, key) {
  return call('rl_reverse_search_history', sign, key);
}

/**
 * @function revert_line
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_revert_line', dlsym(libreadline, 'rl_revert_line'), null, 'int', 'int', 'int');
export function revert_line(count, key) {
  return call('rl_revert_line', count, key);
}

/**
 * @function rubout
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_rubout', dlsym(libreadline, 'rl_rubout'), null, 'int', 'int', 'int');
export function rubout(count, key) {
  return call('rl_rubout', count, key);
}

/**
 * @function rubout_or_delete
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_rubout_or_delete', dlsym(libreadline, 'rl_rubout_or_delete'), null, 'int', 'int', 'int');
export function rubout_or_delete(count, key) {
  return call('rl_rubout_or_delete', count, key);
}

/**
 * @function save_prompt
 *
 * @return   {Number}
 */
define('rl_save_prompt', dlsym(libreadline, 'rl_save_prompt'), null, 'void');
export function save_prompt() {
  return call('rl_save_prompt');
}

/**
 * @function save_state
 *
 * @param    {Number}        sp
 *
 * @return   {Number}
 */
define('rl_save_state', dlsym(libreadline, 'rl_save_state'), null, 'int', 'void *');
export function save_state(sp) {
  return call('rl_save_state', sp);
}

/**
 * @function set_key
 *
 * @param    {String}        keyseq
 * @param    {Number}        function
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_set_key', dlsym(libreadline, 'rl_set_key'), null, 'int', 'char *', 'void *', 'long');
export function set_key(keyseq, func, map) {
  return call('rl_set_key', keyseq, func, map);
}

/**
 * @function set_keyboard_input_timeout
 *
 * @param    {Number}        u
 *
 * @return   {Number}
 */
define('rl_set_keyboard_input_timeout', dlsym(libreadline, 'rl_set_keyboard_input_timeout'), null, 'int', 'int');
export function set_keyboard_input_timeout(u) {
  return call('rl_set_keyboard_input_timeout', u);
}

/**
 * @function set_keymap
 *
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_set_keymap', dlsym(libreadline, 'rl_set_keymap'), null, 'void', 'long');
export function set_keymap(map) {
  return call('rl_set_keymap', map);
}

/**
 * @function set_keymap_from_edit_mode
 *
 * @return   {Number}
 */
define('rl_set_keymap_from_edit_mode', dlsym(libreadline, 'rl_set_keymap_from_edit_mode'), null, 'void');
export function set_keymap_from_edit_mode() {
  return call('rl_set_keymap_from_edit_mode');
}

/**
 * @function set_keymap_name
 *
 * @param    {String}        name
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_set_keymap_name', dlsym(libreadline, 'rl_set_keymap_name'), null, 'int', 'char *', 'long');
export function set_keymap_name(name, map) {
  return call('rl_set_keymap_name', name, map);
}

/**
 * @function set_mark
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_set_mark', dlsym(libreadline, 'rl_set_mark'), null, 'int', 'int', 'int');
export function set_mark(count, key) {
  return call('rl_set_mark', count, key);
}

/**
 * @function set_paren_blink_timeout
 *
 * @param    {Number}        u
 *
 * @return   {Number}
 */
define('rl_set_paren_blink_timeout', dlsym(libreadline, 'rl_set_paren_blink_timeout'), null, 'int', 'int');
export function set_paren_blink_timeout(u) {
  return call('rl_set_paren_blink_timeout', u);
}

/**
 * @function set_prompt
 *
 * @param    {String}        prompt
 *
 * @return   {Number}
 */
define('rl_set_prompt', dlsym(libreadline, 'rl_set_prompt'), null, 'int', 'char *');
export function set_prompt(prompt) {
  return call('rl_set_prompt', prompt);
}

/**
 * @function set_screen_size
 *
 * @param    {Number}        rows
 * @param    {Number}        cols
 *
 * @return   {Number}
 */
define('rl_set_screen_size', dlsym(libreadline, 'rl_set_screen_size'), null, 'void', 'int', 'int');
export function set_screen_size(rows, cols) {
  return call('rl_set_screen_size', rows, cols);
}

/**
 * @function set_signals
 *
 * @return   {Number}
 */
define('rl_set_signals', dlsym(libreadline, 'rl_set_signals'), null, 'int');
export function set_signals() {
  return call('rl_set_signals');
}

/**
 * @function show_char
 *
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_show_char', dlsym(libreadline, 'rl_show_char'), null, 'int', 'int');
export function show_char(c) {
  return call('rl_show_char', c);
}

/**
 * @function skip_csi_sequence
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_skip_csi_sequence', dlsym(libreadline, 'rl_skip_csi_sequence'), null, 'int', 'int', 'int');
export function skip_csi_sequence(count, key) {
  return call('rl_skip_csi_sequence', count, key);
}

/**
 * @function start_kbd_macro
 *
 * @param    {Number}        ignore1
 * @param    {Number}        ignore2
 *
 * @return   {Number}
 */
define('rl_start_kbd_macro', dlsym(libreadline, 'rl_start_kbd_macro'), null, 'int', 'int', 'int');
export function start_kbd_macro(ignore1, ignore2) {
  return call('rl_start_kbd_macro', ignore1, ignore2);
}

/**
 * @function stop_output
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_stop_output', dlsym(libreadline, 'rl_stop_output'), null, 'int', 'int', 'int');
export function stop_output(count, key) {
  return call('rl_stop_output', count, key);
}

/**
 * @function stuff_char
 *
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_stuff_char', dlsym(libreadline, 'rl_stuff_char'), null, 'int', 'int');
export function stuff_char(key) {
  return call('rl_stuff_char', key);
}

/**
 * @function tab_insert
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_tab_insert', dlsym(libreadline, 'rl_tab_insert'), null, 'int', 'int', 'int');
export function tab_insert(count, key) {
  return call('rl_tab_insert', count, key);
}

/**
 * @function tilde_expand
 *
 * @param    {Number}        ignore
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_tilde_expand', dlsym(libreadline, 'rl_tilde_expand'), null, 'int', 'int', 'int');
export function tilde_expand(ignore, key) {
  return call('rl_tilde_expand', ignore, key);
}

/**
 * @function translate_keyseq
 *
 * @param    {String}        seq
 * @param    {String}        array
 * @param    {Number}        len
 *
 * @return   {Number}
 */
define('rl_translate_keyseq', dlsym(libreadline, 'rl_translate_keyseq'), null, 'int', 'char *', 'char *', 'void *');
export function translate_keyseq(seq, array, len) {
  return call('rl_translate_keyseq', seq, array, len);
}

/**
 * @function transposeChars
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_transpose_chars', dlsym(libreadline, 'rl_transpose_chars'), null, 'int', 'int', 'int');
export function transposeChars(count, key) {
  return call('rl_transpose_chars', count, key);
}

/**
 * @function transposeWords
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_transpose_words', dlsym(libreadline, 'rl_transpose_words'), null, 'int', 'int', 'int');
export function transposeWords(count, key) {
  return call('rl_transpose_words', count, key);
}

/**
 * @function tty_set_default_bindings
 *
 * @param    {Number}        kmap
 *
 * @return   {Number}
 */
define('rl_tty_set_default_bindings', dlsym(libreadline, 'rl_tty_set_default_bindings'), null, 'void', 'long');
export function tty_set_default_bindings(kmap) {
  return call('rl_tty_set_default_bindings', kmap);
}

/**
 * @function tty_set_echoing
 *
 * @param    {Number}        u
 *
 * @return   {Number}
 */
define('rl_tty_set_echoing', dlsym(libreadline, 'rl_tty_set_echoing'), null, 'int', 'int');
export function tty_set_echoing(u) {
  return call('rl_tty_set_echoing', u);
}

/**
 * @function tty_status
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_tty_status', dlsym(libreadline, 'rl_tty_status'), null, 'int', 'int', 'int');
export function tty_status(count, key) {
  return call('rl_tty_status', count, key);
}

/**
 * @function tty_unset_default_bindings
 *
 * @param    {Number}        kmap
 *
 * @return   {Number}
 */
define('rl_tty_unset_default_bindings', dlsym(libreadline, 'rl_tty_unset_default_bindings'), null, 'void', 'long');
export function tty_unset_default_bindings(kmap) {
  return call('rl_tty_unset_default_bindings', kmap);
}

/**
 * @function unbind_command_in_map
 *
 * @param    {String}        command
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_unbind_command_in_map', dlsym(libreadline, 'rl_unbind_command_in_map'), null, 'int', 'char *', 'long');
export function unbind_command_in_map(command, map) {
  return call('rl_unbind_command_in_map', command, map);
}

/**
 * @function unbind_function_in_map
 *
 * @param    {Number}        func
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_unbind_function_in_map', dlsym(libreadline, 'rl_unbind_function_in_map'), null, 'int', 'void *', 'long');
export function unbind_function_in_map(func, map) {
  return call('rl_unbind_function_in_map', func, map);
}

/**
 * @function unbind_key
 *
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_unbind_key', dlsym(libreadline, 'rl_unbind_key'), null, 'int', 'int');
export function unbind_key(key) {
  return call('rl_unbind_key', key);
}

/**
 * @function unbind_key_in_map
 *
 * @param    {Number}        key
 * @param    {Number}        map
 *
 * @return   {Number}
 */
define('rl_unbind_key_in_map', dlsym(libreadline, 'rl_unbind_key_in_map'), null, 'int', 'int', 'long');
export function unbind_key_in_map(key, map) {
  return call('rl_unbind_key_in_map', key, map);
}

/**
 * @function undo_command
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_undo_command', dlsym(libreadline, 'rl_undo_command'), null, 'int', 'int', 'int');
export function undo_command(count, key) {
  return call('rl_undo_command', count, key);
}

/**
 * @function universal_argument
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_universal_argument', dlsym(libreadline, 'rl_universal_argument'), null, 'int', 'int', 'int');
export function universal_argument(count, key) {
  return call('rl_universal_argument', count, key);
}

/**
 * @function unix_filename_rubout
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_unix_filename_rubout', dlsym(libreadline, 'rl_unix_filename_rubout'), null, 'int', 'int', 'int');
export function unix_filename_rubout(count, key) {
  return call('rl_unix_filename_rubout', count, key);
}

/**
 * @function unix_line_discard
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_unix_line_discard', dlsym(libreadline, 'rl_unix_line_discard'), null, 'int', 'int', 'int');
export function unix_line_discard(count, key) {
  return call('rl_unix_line_discard', count, key);
}

/**
 * @function unix_word_rubout
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_unix_word_rubout', dlsym(libreadline, 'rl_unix_word_rubout'), null, 'int', 'int', 'int');
export function unix_word_rubout(count, key) {
  return call('rl_unix_word_rubout', count, key);
}

/**
 * @function untranslate_keyseq
 *
 * @param    {Number}        seq
 *
 * @return   {String}
 */
define('rl_untranslate_keyseq', dlsym(libreadline, 'rl_untranslate_keyseq'), null, 'char *', 'int');
export function untranslate_keyseq(seq) {
  return call('rl_untranslate_keyseq', seq);
}

/**
 * @function upcaseWord
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_upcase_word', dlsym(libreadline, 'rl_upcase_word'), null, 'int', 'int', 'int');
export function upcaseWord(count, key) {
  return call('rl_upcase_word', count, key);
}

/**
 * @function username_completion_function
 *
 * @param    {String}        text
 * @param    {Number}        state
 *
 * @return   {String}
 */
define('rl_username_completion_function', dlsym(libreadline, 'rl_username_completion_function'), null, 'char *', 'char *', 'int');
export function username_completion_function(text, state) {
  return call('rl_username_completion_function', text, state);
}

/**
 * @function variable_bind
 *
 * @param    {String}        name
 * @param    {String}        value
 *
 * @return   {Number}
 */
define('rl_variable_bind', dlsym(libreadline, 'rl_variable_bind'), null, 'int', 'char *', 'char *');
export function variable_bind(name, value) {
  return call('rl_variable_bind', name, value);
}

/**
 * @function variable_dumper
 *
 * @param    {Number}        print_readably
 *
 * @return   {Number}
 */
define('rl_variable_dumper', dlsym(libreadline, 'rl_variable_dumper'), null, 'void', 'int');
export function variable_dumper(print_readably) {
  return call('rl_variable_dumper', print_readably);
}

/**
 * @function variable_value
 *
 * @param    {String}        name
 *
 * @return   {String}
 */
define('rl_variable_value', dlsym(libreadline, 'rl_variable_value'), null, 'char *', 'char *');
export function variable_value(name) {
  return call('rl_variable_value', name);
}

/**
 * @function vi_append_eol
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_append_eol', dlsym(libreadline, 'rl_vi_append_eol'), null, 'int', 'int', 'int');
export function vi_append_eol(count, key) {
  return call('rl_vi_append_eol', count, key);
}

/**
 * @function vi_append_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_append_mode', dlsym(libreadline, 'rl_vi_append_mode'), null, 'int', 'int', 'int');
export function vi_append_mode(count, key) {
  return call('rl_vi_append_mode', count, key);
}

/**
 * @function vi_arg_digit
 *
 * @param    {Number}        count
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_vi_arg_digit', dlsym(libreadline, 'rl_vi_arg_digit'), null, 'int', 'int', 'int');
export function vi_arg_digit(count, c) {
  return call('rl_vi_arg_digit', count, c);
}

/**
 * @function vi_bWord
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_bWord', dlsym(libreadline, 'rl_vi_bWord'), null, 'int', 'int', 'int');
export function vi_bWord(count, ignore) {
  return call('rl_vi_bWord', count, ignore);
}

/**
 * @function vi_back_to_indent
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_back_to_indent', dlsym(libreadline, 'rl_vi_back_to_indent'), null, 'int', 'int', 'int');
export function vi_back_to_indent(count, key) {
  return call('rl_vi_back_to_indent', count, key);
}

/**
 * @function vi_bracktype
 *
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_vi_bracktype', dlsym(libreadline, 'rl_vi_bracktype'), null, 'int', 'int');
export function vi_bracktype(c) {
  return call('rl_vi_bracktype', c);
}

/**
 * @function vi_bword
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_bword', dlsym(libreadline, 'rl_vi_bword'), null, 'int', 'int', 'int');
export function vi_bword(count, ignore) {
  return call('rl_vi_bword', count, ignore);
}

/**
 * @function vi_change_case
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_change_case', dlsym(libreadline, 'rl_vi_change_case'), null, 'int', 'int', 'int');
export function vi_change_case(count, ignore) {
  return call('rl_vi_change_case', count, ignore);
}

/**
 * @function vi_change_char
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_change_char', dlsym(libreadline, 'rl_vi_change_char'), null, 'int', 'int', 'int');
export function vi_change_char(count, key) {
  return call('rl_vi_change_char', count, key);
}

/**
 * @function vi_change_to
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_change_to', dlsym(libreadline, 'rl_vi_change_to'), null, 'int', 'int', 'int');
export function vi_change_to(count, key) {
  return call('rl_vi_change_to', count, key);
}

/**
 * @function vi_char_search
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_char_search', dlsym(libreadline, 'rl_vi_char_search'), null, 'int', 'int', 'int');
export function vi_char_search(count, key) {
  return call('rl_vi_char_search', count, key);
}

/**
 * @function vi_check
 *
 * @return   {Number}
 */
define('rl_vi_check', dlsym(libreadline, 'rl_vi_check'), null, 'int');
export function vi_check() {
  return call('rl_vi_check');
}

/**
 * @function vi_column
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_column', dlsym(libreadline, 'rl_vi_column'), null, 'int', 'int', 'int');
export function vi_column(count, key) {
  return call('rl_vi_column', count, key);
}

/**
 * @function vi_complete
 *
 * @param    {Number}        ignore
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_complete', dlsym(libreadline, 'rl_vi_complete'), null, 'int', 'int', 'int');
export function vi_complete(ignore, key) {
  return call('rl_vi_complete', ignore, key);
}

/**
 * @function vi_delete
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_delete', dlsym(libreadline, 'rl_vi_delete'), null, 'int', 'int', 'int');
export function vi_delete(count, key) {
  return call('rl_vi_delete', count, key);
}

/**
 * @function vi_delete_to
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_delete_to', dlsym(libreadline, 'rl_vi_delete_to'), null, 'int', 'int', 'int');
export function vi_delete_to(count, key) {
  return call('rl_vi_delete_to', count, key);
}

/**
 * @function vi_domove
 *
 * @param    {Number}        x
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_domove', dlsym(libreadline, 'rl_vi_domove'), null, 'int', 'int', 'void *');
export function vi_domove(x, ignore) {
  return call('rl_vi_domove', x, ignore);
}

/**
 * @function vi_eWord
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_eWord', dlsym(libreadline, 'rl_vi_eWord'), null, 'int', 'int', 'int');
export function vi_eWord(count, ignore) {
  return call('rl_vi_eWord', count, ignore);
}

/**
 * @function vi_editing_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_editing_mode', dlsym(libreadline, 'rl_vi_editing_mode'), null, 'int', 'int', 'int');
export function vi_editing_mode(count, key) {
  return call('rl_vi_editing_mode', count, key);
}

/**
 * @function vi_end_word
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_end_word', dlsym(libreadline, 'rl_vi_end_word'), null, 'int', 'int', 'int');
export function vi_end_word(count, key) {
  return call('rl_vi_end_word', count, key);
}

/**
 * @function vi_eof_maybe
 *
 * @param    {Number}        count
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_vi_eof_maybe', dlsym(libreadline, 'rl_vi_eof_maybe'), null, 'int', 'int', 'int');
export function vi_eof_maybe(count, c) {
  return call('rl_vi_eof_maybe', count, c);
}

/**
 * @function vi_eword
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_eword', dlsym(libreadline, 'rl_vi_eword'), null, 'int', 'int', 'int');
export function vi_eword(count, ignore) {
  return call('rl_vi_eword', count, ignore);
}

/**
 * @function vi_fWord
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_fWord', dlsym(libreadline, 'rl_vi_fWord'), null, 'int', 'int', 'int');
export function vi_fWord(count, ignore) {
  return call('rl_vi_fWord', count, ignore);
}

/**
 * @function vi_fetch_history
 *
 * @param    {Number}        count
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_vi_fetch_history', dlsym(libreadline, 'rl_vi_fetch_history'), null, 'int', 'int', 'int');
export function vi_fetch_history(count, c) {
  return call('rl_vi_fetch_history', count, c);
}

/**
 * @function vi_first_print
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_first_print', dlsym(libreadline, 'rl_vi_first_print'), null, 'int', 'int', 'int');
export function vi_first_print(count, key) {
  return call('rl_vi_first_print', count, key);
}

/**
 * @function vi_fword
 *
 * @param    {Number}        count
 * @param    {Number}        ignore
 *
 * @return   {Number}
 */
define('rl_vi_fword', dlsym(libreadline, 'rl_vi_fword'), null, 'int', 'int', 'int');
export function vi_fword(count, ignore) {
  return call('rl_vi_fword', count, ignore);
}

/**
 * @function vi_goto_mark
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_goto_mark', dlsym(libreadline, 'rl_vi_goto_mark'), null, 'int', 'int', 'int');
export function vi_goto_mark(count, key) {
  return call('rl_vi_goto_mark', count, key);
}

/**
 * @function vi_insert_beg
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_insert_beg', dlsym(libreadline, 'rl_vi_insert_beg'), null, 'int', 'int', 'int');
export function vi_insert_beg(count, key) {
  return call('rl_vi_insert_beg', count, key);
}

/**
 * @function vi_insert_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_insert_mode', dlsym(libreadline, 'rl_vi_insert_mode'), null, 'int', 'int', 'int');
export function vi_insert_mode(count, key) {
  return call('rl_vi_insert_mode', count, key);
}

/**
 * @function vi_insertion_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_insertion_mode', dlsym(libreadline, 'rl_vi_insertion_mode'), null, 'int', 'int', 'int');
export function vi_insertion_mode(count, key) {
  return call('rl_vi_insertion_mode', count, key);
}

/**
 * @function vi_match
 *
 * @param    {Number}        ignore
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_match', dlsym(libreadline, 'rl_vi_match'), null, 'int', 'int', 'int');
export function vi_match(ignore, key) {
  return call('rl_vi_match', ignore, key);
}

/**
 * @function vi_movement_mode
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_movement_mode', dlsym(libreadline, 'rl_vi_movement_mode'), null, 'int', 'int', 'int');
export function vi_movement_mode(count, key) {
  return call('rl_vi_movement_mode', count, key);
}

/**
 * @function vi_next_word
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_next_word', dlsym(libreadline, 'rl_vi_next_word'), null, 'int', 'int', 'int');
export function vi_next_word(count, key) {
  return call('rl_vi_next_word', count, key);
}

/**
 * @function vi_overstrike
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_overstrike', dlsym(libreadline, 'rl_vi_overstrike'), null, 'int', 'int', 'int');
export function vi_overstrike(count, key) {
  return call('rl_vi_overstrike', count, key);
}

/**
 * @function vi_overstrike_delete
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_overstrike_delete', dlsym(libreadline, 'rl_vi_overstrike_delete'), null, 'int', 'int', 'int');
export function vi_overstrike_delete(count, key) {
  return call('rl_vi_overstrike_delete', count, key);
}

/**
 * @function vi_prev_word
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_prev_word', dlsym(libreadline, 'rl_vi_prev_word'), null, 'int', 'int', 'int');
export function vi_prev_word(count, key) {
  return call('rl_vi_prev_word', count, key);
}

/**
 * @function vi_put
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_put', dlsym(libreadline, 'rl_vi_put'), null, 'int', 'int', 'int');
export function vi_put(count, key) {
  return call('rl_vi_put', count, key);
}

/**
 * @function vi_redo
 *
 * @param    {Number}        count
 * @param    {Number}        c
 *
 * @return   {Number}
 */
define('rl_vi_redo', dlsym(libreadline, 'rl_vi_redo'), null, 'int', 'int', 'int');
export function vi_redo(count, c) {
  return call('rl_vi_redo', count, c);
}

/**
 * @function vi_rubout
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_rubout', dlsym(libreadline, 'rl_vi_rubout'), null, 'int', 'int', 'int');
export function vi_rubout(count, key) {
  return call('rl_vi_rubout', count, key);
}

/**
 * @function vi_search
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_search', dlsym(libreadline, 'rl_vi_search'), null, 'int', 'int', 'int');
export function vi_search(count, key) {
  return call('rl_vi_search', count, key);
}

/**
 * @function vi_search_again
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_search_again', dlsym(libreadline, 'rl_vi_search_again'), null, 'int', 'int', 'int');
export function vi_search_again(count, key) {
  return call('rl_vi_search_again', count, key);
}

/**
 * @function vi_set_mark
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_set_mark', dlsym(libreadline, 'rl_vi_set_mark'), null, 'int', 'int', 'int');
export function vi_set_mark(count, key) {
  return call('rl_vi_set_mark', count, key);
}

/**
 * @function vi_start_inserting
 *
 * @param    {Number}        key
 * @param    {Number}        repeat
 * @param    {Number}        sign
 *
 * @return   {Number}
 */
define('rl_vi_start_inserting', dlsym(libreadline, 'rl_vi_start_inserting'), null, 'void', 'int', 'int', 'int');
export function vi_start_inserting(key, repeat, sign) {
  return call('rl_vi_start_inserting', key, repeat, sign);
}

/**
 * @function vi_subst
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_subst', dlsym(libreadline, 'rl_vi_subst'), null, 'int', 'int', 'int');
export function vi_subst(count, key) {
  return call('rl_vi_subst', count, key);
}

/**
 * @function vi_tilde_expand
 *
 * @param    {Number}        ignore
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_tilde_expand', dlsym(libreadline, 'rl_vi_tilde_expand'), null, 'int', 'int', 'int');
export function vi_tilde_expand(ignore, key) {
  return call('rl_vi_tilde_expand', ignore, key);
}

/**
 * @function vi_undo
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_undo', dlsym(libreadline, 'rl_vi_undo'), null, 'int', 'int', 'int');
export function vi_undo(count, key) {
  return call('rl_vi_undo', count, key);
}

/**
 * @function vi_unix_word_rubout
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_unix_word_rubout', dlsym(libreadline, 'rl_vi_unix_word_rubout'), null, 'int', 'int', 'int');
export function vi_unix_word_rubout(count, key) {
  return call('rl_vi_unix_word_rubout', count, key);
}

/**
 * @function vi_yank_arg
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_yank_arg', dlsym(libreadline, 'rl_vi_yank_arg'), null, 'int', 'int', 'int');
export function vi_yank_arg(count, key) {
  return call('rl_vi_yank_arg', count, key);
}

/**
 * @function vi_yank_pop
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_yank_pop', dlsym(libreadline, 'rl_vi_yank_pop'), null, 'int', 'int', 'int');
export function vi_yank_pop(count, key) {
  return call('rl_vi_yank_pop', count, key);
}

/**
 * @function vi_yank_to
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_vi_yank_to', dlsym(libreadline, 'rl_vi_yank_to'), null, 'int', 'int', 'int');
export function vi_yank_to(count, key) {
  return call('rl_vi_yank_to', count, key);
}

/**
 * @function yank
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_yank', dlsym(libreadline, 'rl_yank'), null, 'int', 'int', 'int');
export function yank(count, key) {
  return call('rl_yank', count, key);
}

/**
 * @function yank_last_arg
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_yank_last_arg', dlsym(libreadline, 'rl_yank_last_arg'), null, 'int', 'int', 'int');
export function yank_last_arg(count, key) {
  return call('rl_yank_last_arg', count, key);
}

/**
 * @function yank_nth_arg
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_yank_nth_arg', dlsym(libreadline, 'rl_yank_nth_arg'), null, 'int', 'int', 'int');
export function yank_nth_arg(count, key) {
  return call('rl_yank_nth_arg', count, key);
}

/**
 * @function yank_pop
 *
 * @param    {Number}        count
 * @param    {Number}        key
 *
 * @return   {Number}
 */
define('rl_yank_pop', dlsym(libreadline, 'rl_yank_pop'), null, 'int', 'int', 'int');
export function yank_pop(count, key) {
  return call('rl_yank_pop', count, key);
}