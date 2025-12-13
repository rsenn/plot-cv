import { call, define, dlopen, dlsym, RTLD_NOW } from 'ffi';

const libeditline = dlopen('/usr/local/lib/libeditline.so.1', RTLD_NOW);

export const el_hist_size = dlsym('el_hist_size', libeditline);

export const el_no_echo = dlsym('el_no_echo', libeditline);

export const el_no_hist = dlsym('el_no_hist', libeditline);

export const prompt_len = dlsym('prompt_len', libeditline);

export const rl_attempted_completion_function = dlsym('rl_attempted_completion_function', libeditline);

export const rl_attempted_completion_over = dlsym('rl_attempted_completion_over', libeditline);
export const rl_completion_entry_function = dlsym('rl_completion_entry_function', libeditline);
export const rl_deprep_term_function = dlsym('rl_deprep_term_function', libeditline);
export const rl_end = dlsym('rl_end', libeditline);
export const rl_eof = dlsym('rl_eof', libeditline);
export const rl_erase = dlsym('rl_erase', libeditline);
export const rl_event_hook = dlsym('rl_event_hook', libeditline);
export const rl_getc_function = dlsym('rl_getc_function', libeditline);
export const rl_inhibit_complete = dlsym('rl_inhibit_complete', libeditline);
export const rl_instream = dlsym('rl_instream', libeditline);
export const rl_intr = dlsym('rl_intr', libeditline);
export const rl_kill = dlsym('rl_kill', libeditline);
export const rl_line_buffer = dlsym('rl_line_buffer', libeditline);
export const rl_mark = dlsym('rl_mark', libeditline);
export const rl_meta_chars = dlsym('rl_meta_chars', libeditline);
export const rl_outstream = dlsym('rl_outstream', libeditline);
export const rl_point = dlsym('rl_point', libeditline);
export const rl_prep_term_function = dlsym('rl_prep_term_function', libeditline);
export const rl_prompt = dlsym('rl_prompt', libeditline);
export const rl_quit = dlsym('rl_quit', libeditline);
export const rl_readline_name = dlsym('rl_readline_name', libeditline);

/* char* el_find_word()  */
define('el_find_word', dlsym('el_find_word', libeditline), null, 'char *');
function el_find_word() {
  return call('el_find_word');
}

/* void el_print_columns(int ac, char** av)  */
define('el_print_columns', dlsym('el_print_columns', libeditline), null, 'void', 'int', 'void *');
function el_print_columns(ac, av) {
  return call('el_print_columns', ac, av);
}

/* enum el_status_t el_ring_bell()  */
define('el_ring_bell', dlsym('el_ring_bell', libeditline), null, 'int');
function el_ring_bell() {
  return call('el_ring_bell');
}

/* enum el_status_t el_del_char()  */
define('el_del_char', dlsym('el_del_char', libeditline), null, 'int');
function el_del_char() {
  return call('el_del_char');
}

/* enum el_status_t el_bind_key(int key, el_keymap_func_t* func)  */
define('el_bind_key', dlsym('el_bind_key', libeditline), null, 'int', 'int', 'void *');
function el_bind_key(key, func) {
  return call('el_bind_key', key, func);
}

/* enum el_status_t el_bind_key_in_metamap(int key, el_keymap_func_t* func)  */
define('el_bind_key_in_metamap', dlsym('el_bind_key_in_metamap', libeditline), null, 'int', 'int', 'void *');
function el_bind_key_in_metamap(key, func) {
  return call('el_bind_key_in_metamap', key, func);
}

/* const char* el_next_hist()  */
define('el_next_hist', dlsym('el_next_hist', libeditline), null, 'void *');
function el_next_hist() {
  return call('el_next_hist');
}

/* const char* el_prev_hist()  */
define('el_prev_hist', dlsym('el_prev_hist', libeditline), null, 'void *');
function el_prev_hist() {
  return call('el_prev_hist');
}

/* char* rl_complete(char* token, int* match)  */
define('rl_complete', dlsym('rl_complete', libeditline), null, 'char *', 'char *', 'void *');
function rl_complete(token, match) {
  return call('rl_complete', token, match);
}

/* int rl_list_possib(char* token, char*** av)  */
define('rl_list_possib', dlsym('rl_list_possib', libeditline), null, 'int', 'char *', 'void *');
function rl_list_possib(token, av) {
  return call('rl_list_possib', token, av);
}

/* char** rl_completion_matches(const char* token, rl_compentry_func_t* generator)  */
define('rl_completion_matches', dlsym('rl_completion_matches', libeditline), null, 'void *', 'void *', 'void *');
function rl_completion_matches(token, generator) {
  return call('rl_completion_matches', token, generator);
}

/* char* rl_filename_completion_function(const char* text, int state)  */
define('rl_filename_completion_function', dlsym('rl_filename_completion_function', libeditline), null, 'char *', 'void *', 'int');
function rl_filename_completion_function(text, state) {
  return call('rl_filename_completion_function', text, state);
}

/* void rl_initialize()  */
define('rl_initialize', dlsym('rl_initialize', libeditline), null, 'void');
function rl_initialize() {
  return call('rl_initialize');
}

/* void rl_reset_terminal(const char* terminal_name)  */
define('rl_reset_terminal', dlsym('rl_reset_terminal', libeditline), null, 'void', 'void *');
function rl_reset_terminal(terminal_name) {
  return call('rl_reset_terminal', terminal_name);
}

/* void rl_uninitialize()  */
define('rl_uninitialize', dlsym('rl_uninitialize', libeditline), null, 'void');
function rl_uninitialize() {
  return call('rl_uninitialize');
}

/* void rl_save_prompt()  */
define('rl_save_prompt', dlsym('rl_save_prompt', libeditline), null, 'void');
function rl_save_prompt() {
  return call('rl_save_prompt');
}

/* void rl_restore_prompt()  */
define('rl_restore_prompt', dlsym('rl_restore_prompt', libeditline), null, 'void');
function rl_restore_prompt() {
  return call('rl_restore_prompt');
}

/* void rl_set_prompt(const char* prompt)  */
define('rl_set_prompt', dlsym('rl_set_prompt', libeditline), null, 'void', 'void *');
function rl_set_prompt(prompt) {
  return call('rl_set_prompt', prompt);
}

/* void rl_clear_message()  */
define('rl_clear_message', dlsym('rl_clear_message', libeditline), null, 'void');
function rl_clear_message() {
  return call('rl_clear_message');
}

/* void rl_forced_update_display()  */
define('rl_forced_update_display', dlsym('rl_forced_update_display', libeditline), null, 'void');
function rl_forced_update_display() {
  return call('rl_forced_update_display');
}

/* void rl_prep_terminal(int meta_flag)  */
define('rl_prep_terminal', dlsym('rl_prep_terminal', libeditline), null, 'void', 'int');
function rl_prep_terminal(meta_flag) {
  return call('rl_prep_terminal', meta_flag);
}

/* void rl_deprep_terminal()  */
define('rl_deprep_terminal', dlsym('rl_deprep_terminal', libeditline), null, 'void');
function rl_deprep_terminal() {
  return call('rl_deprep_terminal');
}

/* int rl_getc()  */
define('rl_getc', dlsym('rl_getc', libeditline), null, 'int');
function rl_getc() {
  return call('rl_getc');
}

/* int rl_insert_text(const char* text)  */
define('rl_insert_text', dlsym('rl_insert_text', libeditline), null, 'int', 'void *');
function rl_insert_text(text) {
  return call('rl_insert_text', text);
}

/* int rl_refresh_line(int ignore1, int ignore2)  */
define('rl_refresh_line', dlsym('rl_refresh_line', libeditline), null, 'int', 'int', 'int');
function rl_refresh_line(ignore1, ignore2) {
  return call('rl_refresh_line', ignore1, ignore2);
}

/* rl_getc_func_t* rl_set_getc_func(rl_getc_func_t* func)  */
define('rl_set_getc_func', dlsym('rl_set_getc_func', libeditline), null, 'void *', 'void *');
function rl_set_getc_func(func) {
  return call('rl_set_getc_func', func);
}

/* rl_complete_func_t* rl_set_complete_func(rl_complete_func_t* func)  */
define('rl_set_complete_func', dlsym('rl_set_complete_func', libeditline), null, 'void *', 'void *');
function rl_set_complete_func(func) {
  return call('rl_set_complete_func', func);
}

/* rl_list_possib_func_t* rl_set_list_possib_func(rl_list_possib_func_t* func)  */
define('rl_set_list_possib_func', dlsym('rl_set_list_possib_func', libeditline), null, 'void *', 'void *');
function rl_set_list_possib_func(func) {
  return call('rl_set_list_possib_func', func);
}

/* void rl_callback_handler_install(const char* prompt, rl_vcpfunc_t* lhandler)  */
define('rl_callback_handler_install', dlsym('rl_callback_handler_install', libeditline), null, 'void', 'void *', 'void *');
function rl_callback_handler_install(prompt, lhandler) {
  return call('rl_callback_handler_install', prompt, lhandler);
}

/* void rl_callback_read_char()  */
define('rl_callback_read_char', dlsym('rl_callback_read_char', libeditline), null, 'void');
function rl_callback_read_char() {
  return call('rl_callback_read_char');
}

/* void rl_callback_handler_remove()  */
define('rl_callback_handler_remove', dlsym('rl_callback_handler_remove', libeditline), null, 'void');
function rl_callback_handler_remove() {
  return call('rl_callback_handler_remove');
}