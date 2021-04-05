/* char* el_find_word()  */
define('el_find_word', dlsym('el_find_word', editline_handle), null, 'char *');
function el_find_word() {
  return call('el_find_word', );
}

/* void el_print_columns(int ac, char** av)  */
define('el_print_columns', dlsym('el_print_columns', editline_handle), null, 'void', 'int', 'void *');
function el_print_columns(ac, av) {
  return call('el_print_columns', ac, av);
}

/* enum el_status_t el_ring_bell()  */
define('el_ring_bell', dlsym('el_ring_bell', editline_handle), null, 'int');
function el_ring_bell() {
  return call('el_ring_bell', );
}

/* enum el_status_t el_del_char()  */
define('el_del_char', dlsym('el_del_char', editline_handle), null, 'int');
function el_del_char() {
  return call('el_del_char', );
}

/* enum el_status_t el_bind_key(int key, el_keymap_func_t* function)  */
define('el_bind_key', dlsym('el_bind_key', editline_handle), null, 'int', 'int', 'void *');
function el_bind_key(key, function) {
  return call('el_bind_key', key, function);
}

/* enum el_status_t el_bind_key_in_metamap(int key, el_keymap_func_t* function)  */
define('el_bind_key_in_metamap', dlsym('el_bind_key_in_metamap', editline_handle), null, 'int', 'int', 'void *');
function el_bind_key_in_metamap(key, function) {
  return call('el_bind_key_in_metamap', key, function);
}

/* const char* el_next_hist()  */
define('el_next_hist', dlsym('el_next_hist', editline_handle), null, 'void *');
function el_next_hist() {
  return call('el_next_hist', );
}

/* const char* el_prev_hist()  */
define('el_prev_hist', dlsym('el_prev_hist', editline_handle), null, 'void *');
function el_prev_hist() {
  return call('el_prev_hist', );
}

/* char* rl_complete(char* token, int* match)  */
define('rl_complete', dlsym('rl_complete', editline_handle), null, 'char *', 'char *', 'void *');
function rl_complete(token, match) {
  return call('rl_complete', token, match);
}

/* int rl_list_possib(char* token, char*** av)  */
define('rl_list_possib', dlsym('rl_list_possib', editline_handle), null, 'int', 'char *', 'void *');
function rl_list_possib(token, av) {
  return call('rl_list_possib', token, av);
}

/* char** rl_completion_matches(const char* token, rl_compentry_func_t* generator)  */
define('rl_completion_matches', dlsym('rl_completion_matches', editline_handle), null, 'void *', 'void *', 'void *');
function rl_completion_matches(token, generator) {
  return call('rl_completion_matches', token, generator);
}

/* char* rl_filename_completion_function(const char* text, int state)  */
define('rl_filename_completion_function', dlsym('rl_filename_completion_function', editline_handle), null, 'char *', 'void *', 'int');
function rl_filename_completion_function(text, state) {
  return call('rl_filename_completion_function', text, state);
}

/* void rl_initialize()  */
define('rl_initialize', dlsym('rl_initialize', editline_handle), null, 'void');
function rl_initialize() {
  return call('rl_initialize', );
}

/* void rl_reset_terminal(const char* terminal_name)  */
define('rl_reset_terminal', dlsym('rl_reset_terminal', editline_handle), null, 'void', 'void *');
function rl_reset_terminal(terminal_name) {
  return call('rl_reset_terminal', terminal_name);
}

/* void rl_uninitialize()  */
define('rl_uninitialize', dlsym('rl_uninitialize', editline_handle), null, 'void');
function rl_uninitialize() {
  return call('rl_uninitialize', );
}

/* void rl_save_prompt()  */
define('rl_save_prompt', dlsym('rl_save_prompt', editline_handle), null, 'void');
function rl_save_prompt() {
  return call('rl_save_prompt', );
}

/* void rl_restore_prompt()  */
define('rl_restore_prompt', dlsym('rl_restore_prompt', editline_handle), null, 'void');
function rl_restore_prompt() {
  return call('rl_restore_prompt', );
}

/* void rl_set_prompt(const char* prompt)  */
define('rl_set_prompt', dlsym('rl_set_prompt', editline_handle), null, 'void', 'void *');
function rl_set_prompt(prompt) {
  return call('rl_set_prompt', prompt);
}

/* void rl_clear_message()  */
define('rl_clear_message', dlsym('rl_clear_message', editline_handle), null, 'void');
function rl_clear_message() {
  return call('rl_clear_message', );
}

/* void rl_forced_update_display()  */
define('rl_forced_update_display', dlsym('rl_forced_update_display', editline_handle), null, 'void');
function rl_forced_update_display() {
  return call('rl_forced_update_display', );
}

/* void rl_prep_terminal(int meta_flag)  */
define('rl_prep_terminal', dlsym('rl_prep_terminal', editline_handle), null, 'void', 'int');
function rl_prep_terminal(meta_flag) {
  return call('rl_prep_terminal', meta_flag);
}

/* void rl_deprep_terminal()  */
define('rl_deprep_terminal', dlsym('rl_deprep_terminal', editline_handle), null, 'void');
function rl_deprep_terminal() {
  return call('rl_deprep_terminal', );
}

/* int rl_getc()  */
define('rl_getc', dlsym('rl_getc', editline_handle), null, 'int');
function rl_getc() {
  return call('rl_getc', );
}

/* int rl_insert_text(const char* text)  */
define('rl_insert_text', dlsym('rl_insert_text', editline_handle), null, 'int', 'void *');
function rl_insert_text(text) {
  return call('rl_insert_text', text);
}

/* int rl_refresh_line(int ignore1, int ignore2)  */
define('rl_refresh_line', dlsym('rl_refresh_line', editline_handle), null, 'int', 'int', 'int');
function rl_refresh_line(ignore1, ignore2) {
  return call('rl_refresh_line', ignore1, ignore2);
}

/* rl_getc_func_t* rl_set_getc_func(rl_getc_func_t* func)  */
define('rl_set_getc_func', dlsym('rl_set_getc_func', editline_handle), null, 'void *', 'void *');
function rl_set_getc_func(func) {
  return call('rl_set_getc_func', func);
}

/* rl_complete_func_t* rl_set_complete_func(rl_complete_func_t* func)  */
define('rl_set_complete_func', dlsym('rl_set_complete_func', editline_handle), null, 'void *', 'void *');
function rl_set_complete_func(func) {
  return call('rl_set_complete_func', func);
}

/* rl_list_possib_func_t* rl_set_list_possib_func(rl_list_possib_func_t* func)  */
define('rl_set_list_possib_func', dlsym('rl_set_list_possib_func', editline_handle), null, 'void *', 'void *');
function rl_set_list_possib_func(func) {
  return call('rl_set_list_possib_func', func);
}

/* void rl_callback_handler_install(const char* prompt, rl_vcpfunc_t* lhandler)  */
define('rl_callback_handler_install', dlsym('rl_callback_handler_install', editline_handle), null, 'void', 'void *', 'void *');
function rl_callback_handler_install(prompt, lhandler) {
  return call('rl_callback_handler_install', prompt, lhandler);
}

/* void rl_callback_read_char()  */
define('rl_callback_read_char', dlsym('rl_callback_read_char', editline_handle), null, 'void');
function rl_callback_read_char() {
  return call('rl_callback_read_char', );
}

/* void rl_callback_handler_remove()  */
define('rl_callback_handler_remove', dlsym('rl_callback_handler_remove', editline_handle), null, 'void');
function rl_callback_handler_remove() {
  return call('rl_callback_handler_remove', );
}
