import { dlopen, define, dlerror, dlclose, dlsym, call, errno, RTLD_NOW } from 'ffi';
const libportmidi = dlopen('libportmidi.so.0', RTLD_NOW);
/**
 * @function Pm_Initialize
 *
 * @return   {Number}
 */
define('Pm_Initialize', dlsym(libportmidi, 'Pm_Initialize'), null, 'int');
function Pm_Initialize() {
  return call('Pm_Initialize');
}
/**
 * @function Pm_Terminate
 *
 * @return   {Number}
 */
define('Pm_Terminate', dlsym(libportmidi, 'Pm_Terminate'), null, 'int');
function Pm_Terminate() {
  return call('Pm_Terminate');
}
/**
 * @function Pm_HasHostError
 *
 * @param    {ArrayBuffer}   stream
 *
 * @return   {Number}
 */
define('Pm_HasHostError', dlsym(libportmidi, 'Pm_HasHostError'), null, 'int', 'void *');
function Pm_HasHostError(stream) {
  return call('Pm_HasHostError', stream);
}
/**
 * @function Pm_GetErrorText
 *
 * @param    {Number}        errnum
 *
 * @return   {String}
 */
define('Pm_GetErrorText', dlsym(libportmidi, 'Pm_GetErrorText'), null, 'char *', 'int');
function Pm_GetErrorText(errnum) {
  return call('Pm_GetErrorText', errnum);
}
/**
 * @function Pm_GetHostErrorText
 *
 * @param    {String}        msg
 * @param    {Number}        len
 */
define('Pm_GetHostErrorText', dlsym(libportmidi, 'Pm_GetHostErrorText'), null, 'void', 'char *', 'unsigned int');
function Pm_GetHostErrorText(msg, len) {
  call('Pm_GetHostErrorText', msg, len);
}
/**
 * @function Pm_CountDevices
 *
 * @return   {Number}
 */
define('Pm_CountDevices', dlsym(libportmidi, 'Pm_CountDevices'), null, 'int');
function Pm_CountDevices() {
  return call('Pm_CountDevices');
}
/**
 * @function Pm_GetDefaultInputDeviceID
 *
 * @return   {Number}
 */
define('Pm_GetDefaultInputDeviceID', dlsym(libportmidi, 'Pm_GetDefaultInputDeviceID'), null, 'int');
function Pm_GetDefaultInputDeviceID() {
  return call('Pm_GetDefaultInputDeviceID');
}
/**
 * @function Pm_GetDefaultOutputDeviceID
 *
 * @return   {Number}
 */
define('Pm_GetDefaultOutputDeviceID', dlsym(libportmidi, 'Pm_GetDefaultOutputDeviceID'), null, 'int');
function Pm_GetDefaultOutputDeviceID() {
  return call('Pm_GetDefaultOutputDeviceID');
}
/**
 * @function Pm_GetDeviceInfo
 *
 * @param    {Number}        id
 *
 * @return   {Number}
 */
define('Pm_GetDeviceInfo', dlsym(libportmidi, 'Pm_GetDeviceInfo'), null, 'void *', 'int');
function Pm_GetDeviceInfo(id) {
  return call('Pm_GetDeviceInfo', id);
}
/**
 * @function Pm_OpenInput
 *
 * @param    {ArrayBuffer}   stream
 * @param    {Number}        inputDevice
 * @param    {Object|null}   inputDriverInfo
 * @param    {Number}        bufferSize
 * @param    {Function|null} time_proc
 * @param    {Number}        time_info
 *
 * @return   {Number}
 */
define(
  'Pm_OpenInput',
  dlsym(libportmidi, 'Pm_OpenInput'),
  null,
  'int',
  'void *',
  'int',
  'void *',
  'int',
  'void *',
  'void *',
);
function Pm_OpenInput(stream, inputDevice, inputDriverInfo, bufferSize, time_proc, time_info) {
  return call('Pm_OpenInput', stream, inputDevice, inputDriverInfo, bufferSize, time_proc, time_info);
}
/**
 * @function Pm_OpenOutput
 *
 * @param    {ArrayBuffer}   stream
 * @param    {Number}        outputDevice
 * @param    {Object|null}   outputDriverInfo
 * @param    {Number}        bufferSize
 * @param    {Number}        time_proc
 * @param    {Number}        time_info
 * @param    {Number}        latency
 *
 * @return   {Number}
 */
define(
  'Pm_OpenOutput',
  dlsym(libportmidi, 'Pm_OpenOutput'),
  null,
  'int',
  'void *',
  'int',
  'void *',
  'int',
  'void *',
  'void *',
  'int',
);
function Pm_OpenOutput(stream, outputDevice, outputDriverInfo, bufferSize, time_proc, time_info, latency) {
  return call('Pm_OpenOutput', stream, outputDevice, outputDriverInfo, bufferSize, time_proc, time_info, latency);
}
/**
 * @function Pm_SetFilter
 *
 * @param    {ArrayBuffer}   stream
 * @param    {Number}        filters
 *
 * @return   {Number}
 */
define('Pm_SetFilter', dlsym(libportmidi, 'Pm_SetFilter'), null, 'int', 'void *', 'int');
function Pm_SetFilter(stream, filters) {
  return call('Pm_SetFilter', stream, filters);
}
/**
 * @function Pm_SetChannelMask
 *
 * @param    {ArrayBuffer}   stream
 * @param    {Number}        mask
 *
 * @return   {Number}
 */
define('Pm_SetChannelMask', dlsym(libportmidi, 'Pm_SetChannelMask'), null, 'int', 'void *', 'int');
function Pm_SetChannelMask(stream, mask) {
  return call('Pm_SetChannelMask', stream, mask);
}
/**
 * @function Pm_Abort
 *
 * @param    {ArrayBuffer}   stream
 *
 * @return   {Number}
 */
define('Pm_Abort', dlsym(libportmidi, 'Pm_Abort'), null, 'int', 'void *');
function Pm_Abort(stream) {
  return call('Pm_Abort', stream);
}
/**
 * @function Pm_Close
 *
 * @param    {ArrayBuffer}   stream
 *
 * @return   {Number}
 */
define('Pm_Close', dlsym(libportmidi, 'Pm_Close'), null, 'int', 'void *');
function Pm_Close(stream) {
  return call('Pm_Close', stream);
}
/**
 * @function Pm_Synchronize
 *
 * @param    {ArrayBuffer}   stream
 *
 * @return   {Number}
 */
define('Pm_Synchronize', dlsym(libportmidi, 'Pm_Synchronize'), null, 'int', 'void *');
function Pm_Synchronize(stream) {
  return call('Pm_Synchronize', stream);
}
/**
 * @function Pm_Read
 *
 * @param    {ArrayBuffer}   stream
 * @param    {ArrayBuffer}   buffer
 * @param    {Number}        length
 *
 * @return   {Number}
 */
define('Pm_Read', dlsym(libportmidi, 'Pm_Read'), null, 'int', 'void *', 'void *', 'int');
function Pm_Read(stream, buffer, length) {
  return call('Pm_Read', stream, buffer, length);
}
/**
 * @function Pm_Poll
 *
 * @param    {ArrayBuffer}   stream
 *
 * @return   {Number}
 */
define('Pm_Poll', dlsym(libportmidi, 'Pm_Poll'), null, 'int', 'void *');
function Pm_Poll(stream) {
  return call('Pm_Poll', stream);
}
/**
 * @function Pm_Write
 *
 * @param    {ArrayBuffer}   stream
 * @param    {ArrayBuffer}   buffer
 * @param    {Number}        length
 *
 * @return   {Number}
 */
define('Pm_Write', dlsym(libportmidi, 'Pm_Write'), null, 'int', 'void *', 'void *', 'int');
function Pm_Write(stream, buffer, length) {
  return call('Pm_Write', stream, buffer, length);
}
/**
 * @function Pm_WriteShort
 *
 * @param    {ArrayBuffer}   stream
 * @param    {Number}        when
 * @param    {Number}        msg
 *
 * @return   {Number}
 */
define('Pm_WriteShort', dlsym(libportmidi, 'Pm_WriteShort'), null, 'int', 'void *', 'int', 'int');
function Pm_WriteShort(stream, when, msg) {
  return call('Pm_WriteShort', stream, when, msg);
}
/**
 * @function Pm_WriteSysEx
 *
 * @param    {ArrayBuffer}   stream
 * @param    {Number}        when
 * @param    {Number}        msg
 *
 * @return   {Number}
 */
define('Pm_WriteSysEx', dlsym(libportmidi, 'Pm_WriteSysEx'), null, 'int', 'void *', 'int', 'void *');
function Pm_WriteSysEx(stream, when, msg) {
  return call('Pm_WriteSysEx', stream, when, msg);
}
/**
 * @function Pm_QueueCreate
 *
 * @param    {Number}        num_msgs
 * @param    {Number}        bytes_per_msg
 *
 * @return   {Number}
 */
define('Pm_QueueCreate', dlsym(libportmidi, 'Pm_QueueCreate'), null, 'void *', 'long', 'int');
function Pm_QueueCreate(num_msgs, bytes_per_msg) {
  return call('Pm_QueueCreate', num_msgs, bytes_per_msg);
}
/**
 * @function Pm_QueueDestroy
 *
 * @param    {Number}        queue
 *
 * @return   {Number}
 */
define('Pm_QueueDestroy', dlsym(libportmidi, 'Pm_QueueDestroy'), null, 'int', 'void *');
function Pm_QueueDestroy(queue) {
  return call('Pm_QueueDestroy', queue);
}
/**
 * @function Pm_Dequeue
 *
 * @param    {Number}        queue
 * @param    {Number}        msg
 *
 * @return   {Number}
 */
define('Pm_Dequeue', dlsym(libportmidi, 'Pm_Dequeue'), null, 'int', 'void *', 'void *');
function Pm_Dequeue(queue, msg) {
  return call('Pm_Dequeue', queue, msg);
}
/**
 * @function Pm_Enqueue
 *
 * @param    {Number}        queue
 * @param    {Number}        msg
 *
 * @return   {Number}
 */
define('Pm_Enqueue', dlsym(libportmidi, 'Pm_Enqueue'), null, 'int', 'void *', 'void *');
function Pm_Enqueue(queue, msg) {
  return call('Pm_Enqueue', queue, msg);
}
/**
 * @function Pm_QueueFull
 *
 * @param    {Number}        queue
 *
 * @return   {Number}
 */
define('Pm_QueueFull', dlsym(libportmidi, 'Pm_QueueFull'), null, 'int', 'void *');
function Pm_QueueFull(queue) {
  return call('Pm_QueueFull', queue);
}
/**
 * @function Pm_QueueEmpty
 *
 * @param    {Number}        queue
 *
 * @return   {Number}
 */
define('Pm_QueueEmpty', dlsym(libportmidi, 'Pm_QueueEmpty'), null, 'int', 'void *');
function Pm_QueueEmpty(queue) {
  return call('Pm_QueueEmpty', queue);
}
/**
 * @function Pm_QueuePeek
 *
 * @param    {Number}        queue
 *
 * @return   {Number}
 */
define('Pm_QueuePeek', dlsym(libportmidi, 'Pm_QueuePeek'), null, 'void *', 'void *');
function Pm_QueuePeek(queue) {
  return call('Pm_QueuePeek', queue);
}
/**
 * @function Pm_SetOverflow
 *
 * @param    {Number}        queue
 *
 * @return   {Number}
 */
define('Pm_SetOverflow', dlsym(libportmidi, 'Pm_SetOverflow'), null, 'int', 'void *');
function Pm_SetOverflow(queue) {
  return call('Pm_SetOverflow', queue);
}
/**
 * @function Pt_Start
 *
 * @param    {Number}        resolution
 * @param    {Number}        callback
 * @param    {Number}        userData
 *
 * @return   {Number}
 */
define('Pt_Start', dlsym(libportmidi, 'Pt_Start'), null, 'int', 'int', 'void *', 'void *');
function Pt_Start(resolution, callback, userData) {
  return call('Pt_Start', resolution, callback, userData);
}
/**
 * @function Pt_Stop
 *
 * @return   {Number}
 */
define('Pt_Stop', dlsym(libportmidi, 'Pt_Stop'), null, 'int');
function Pt_Stop() {
  return call('Pt_Stop');
}
/**
 * @function Pt_Started
 *
 * @return   {Number}
 */
define('Pt_Started', dlsym(libportmidi, 'Pt_Started'), null, 'int');
function Pt_Started() {
  return call('Pt_Started');
}
