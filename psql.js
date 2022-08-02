import { dlopen, define, dlerror, dlclose, dlsym, call, errno, RTLD_NOW } from 'ffi';

const libpq = dlopen('libpq.so', RTLD_NOW);

/**       
 * @function PQconnectStart  
 * 
 * @param    {String}        conninfo
 * 
 * @return   {Number}        
 */
define('PQconnectStart', dlsym(libpq, 'PQconnectStart'), null, 'void *', 'char *');
function PQconnectStart(conninfo) {
  return call('PQconnectStart', conninfo);
}

/**       
 * @function PQconnectStartParams
 * 
 * @param    {Number}        keywords
 * @param    {Number}        values
 * @param    {Number}        expand_dbname
 * 
 * @return   {Number}        
 */
define('PQconnectStartParams', dlsym(libpq, 'PQconnectStartParams'), null, 'void *', 'long', 'long', 'int');
function PQconnectStartParams(keywords, values, expand_dbname) {
  return call('PQconnectStartParams', keywords, values, expand_dbname);
}


/**       
 * @function PQconnectdb     
 * 
 * @param    {String}        conninfo
 * 
 * @return   {Number}        
 */
define('PQconnectdb', dlsym(libpq, 'PQconnectdb'), null, 'void *', 'char *');
function PQconnectdb(conninfo) {
  return call('PQconnectdb', conninfo);
}

/**       
 * @function PQconnectdbParams
 * 
 * @param    {Number}        keywords
 * @param    {Number}        values
 * @param    {Number}        expand_dbname
 * 
 * @return   {Number}        
 */
define('PQconnectdbParams', dlsym(libpq, 'PQconnectdbParams'), null, 'void *', 'long', 'long', 'int');
function PQconnectdbParams(keywords, values, expand_dbname) {
  return call('PQconnectdbParams', keywords, values, expand_dbname);
}

/**       
 * @function PQsetdbLogin    
 * 
 * @param    {String}        pghost
 * @param    {String}        pgport
 * @param    {String}        pgoptions
 * @param    {String}        pgtty
 * @param    {String}        dbName
 * @param    {String}        login
 * @param    {String}        pwd
 * 
 * @return   {Number}        
 */
define('PQsetdbLogin', dlsym(libpq, 'PQsetdbLogin'), null, 'void *', 'char *', 'char *', 'char *', 'char *', 'char *', 'char *', 'char *');
function PQsetdbLogin(pghost, pgport, pgoptions, pgtty, dbName, login, pwd) {
  return call('PQsetdbLogin', pghost, pgport, pgoptions, pgtty, dbName, login, pwd);
}

/**       
 * @function PQfinish        
 * 
 * @param    {Number}        conn
 */
define('PQfinish', dlsym(libpq, 'PQfinish'), null, 'void', 'long');
function PQfinish(conn) {
  call('PQfinish', conn);
}

/**       
 * @function PQconndefaults  
 * 
 * @return   {Number}        
 */
define('PQconndefaults', dlsym(libpq, 'PQconndefaults'), null, 'void *');
function PQconndefaults() {
  return call('PQconndefaults', );
}

/**       
 * @function PQconninfoParse 
 * 
 * @param    {String}        conninfo
 * @param    {Number}        errmsg
 * 
 * @return   {Number}        
 */
define('PQconninfoParse', dlsym(libpq, 'PQconninfoParse'), null, 'void *', 'char *', 'long');
function PQconninfoParse(conninfo, errmsg) {
  return call('PQconninfoParse', conninfo, errmsg);
}

/**       
 * @function PQconninfo      
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQconninfo', dlsym(libpq, 'PQconninfo'), null, 'void *', 'long');
function PQconninfo(conn) {
  return call('PQconninfo', conn);
}

/**       
 * @function PQconninfoFree  
 * 
 * @param    {Number}        connOptions
 */
define('PQconninfoFree', dlsym(libpq, 'PQconninfoFree'), null, 'void', 'long');
function PQconninfoFree(connOptions) {
  call('PQconninfoFree', connOptions);
}

/**       
 * @function PQresetStart    
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQresetStart', dlsym(libpq, 'PQresetStart'), null, 'int', 'long');
function PQresetStart(conn) {
  return call('PQresetStart', conn);
}


/**       
 * @function PQreset         
 * 
 * @param    {Number}        conn
 */
define('PQreset', dlsym(libpq, 'PQreset'), null, 'void', 'long');
function PQreset(conn) {
  call('PQreset', conn);
}

/**       
 * @function PQgetCancel     
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQgetCancel', dlsym(libpq, 'PQgetCancel'), null, 'void *', 'long');
function PQgetCancel(conn) {
  return call('PQgetCancel', conn);
}

/**       
 * @function PQfreeCancel    
 * 
 * @param    {Number}        cancel
 */
define('PQfreeCancel', dlsym(libpq, 'PQfreeCancel'), null, 'void', 'long');
function PQfreeCancel(cancel) {
  call('PQfreeCancel', cancel);
}

/**       
 * @function PQcancel        
 * 
 * @param    {Number}        cancel
 * @param    {String}        errbuf
 * @param    {Number}        errbufsize
 * 
 * @return   {Number}        
 */
define('PQcancel', dlsym(libpq, 'PQcancel'), null, 'int', 'long', 'char *', 'int');
function PQcancel(cancel, errbuf, errbufsize) {
  return call('PQcancel', cancel, errbuf, errbufsize);
}

/**       
 * @function PQrequestCancel 
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQrequestCancel', dlsym(libpq, 'PQrequestCancel'), null, 'int', 'long');
function PQrequestCancel(conn) {
  return call('PQrequestCancel', conn);
}

/**       
 * @function PQdb            
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQdb', dlsym(libpq, 'PQdb'), null, 'char *', 'long');
function PQdb(conn) {
  return call('PQdb', conn);
}

/**       
 * @function PQuser          
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQuser', dlsym(libpq, 'PQuser'), null, 'char *', 'long');
function PQuser(conn) {
  return call('PQuser', conn);
}

/**       
 * @function PQpass          
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQpass', dlsym(libpq, 'PQpass'), null, 'char *', 'long');
function PQpass(conn) {
  return call('PQpass', conn);
}

/**       
 * @function PQhost          
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQhost', dlsym(libpq, 'PQhost'), null, 'char *', 'long');
function PQhost(conn) {
  return call('PQhost', conn);
}

/**       
 * @function PQhostaddr      
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQhostaddr', dlsym(libpq, 'PQhostaddr'), null, 'char *', 'long');
function PQhostaddr(conn) {
  return call('PQhostaddr', conn);
}

/**       
 * @function PQport          
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQport', dlsym(libpq, 'PQport'), null, 'char *', 'long');
function PQport(conn) {
  return call('PQport', conn);
}

/**       
 * @function PQtty           
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQtty', dlsym(libpq, 'PQtty'), null, 'char *', 'long');
function PQtty(conn) {
  return call('PQtty', conn);
}

/**       
 * @function PQoptions       
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQoptions', dlsym(libpq, 'PQoptions'), null, 'char *', 'long');
function PQoptions(conn) {
  return call('PQoptions', conn);
}



/**       
 * @function PQparameterStatus
 * 
 * @param    {Number}        conn
 * @param    {String}        paramName
 * 
 * @return   {String}        
 */
define('PQparameterStatus', dlsym(libpq, 'PQparameterStatus'), null, 'char *', 'long', 'char *');
function PQparameterStatus(conn, paramName) {
  return call('PQparameterStatus', conn, paramName);
}

/**       
 * @function PQprotocolVersion
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQprotocolVersion', dlsym(libpq, 'PQprotocolVersion'), null, 'int', 'long');
function PQprotocolVersion(conn) {
  return call('PQprotocolVersion', conn);
}

/**       
 * @function PQserverVersion 
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQserverVersion', dlsym(libpq, 'PQserverVersion'), null, 'int', 'long');
function PQserverVersion(conn) {
  return call('PQserverVersion', conn);
}

/**       
 * @function PQerrorMessage  
 * 
 * @param    {Number}        conn
 * 
 * @return   {String}        
 */
define('PQerrorMessage', dlsym(libpq, 'PQerrorMessage'), null, 'char *', 'long');
function PQerrorMessage(conn) {
  return call('PQerrorMessage', conn);
}

/**       
 * @function PQsocket        
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQsocket', dlsym(libpq, 'PQsocket'), null, 'int', 'long');
function PQsocket(conn) {
  return call('PQsocket', conn);
}

/**       
 * @function PQbackendPID    
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQbackendPID', dlsym(libpq, 'PQbackendPID'), null, 'int', 'long');
function PQbackendPID(conn) {
  return call('PQbackendPID', conn);
}

/**       
 * @function PQconnectionNeedsPassword
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQconnectionNeedsPassword', dlsym(libpq, 'PQconnectionNeedsPassword'), null, 'int', 'long');
function PQconnectionNeedsPassword(conn) {
  return call('PQconnectionNeedsPassword', conn);
}

/**       
 * @function PQconnectionUsedPassword
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQconnectionUsedPassword', dlsym(libpq, 'PQconnectionUsedPassword'), null, 'int', 'long');
function PQconnectionUsedPassword(conn) {
  return call('PQconnectionUsedPassword', conn);
}

/**       
 * @function PQclientEncoding
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQclientEncoding', dlsym(libpq, 'PQclientEncoding'), null, 'int', 'long');
function PQclientEncoding(conn) {
  return call('PQclientEncoding', conn);
}

/**       
 * @function PQsetClientEncoding
 * 
 * @param    {Number}        conn
 * @param    {String}        encoding
 * 
 * @return   {Number}        
 */
define('PQsetClientEncoding', dlsym(libpq, 'PQsetClientEncoding'), null, 'int', 'long', 'char *');
function PQsetClientEncoding(conn, encoding) {
  return call('PQsetClientEncoding', conn, encoding);
}

/**       
 * @function PQsslInUse      
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQsslInUse', dlsym(libpq, 'PQsslInUse'), null, 'int', 'long');
function PQsslInUse(conn) {
  return call('PQsslInUse', conn);
}

/**       
 * @function PQsslStruct     
 * 
 * @param    {Number}        conn
 * @param    {String}        struct_name
 * 
 * @return   {Number}        
 */
define('PQsslStruct', dlsym(libpq, 'PQsslStruct'), null, 'long', 'long', 'char *');
function PQsslStruct(conn, struct_name) {
  return call('PQsslStruct', conn, struct_name);
}

/**       
 * @function PQsslAttribute  
 * 
 * @param    {Number}        conn
 * @param    {String}        attribute_name
 * 
 * @return   {String}        
 */
define('PQsslAttribute', dlsym(libpq, 'PQsslAttribute'), null, 'char *', 'long', 'char *');
function PQsslAttribute(conn, attribute_name) {
  return call('PQsslAttribute', conn, attribute_name);
}

/**       
 * @function PQsslAttributeNames
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQsslAttributeNames', dlsym(libpq, 'PQsslAttributeNames'), null, 'void *', 'long');
function PQsslAttributeNames(conn) {
  return call('PQsslAttributeNames', conn);
}

/**       
 * @function PQgetssl        
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQgetssl', dlsym(libpq, 'PQgetssl'), null, 'long', 'long');
function PQgetssl(conn) {
  return call('PQgetssl', conn);
}

/**       
 * @function PQinitSSL       
 * 
 * @param    {Number}        do_init
 */
define('PQinitSSL', dlsym(libpq, 'PQinitSSL'), null, 'void', 'int');
function PQinitSSL(do_init) {
  call('PQinitSSL', do_init);
}

/**       
 * @function PQinitOpenSSL   
 * 
 * @param    {Number}        do_ssl
 * @param    {Number}        do_crypto
 */
define('PQinitOpenSSL', dlsym(libpq, 'PQinitOpenSSL'), null, 'void', 'int', 'int');
function PQinitOpenSSL(do_ssl, do_crypto) {
  call('PQinitOpenSSL', do_ssl, do_crypto);
}

/**       
 * @function PQgssEncInUse   
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQgssEncInUse', dlsym(libpq, 'PQgssEncInUse'), null, 'int', 'long');
function PQgssEncInUse(conn) {
  return call('PQgssEncInUse', conn);
}

/**       
 * @function PQgetgssctx     
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQgetgssctx', dlsym(libpq, 'PQgetgssctx'), null, 'long', 'long');
function PQgetgssctx(conn) {
  return call('PQgetgssctx', conn);
}



/**       
 * @function PQtrace         
 * 
 * @param    {Number}        conn
 * @param    {Number}        debug_port
 */
define('PQtrace', dlsym(libpq, 'PQtrace'), null, 'void', 'long', 'long');
function PQtrace(conn, debug_port) {
  call('PQtrace', conn, debug_port);
}

/**       
 * @function PQuntrace       
 * 
 * @param    {Number}        conn
 */
define('PQuntrace', dlsym(libpq, 'PQuntrace'), null, 'void', 'long');
function PQuntrace(conn) {
  call('PQuntrace', conn);
}




/**       
 * @function PQexec          
 * 
 * @param    {Number}        conn
 * @param    {String}        query
 * 
 * @return   {Number}        
 */
define('PQexec', dlsym(libpq, 'PQexec'), null, 'void *', 'long', 'char *');
function PQexec(conn, query) {
  return call('PQexec', conn, query);
}

/**       
 * @function PQexecParams    
 * 
 * @param    {Number}        conn
 * @param    {String}        command
 * @param    {Number}        nParams
 * @param    {Number}        paramTypes
 * @param    {Number}        paramValues
 * @param    {Number}        paramLengths
 * @param    {Number}        paramFormats
 * @param    {Number}        resultFormat
 * 
 * @return   {Number}        
 */
define('PQexecParams', dlsym(libpq, 'PQexecParams'), null, 'void *', 'long', 'char *', 'int', 'long', 'long', 'long', 'long', 'int');
function PQexecParams(conn, command, nParams, paramTypes, paramValues, paramLengths, paramFormats, resultFormat) {
  return call('PQexecParams', conn, command, nParams, paramTypes, paramValues, paramLengths, paramFormats, resultFormat);
}

/**       
 * @function PQprepare       
 * 
 * @param    {Number}        conn
 * @param    {String}        stmtName
 * @param    {String}        query
 * @param    {Number}        nParams
 * @param    {Number}        paramTypes
 * 
 * @return   {Number}        
 */
define('PQprepare', dlsym(libpq, 'PQprepare'), null, 'void *', 'long', 'char *', 'char *', 'int', 'long');
function PQprepare(conn, stmtName, query, nParams, paramTypes) {
  return call('PQprepare', conn, stmtName, query, nParams, paramTypes);
}

/**       
 * @function PQexecPrepared  
 * 
 * @param    {Number}        conn
 * @param    {String}        stmtName
 * @param    {Number}        nParams
 * @param    {Number}        paramValues
 * @param    {Number}        paramLengths
 * @param    {Number}        paramFormats
 * @param    {Number}        resultFormat
 * 
 * @return   {Number}        
 */
define('PQexecPrepared', dlsym(libpq, 'PQexecPrepared'), null, 'void *', 'long', 'char *', 'int', 'long', 'long', 'long', 'int');
function PQexecPrepared(conn, stmtName, nParams, paramValues, paramLengths, paramFormats, resultFormat) {
  return call('PQexecPrepared', conn, stmtName, nParams, paramValues, paramLengths, paramFormats, resultFormat);
}

/**       
 * @function PQsendQuery     
 * 
 * @param    {Number}        conn
 * @param    {String}        query
 * 
 * @return   {Number}        
 */
define('PQsendQuery', dlsym(libpq, 'PQsendQuery'), null, 'int', 'long', 'char *');
function PQsendQuery(conn, query) {
  return call('PQsendQuery', conn, query);
}

/**       
 * @function PQsendQueryParams
 * 
 * @param    {Number}        conn
 * @param    {String}        command
 * @param    {Number}        nParams
 * @param    {Number}        paramTypes
 * @param    {Number}        paramValues
 * @param    {Number}        paramLengths
 * @param    {Number}        paramFormats
 * @param    {Number}        resultFormat
 * 
 * @return   {Number}        
 */
define('PQsendQueryParams', dlsym(libpq, 'PQsendQueryParams'), null, 'int', 'long', 'char *', 'int', 'long', 'long', 'long', 'long', 'int');
function PQsendQueryParams(conn, command, nParams, paramTypes, paramValues, paramLengths, paramFormats, resultFormat) {
  return call('PQsendQueryParams', conn, command, nParams, paramTypes, paramValues, paramLengths, paramFormats, resultFormat);
}

/**       
 * @function PQsendPrepare   
 * 
 * @param    {Number}        conn
 * @param    {String}        stmtName
 * @param    {String}        query
 * @param    {Number}        nParams
 * @param    {Number}        paramTypes
 * 
 * @return   {Number}        
 */
define('PQsendPrepare', dlsym(libpq, 'PQsendPrepare'), null, 'int', 'long', 'char *', 'char *', 'int', 'long');
function PQsendPrepare(conn, stmtName, query, nParams, paramTypes) {
  return call('PQsendPrepare', conn, stmtName, query, nParams, paramTypes);
}

/**       
 * @function PQsendQueryPrepared
 * 
 * @param    {Number}        conn
 * @param    {String}        stmtName
 * @param    {Number}        nParams
 * @param    {Number}        paramValues
 * @param    {Number}        paramLengths
 * @param    {Number}        paramFormats
 * @param    {Number}        resultFormat
 * 
 * @return   {Number}        
 */
define('PQsendQueryPrepared', dlsym(libpq, 'PQsendQueryPrepared'), null, 'int', 'long', 'char *', 'int', 'long', 'long', 'long', 'int');
function PQsendQueryPrepared(conn, stmtName, nParams, paramValues, paramLengths, paramFormats, resultFormat) {
  return call('PQsendQueryPrepared', conn, stmtName, nParams, paramValues, paramLengths, paramFormats, resultFormat);
}

/**       
 * @function PQsetSingleRowMode
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQsetSingleRowMode', dlsym(libpq, 'PQsetSingleRowMode'), null, 'int', 'long');
function PQsetSingleRowMode(conn) {
  return call('PQsetSingleRowMode', conn);
}

/**       
 * @function PQgetResult     
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQgetResult', dlsym(libpq, 'PQgetResult'), null, 'void *', 'long');
function PQgetResult(conn) {
  return call('PQgetResult', conn);
}

/**       
 * @function PQisBusy        
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQisBusy', dlsym(libpq, 'PQisBusy'), null, 'int', 'long');
function PQisBusy(conn) {
  return call('PQisBusy', conn);
}

/**       
 * @function PQconsumeInput  
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQconsumeInput', dlsym(libpq, 'PQconsumeInput'), null, 'int', 'long');
function PQconsumeInput(conn) {
  return call('PQconsumeInput', conn);
}

/**       
 * @function PQnotifies      
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQnotifies', dlsym(libpq, 'PQnotifies'), null, 'void *', 'long');
function PQnotifies(conn) {
  return call('PQnotifies', conn);
}

/**       
 * @function PQputCopyData   
 * 
 * @param    {Number}        conn
 * @param    {String}        buffer
 * @param    {Number}        nbytes
 * 
 * @return   {Number}        
 */
define('PQputCopyData', dlsym(libpq, 'PQputCopyData'), null, 'int', 'long', 'char *', 'int');
function PQputCopyData(conn, buffer, nbytes) {
  return call('PQputCopyData', conn, buffer, nbytes);
}

/**       
 * @function PQputCopyEnd    
 * 
 * @param    {Number}        conn
 * @param    {String}        errormsg
 * 
 * @return   {Number}        
 */
define('PQputCopyEnd', dlsym(libpq, 'PQputCopyEnd'), null, 'int', 'long', 'char *');
function PQputCopyEnd(conn, errormsg) {
  return call('PQputCopyEnd', conn, errormsg);
}

/**       
 * @function PQgetCopyData   
 * 
 * @param    {Number}        conn
 * @param    {Number}        buffer
 * @param    {Number}        async
 * 
 * @return   {Number}        
 */
define('PQgetCopyData', dlsym(libpq, 'PQgetCopyData'), null, 'int', 'long', 'long', 'int');
function PQgetCopyData(conn, buffer, async) {
  return call('PQgetCopyData', conn, buffer, async);
}

/**       
 * @function PQgetline       
 * 
 * @param    {Number}        conn
 * @param    {String}        string
 * @param    {Number}        length
 * 
 * @return   {Number}        
 */
define('PQgetline', dlsym(libpq, 'PQgetline'), null, 'int', 'long', 'char *', 'int');
function PQgetline(conn, string, length) {
  return call('PQgetline', conn, string, length);
}

/**       
 * @function PQputline       
 * 
 * @param    {Number}        conn
 * @param    {String}        string
 * 
 * @return   {Number}        
 */
define('PQputline', dlsym(libpq, 'PQputline'), null, 'int', 'long', 'char *');
function PQputline(conn, string) {
  return call('PQputline', conn, string);
}

/**       
 * @function PQgetlineAsync  
 * 
 * @param    {Number}        conn
 * @param    {String}        buffer
 * @param    {Number}        bufsize
 * 
 * @return   {Number}        
 */
define('PQgetlineAsync', dlsym(libpq, 'PQgetlineAsync'), null, 'int', 'long', 'char *', 'int');
function PQgetlineAsync(conn, buffer, bufsize) {
  return call('PQgetlineAsync', conn, buffer, bufsize);
}

/**       
 * @function PQputnbytes     
 * 
 * @param    {Number}        conn
 * @param    {String}        buffer
 * @param    {Number}        nbytes
 * 
 * @return   {Number}        
 */
define('PQputnbytes', dlsym(libpq, 'PQputnbytes'), null, 'int', 'long', 'char *', 'int');
function PQputnbytes(conn, buffer, nbytes) {
  return call('PQputnbytes', conn, buffer, nbytes);
}

/**       
 * @function PQendcopy       
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQendcopy', dlsym(libpq, 'PQendcopy'), null, 'int', 'long');
function PQendcopy(conn) {
  return call('PQendcopy', conn);
}

/**       
 * @function PQsetnonblocking
 * 
 * @param    {Number}        conn
 * @param    {Number}        arg
 * 
 * @return   {Number}        
 */
define('PQsetnonblocking', dlsym(libpq, 'PQsetnonblocking'), null, 'int', 'long', 'int');
function PQsetnonblocking(conn, arg) {
  return call('PQsetnonblocking', conn, arg);
}

/**       
 * @function PQisnonblocking 
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQisnonblocking', dlsym(libpq, 'PQisnonblocking'), null, 'int', 'long');
function PQisnonblocking(conn) {
  return call('PQisnonblocking', conn);
}

/**       
 * @function PQisthreadsafe  
 * 
 * @return   {Number}        
 */
define('PQisthreadsafe', dlsym(libpq, 'PQisthreadsafe'), null, 'int');
function PQisthreadsafe() {
  return call('PQisthreadsafe', );
}



/**       
 * @function PQflush         
 * 
 * @param    {Number}        conn
 * 
 * @return   {Number}        
 */
define('PQflush', dlsym(libpq, 'PQflush'), null, 'int', 'long');
function PQflush(conn) {
  return call('PQflush', conn);
}

/**       
 * @function PQfn            
 * 
 * @param    {Number}        conn
 * @param    {Number}        fnid
 * @param    {Number}        result_buf
 * @param    {Number}        result_len
 * @param    {Number}        result_is_int
 * @param    {Number}        args
 * @param    {Number}        nargs
 * 
 * @return   {Number}        
 */
define('PQfn', dlsym(libpq, 'PQfn'), null, 'void *', 'long', 'int', 'long', 'long', 'int', 'long', 'int');
function PQfn(conn, fnid, result_buf, result_len, result_is_int, args, nargs) {
  return call('PQfn', conn, fnid, result_buf, result_len, result_is_int, args, nargs);
}


/**       
 * @function PQresStatus     
 * 
 * @param    {Number}        status
 * 
 * @return   {String}        
 */
define('PQresStatus', dlsym(libpq, 'PQresStatus'), null, 'char *', 'int');
function PQresStatus(status) {
  return call('PQresStatus', status);
}

/**       
 * @function PQresultErrorMessage
 * 
 * @param    {Number}        res
 * 
 * @return   {String}        
 */
define('PQresultErrorMessage', dlsym(libpq, 'PQresultErrorMessage'), null, 'char *', 'long');
function PQresultErrorMessage(res) {
  return call('PQresultErrorMessage', res);
}

/**       
 * @function PQresultVerboseErrorMessage
 * 
 * @param    {Number}        res
 * @param    {Number}        verbosity
 * @param    {Number}        show_context
 * 
 * @return   {String}        
 */
define('PQresultVerboseErrorMessage', dlsym(libpq, 'PQresultVerboseErrorMessage'), null, 'char *', 'long', 'int', 'int');
function PQresultVerboseErrorMessage(res, verbosity, show_context) {
  return call('PQresultVerboseErrorMessage', res, verbosity, show_context);
}

/**       
 * @function PQresultErrorField
 * 
 * @param    {Number}        res
 * @param    {Number}        fieldcode
 * 
 * @return   {String}        
 */
define('PQresultErrorField', dlsym(libpq, 'PQresultErrorField'), null, 'char *', 'long', 'int');
function PQresultErrorField(res, fieldcode) {
  return call('PQresultErrorField', res, fieldcode);
}

/**       
 * @function PQntuples       
 * 
 * @param    {Number}        res
 * 
 * @return   {Number}        
 */
define('PQntuples', dlsym(libpq, 'PQntuples'), null, 'int', 'long');
function PQntuples(res) {
  return call('PQntuples', res);
}

/**       
 * @function PQnfields       
 * 
 * @param    {Number}        res
 * 
 * @return   {Number}        
 */
define('PQnfields', dlsym(libpq, 'PQnfields'), null, 'int', 'long');
function PQnfields(res) {
  return call('PQnfields', res);
}

/**       
 * @function PQbinaryTuples  
 * 
 * @param    {Number}        res
 * 
 * @return   {Number}        
 */
define('PQbinaryTuples', dlsym(libpq, 'PQbinaryTuples'), null, 'int', 'long');
function PQbinaryTuples(res) {
  return call('PQbinaryTuples', res);
}

/**       
 * @function PQfname         
 * 
 * @param    {Number}        res
 * @param    {Number}        field_num
 * 
 * @return   {String}        
 */
define('PQfname', dlsym(libpq, 'PQfname'), null, 'char *', 'long', 'int');
function PQfname(res, field_num) {
  return call('PQfname', res, field_num);
}

/**       
 * @function PQfnumber       
 * 
 * @param    {Number}        res
 * @param    {String}        field_name
 * 
 * @return   {Number}        
 */
define('PQfnumber', dlsym(libpq, 'PQfnumber'), null, 'int', 'long', 'char *');
function PQfnumber(res, field_name) {
  return call('PQfnumber', res, field_name);
}


/**       
 * @function PQftablecol     
 * 
 * @param    {Number}        res
 * @param    {Number}        field_num
 * 
 * @return   {Number}        
 */
define('PQftablecol', dlsym(libpq, 'PQftablecol'), null, 'int', 'long', 'int');
function PQftablecol(res, field_num) {
  return call('PQftablecol', res, field_num);
}

/**       
 * @function PQfformat       
 * 
 * @param    {Number}        res
 * @param    {Number}        field_num
 * 
 * @return   {Number}        
 */
define('PQfformat', dlsym(libpq, 'PQfformat'), null, 'int', 'long', 'int');
function PQfformat(res, field_num) {
  return call('PQfformat', res, field_num);
}


/**       
 * @function PQfsize         
 * 
 * @param    {Number}        res
 * @param    {Number}        field_num
 * 
 * @return   {Number}        
 */
define('PQfsize', dlsym(libpq, 'PQfsize'), null, 'int', 'long', 'int');
function PQfsize(res, field_num) {
  return call('PQfsize', res, field_num);
}

/**       
 * @function PQfmod          
 * 
 * @param    {Number}        res
 * @param    {Number}        field_num
 * 
 * @return   {Number}        
 */
define('PQfmod', dlsym(libpq, 'PQfmod'), null, 'int', 'long', 'int');
function PQfmod(res, field_num) {
  return call('PQfmod', res, field_num);
}

/**       
 * @function PQcmdStatus     
 * 
 * @param    {Number}        res
 * 
 * @return   {String}        
 */
define('PQcmdStatus', dlsym(libpq, 'PQcmdStatus'), null, 'char *', 'long');
function PQcmdStatus(res) {
  return call('PQcmdStatus', res);
}

/**       
 * @function PQoidStatus     
 * 
 * @param    {Number}        res
 * 
 * @return   {String}        
 */
define('PQoidStatus', dlsym(libpq, 'PQoidStatus'), null, 'char *', 'long');
function PQoidStatus(res) {
  return call('PQoidStatus', res);
}


/**       
 * @function PQcmdTuples     
 * 
 * @param    {Number}        res
 * 
 * @return   {String}        
 */
define('PQcmdTuples', dlsym(libpq, 'PQcmdTuples'), null, 'char *', 'long');
function PQcmdTuples(res) {
  return call('PQcmdTuples', res);
}

/**       
 * @function PQgetvalue      
 * 
 * @param    {Number}        res
 * @param    {Number}        tup_num
 * @param    {Number}        field_num
 * 
 * @return   {String}        
 */
define('PQgetvalue', dlsym(libpq, 'PQgetvalue'), null, 'char *', 'long', 'int', 'int');
function PQgetvalue(res, tup_num, field_num) {
  return call('PQgetvalue', res, tup_num, field_num);
}

/**       
 * @function PQgetlength     
 * 
 * @param    {Number}        res
 * @param    {Number}        tup_num
 * @param    {Number}        field_num
 * 
 * @return   {Number}        
 */
define('PQgetlength', dlsym(libpq, 'PQgetlength'), null, 'int', 'long', 'int', 'int');
function PQgetlength(res, tup_num, field_num) {
  return call('PQgetlength', res, tup_num, field_num);
}

/**       
 * @function PQgetisnull     
 * 
 * @param    {Number}        res
 * @param    {Number}        tup_num
 * @param    {Number}        field_num
 * 
 * @return   {Number}        
 */
define('PQgetisnull', dlsym(libpq, 'PQgetisnull'), null, 'int', 'long', 'int', 'int');
function PQgetisnull(res, tup_num, field_num) {
  return call('PQgetisnull', res, tup_num, field_num);
}

/**       
 * @function PQnparams       
 * 
 * @param    {Number}        res
 * 
 * @return   {Number}        
 */
define('PQnparams', dlsym(libpq, 'PQnparams'), null, 'int', 'long');
function PQnparams(res) {
  return call('PQnparams', res);
}


/**       
 * @function PQdescribePrepared
 * 
 * @param    {Number}        conn
 * @param    {String}        stmt
 * 
 * @return   {Number}        
 */
define('PQdescribePrepared', dlsym(libpq, 'PQdescribePrepared'), null, 'void *', 'long', 'char *');
function PQdescribePrepared(conn, stmt) {
  return call('PQdescribePrepared', conn, stmt);
}

/**       
 * @function PQdescribePortal
 * 
 * @param    {Number}        conn
 * @param    {String}        portal
 * 
 * @return   {Number}        
 */
define('PQdescribePortal', dlsym(libpq, 'PQdescribePortal'), null, 'void *', 'long', 'char *');
function PQdescribePortal(conn, portal) {
  return call('PQdescribePortal', conn, portal);
}

/**       
 * @function PQsendDescribePrepared
 * 
 * @param    {Number}        conn
 * @param    {String}        stmt
 * 
 * @return   {Number}        
 */
define('PQsendDescribePrepared', dlsym(libpq, 'PQsendDescribePrepared'), null, 'int', 'long', 'char *');
function PQsendDescribePrepared(conn, stmt) {
  return call('PQsendDescribePrepared', conn, stmt);
}

/**       
 * @function PQsendDescribePortal
 * 
 * @param    {Number}        conn
 * @param    {String}        portal
 * 
 * @return   {Number}        
 */
define('PQsendDescribePortal', dlsym(libpq, 'PQsendDescribePortal'), null, 'int', 'long', 'char *');
function PQsendDescribePortal(conn, portal) {
  return call('PQsendDescribePortal', conn, portal);
}

/**       
 * @function PQclear         
 * 
 * @param    {Number}        res
 */
define('PQclear', dlsym(libpq, 'PQclear'), null, 'void', 'long');
function PQclear(res) {
  call('PQclear', res);
}

/**       
 * @function PQfreemem       
 * 
 * @param    {Number}        ptr
 */
define('PQfreemem', dlsym(libpq, 'PQfreemem'), null, 'void', 'long');
function PQfreemem(ptr) {
  call('PQfreemem', ptr);
}

/**       
 * @function PQmakeEmptyPGresult
 * 
 * @param    {Number}        conn
 * @param    {Number}        status
 * 
 * @return   {Number}        
 */
define('PQmakeEmptyPGresult', dlsym(libpq, 'PQmakeEmptyPGresult'), null, 'void *', 'long', 'int');
function PQmakeEmptyPGresult(conn, status) {
  return call('PQmakeEmptyPGresult', conn, status);
}

/**       
 * @function PQcopyResult    
 * 
 * @param    {Number}        src
 * @param    {Number}        flags
 * 
 * @return   {Number}        
 */
define('PQcopyResult', dlsym(libpq, 'PQcopyResult'), null, 'void *', 'long', 'int');
function PQcopyResult(src, flags) {
  return call('PQcopyResult', src, flags);
}

/**       
 * @function PQsetResultAttrs
 * 
 * @param    {Number}        res
 * @param    {Number}        numAttributes
 * @param    {Number}        attDescs
 * 
 * @return   {Number}        
 */
define('PQsetResultAttrs', dlsym(libpq, 'PQsetResultAttrs'), null, 'int', 'long', 'int', 'long');
function PQsetResultAttrs(res, numAttributes, attDescs) {
  return call('PQsetResultAttrs', res, numAttributes, attDescs);
}

/**       
 * @function PQresultAlloc   
 * 
 * @param    {Number}        res
 * @param    {Number}        nBytes
 * 
 * @return   {Number}        
 */
define('PQresultAlloc', dlsym(libpq, 'PQresultAlloc'), null, 'long', 'long', 'size_t');
function PQresultAlloc(res, nBytes) {
  return call('PQresultAlloc', res, nBytes);
}


/**       
 * @function PQsetvalue      
 * 
 * @param    {Number}        res
 * @param    {Number}        tup_num
 * @param    {Number}        field_num
 * @param    {String}        value
 * @param    {Number}        len
 * 
 * @return   {Number}        
 */
define('PQsetvalue', dlsym(libpq, 'PQsetvalue'), null, 'int', 'long', 'int', 'int', 'char *', 'int');
function PQsetvalue(res, tup_num, field_num, value, len) {
  return call('PQsetvalue', res, tup_num, field_num, value, len);
}


/**       
 * @function PQescapeLiteral 
 * 
 * @param    {Number}        conn
 * @param    {String}        str
 * @param    {Number}        len
 * 
 * @return   {String}        
 */
define('PQescapeLiteral', dlsym(libpq, 'PQescapeLiteral'), null, 'char *', 'long', 'char *', 'size_t');
function PQescapeLiteral(conn, str, len) {
  return call('PQescapeLiteral', conn, str, len);
}

/**       
 * @function PQescapeIdentifier
 * 
 * @param    {Number}        conn
 * @param    {String}        str
 * @param    {Number}        len
 * 
 * @return   {String}        
 */
define('PQescapeIdentifier', dlsym(libpq, 'PQescapeIdentifier'), null, 'char *', 'long', 'char *', 'size_t');
function PQescapeIdentifier(conn, str, len) {
  return call('PQescapeIdentifier', conn, str, len);
}

/**       
 * @function PQescapeByteaConn
 * 
 * @param    {Number}        conn
 * @param    {Number}        from
 * @param    {Number}        from_length
 * @param    {Number}        to_length
 * 
 * @return   {Number}        
 */
define('PQescapeByteaConn', dlsym(libpq, 'PQescapeByteaConn'), null, 'void *', 'long', 'unsigned long', 'size_t', 'long');
function PQescapeByteaConn(conn, from, from_length, to_length) {
  return call('PQescapeByteaConn', conn, from, from_length, to_length);
}

/**       
 * @function PQunescapeBytea 
 * 
 * @param    {Number}        strtext
 * @param    {Number}        retbuflen
 * 
 * @return   {Number}        
 */
define('PQunescapeBytea', dlsym(libpq, 'PQunescapeBytea'), null, 'void *', 'unsigned long', 'long');
function PQunescapeBytea(strtext, retbuflen) {
  return call('PQunescapeBytea', strtext, retbuflen);
}


/**       
 * @function PQescapeBytea   
 * 
 * @param    {Number}        from
 * @param    {Number}        from_length
 * @param    {Number}        to_length
 * 
 * @return   {Number}        
 */
define('PQescapeBytea', dlsym(libpq, 'PQescapeBytea'), null, 'void *', 'unsigned long', 'size_t', 'long');
function PQescapeBytea(from, from_length, to_length) {
  return call('PQescapeBytea', from, from_length, to_length);
}

/**       
 * @function PQprint         
 * 
 * @param    {Number}        fout
 * @param    {Number}        res
 * @param    {Number}        ps
 */
define('PQprint', dlsym(libpq, 'PQprint'), null, 'void', 'long', 'long', 'long');
function PQprint(fout, res, ps) {
  call('PQprint', fout, res, ps);
}

/**       
 * @function PQdisplayTuples 
 * 
 * @param    {Number}        res
 * @param    {Number}        fp
 * @param    {Number}        fillAlign
 * @param    {String}        fieldSep
 * @param    {Number}        printHeader
 * @param    {Number}        quiet
 */
define('PQdisplayTuples', dlsym(libpq, 'PQdisplayTuples'), null, 'void', 'long', 'long', 'int', 'char *', 'int', 'int');
function PQdisplayTuples(res, fp, fillAlign, fieldSep, printHeader, quiet) {
  call('PQdisplayTuples', res, fp, fillAlign, fieldSep, printHeader, quiet);
}

/**       
 * @function PQprintTuples   
 * 
 * @param    {Number}        res
 * @param    {Number}        fout
 * @param    {Number}        printAttName
 * @param    {Number}        terseOutput
 * @param    {Number}        width
 */
define('PQprintTuples', dlsym(libpq, 'PQprintTuples'), null, 'void', 'long', 'long', 'int', 'int', 'int');
function PQprintTuples(res, fout, printAttName, terseOutput, width) {
  call('PQprintTuples', res, fout, printAttName, terseOutput, width);
}

/**       
 * @function PQlibVersion    
 * 
 * @return   {Number}        
 */
define('PQlibVersion', dlsym(libpq, 'PQlibVersion'), null, 'int');
function PQlibVersion() {
  return call('PQlibVersion', );
}

/**       
 * @function PQmblen         
 * 
 * @param    {String}        s
 * @param    {Number}        encoding
 * 
 * @return   {Number}        
 */
define('PQmblen', dlsym(libpq, 'PQmblen'), null, 'int', 'char *', 'int');
function PQmblen(s, encoding) {
  return call('PQmblen', s, encoding);
}

/**       
 * @function PQdsplen        
 * 
 * @param    {String}        s
 * @param    {Number}        encoding
 * 
 * @return   {Number}        
 */
define('PQdsplen', dlsym(libpq, 'PQdsplen'), null, 'int', 'char *', 'int');
function PQdsplen(s, encoding) {
  return call('PQdsplen', s, encoding);
}

/**       
 * @function PQenv2encoding  
 * 
 * @return   {Number}        
 */
define('PQenv2encoding', dlsym(libpq, 'PQenv2encoding'), null, 'int');
function PQenv2encoding() {
  return call('PQenv2encoding', );
}

/**       
 * @function PQencryptPassword
 * 
 * @param    {String}        passwd
 * @param    {String}        user
 * 
 * @return   {String}        
 */
define('PQencryptPassword', dlsym(libpq, 'PQencryptPassword'), null, 'char *', 'char *', 'char *');
function PQencryptPassword(passwd, user) {
  return call('PQencryptPassword', passwd, user);
}

/**       
 * @function PQencryptPasswordConn
 * 
 * @param    {Number}        conn
 * @param    {String}        passwd
 * @param    {String}        user
 * @param    {String}        algorithm
 * 
 * @return   {String}        
 */
define('PQencryptPasswordConn', dlsym(libpq, 'PQencryptPasswordConn'), null, 'char *', 'long', 'char *', 'char *', 'char *');
function PQencryptPasswordConn(conn, passwd, user, algorithm) {
  return call('PQencryptPasswordConn', conn, passwd, user, algorithm);
}
