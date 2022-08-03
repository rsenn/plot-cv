import { PQconnectStart, PQconnectStartParams, PQconnectdb, PQconnectdbParams, PQsetdbLogin, PQfinish, PQconndefaults, PQconninfoParse, PQconninfo, PQconninfoFree, PQresetStart, PQreset, PQgetCancel, PQfreeCancel, PQcancel, PQrequestCancel, PQdb, PQuser, PQpass, PQhost, PQhostaddr, PQport, PQtty, PQoptions, PQparameterStatus, PQprotocolVersion, PQserverVersion, PQerrorMessage, PQsocket, PQbackendPID, PQconnectionNeedsPassword, PQconnectionUsedPassword, PQclientEncoding, PQsetClientEncoding, PQsslInUse, PQsslStruct, PQsslAttribute, PQsslAttributeNames, PQgetssl, PQinitSSL, PQinitOpenSSL, PQgssEncInUse, PQgetgssctx, PQtrace, PQuntrace, PQexec, PQexecParams, PQprepare, PQexecPrepared, PQsendQuery, PQsendQueryParams, PQsendPrepare, PQsendQueryPrepared, PQsetSingleRowMode, PQgetResult, PQisBusy, PQconsumeInput, PQnotifies, PQputCopyData, PQputCopyEnd, PQgetCopyData, PQgetline, PQputline, PQgetlineAsync, PQputnbytes, PQendcopy, PQsetnonblocking, PQisnonblocking, PQisthreadsafe, PQflush, PQfn, PQresStatus, PQresultErrorMessage, PQresultVerboseErrorMessage, PQresultErrorField, PQntuples, PQnfields, PQbinaryTuples, PQfname, PQfnumber, PQftablecol, PQfformat, PQfsize, PQfmod, PQcmdStatus, PQoidStatus, PQcmdTuples, PQgetvalue, PQgetlength, PQgetisnull, PQnparams, PQdescribePrepared, PQdescribePortal, PQsendDescribePrepared, PQsendDescribePortal, PQclear, PQfreemem, PQmakeEmptyPGresult, PQcopyResult, PQsetResultAttrs, PQresultAlloc, PQsetvalue, PQescapeLiteral, PQescapeIdentifier, PQescapeByteaConn, PQunescapeBytea, PQescapeBytea, PQprint, PQdisplayTuples, PQprintTuples, PQlibVersion, PQmblen, PQdsplen, PQenv2encoding, PQencryptPassword, PQencryptPasswordConn } from './psql.js';

function main(...args) {
  let pq = PQconnectStart(
    'User ID=roman;Password=r4eHuJ;Host=localhost;Port=5432;Database=roman;Pooling=true;Min Pool Size=0;Max Pool Size=100;Connection Lifetime=0;'
  );
  console.log('pq', pq);

  let result = PQexec(pq, 'SELECT * FROM test');
  console.log('result', result);
  result = PQgetResult(pq);
  console.log('result', result);

  let error = PQerrorMessage(pq);
  console.log('error', error);
}

main(...scriptArgs.slice(1));
