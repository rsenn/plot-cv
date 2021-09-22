#!/bin/sh

trap 'rm -f "$TMP"' EXIT
TMP=`mktemp`

HOST=${1:-localhost}

printf "[dn]\nCN=$HOST\n[req]\ndistinguished_name = dn\n[EXT]\nsubjectAltName=DNS:$HOST\nkeyUsage=digitalSignature\nextendedKeyUsage=serverAuth" >$TMP
openssl req -x509 -out $HOST.crt -keyout $HOST.key \
  -newkey rsa:2048 -nodes -sha256 \
  -subj '/CN=$HOST' -extensions EXT -config "$TMP"
