#!/bin/sh
nl="
"
BASE=${1-localhost}
HOST=${2-localhost}
DEFCONFIG="[dn]${nl}CN=${HOST}${nl}[req]${nl}distinguished_name = dn${nl}[EXT]${nl}subjectAltName=DNS:${HOST}${nl}keyUsage=digitalSignature${nl}extendedKeyUsage=serverAuth"
CONFIGFILE=`mktemp`
trap 'rm  -vf "$CONFIGFILE"' EXIT
echo "$DEFCONFIG" >$CONFIGFILE
openssl req \
  -x509 \
  -out "$BASE.crt" \
  -keyout "$BASE.key" \
  -newkey rsa:2048 \
  -nodes \
  -sha256 \
  -subj "/CN=$HOST" \
  -extensions EXT \
  -config $CONFIGFILE "$@"

