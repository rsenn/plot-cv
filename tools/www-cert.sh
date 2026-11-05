#!/bin/sh
# Get a ZeroSSL certificate for www.transistorisiert.ch (and transistorisiert.ch once its DNS
# points at this machine) and install it for nginx.
# nginx.crt only covers the apex name, hence the browser warning on www.
# CA=letsencrypt tools/www-cert.sh uses Let's Encrypt (validates at once; ZeroSSL can queue for hours).
# Needs: port 80 of this machine reachable under every requested name, and the acme-challenge
# location of the live nginx config (webroot /var/www/html).
set -e
ACME="$HOME/.acme.sh/acme.sh"
WWW=www.transistorisiert.ch
APEX=transistorisiert.ch

here=$(dig +short A "$WWW" @1.1.1.1 | tail -1)   # public resolver: the local one may cache an old record
domains="-d $WWW"

if [ "$(dig +short A "$APEX" @1.1.1.1 | tail -1)" = "$here" ]; then
  domains="$domains -d $APEX"
else
  echo "note: $APEX does not resolve to $here (this machine); issuing for $WWW only." >&2
  echo "      Point the apex at this machine, then run this script again." >&2
fi

"$ACME" --issue --server "${CA:-zerossl}" --ecc $domains -w /var/www/html || [ $? = 2 ]   # 2 = still valid, nothing to do
"$ACME" --install-cert --ecc -d "$WWW" \
  --fullchain-file "$HOME/.acme.sh/www.crt" --key-file "$HOME/.acme.sh/www.key" \
  --reloadcmd "sudo systemctl reload nginx"
sudo ln -sf "$HOME/.acme.sh/www.crt" /etc/nginx/www.crt
sudo ln -sf "$HOME/.acme.sh/www.key" /etc/nginx/www.key
