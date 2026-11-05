#!/bin/sh
# Build the static site and install it (and the nginx config) on this host.
#   tools/deploy-site.sh            build + rsync to /var/www/transistorisiert.ch
#   tools/deploy-site.sh --nginx    also install nginx/{ssl,default} and reload
set -e
cd "$(dirname "$0")/.."
DEST=${DEST:-/var/www/transistorisiert.ch}

qjsm directory.js collect           # pages + active nginx locations -> index.json
qjsm tools/build-site.mjs build/site  # stage pages (previews need them)
qjsm directory.js previews          # headless-chrome screenshots, only missing ones (--force: all)
qjsm directory.js collect           # link the previews
qjsm tools/build-site.mjs build/site  # final tree incl. previews/
sudo install -d -o "$USER" "$DEST"
rsync -a --delete build/site/ "$DEST"/

if [ "$1" = --nginx ]; then
  [ -e /etc/nginx/www.crt ] || { echo "no /etc/nginx/www.crt: run tools/www-cert.sh first" >&2; exit 1; }
  sudo install -m 644 nginx/ssl /etc/nginx/sites-available/ssl
  sudo install -m 644 nginx/default /etc/nginx/sites-available/default
  sudo nginx -t && sudo systemctl reload nginx
fi
