#!/bin/bash
# Protect the web panel (and with it the manager API and MQTT WebSocket) with a user name and password.
#   tools/set_password.sh <data dir> <user>        (asks for the password)
#   tools/set_password.sh <data dir> --off         (remove the protection)
# Then restart the web service: docker compose -f docker-compose.demo.yml restart web
set -eu
DATA=${1:?data directory}
mkdir -p "$DATA/auth"
if [ "${2:-}" = "--off" ]; then
  rm -f "$DATA/auth/auth.conf" "$DATA/auth/htpasswd"
  echo "password protection removed"
  exit 0
fi
USER=${2:?user name}
read -r -s -p "password for $USER: " PW
echo
printf '%s:%s\n' "$USER" "$(openssl passwd -apr1 "$PW")" > "$DATA/auth/htpasswd"
cat > "$DATA/auth/auth.conf" <<'EOF'
auth_basic "Junction Perception";
auth_basic_user_file /etc/nginx/auth/htpasswd;
EOF
echo "password set for $USER"
