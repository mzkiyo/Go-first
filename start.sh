#!/bin/bash
# FishTracker Golang - Pterodactyl Startup Fix (no startup command edit needed)
# Taruh file ini sebagai start.sh di root server, chmod +x, terus set EXECUTABLE=start.sh di egg config kalau bisa, atau ganti main.go jadi ini

export GOPATH=/home/container/go
export GOCACHE=/home/container/.cache/go-build
export GOMODCACHE=/home/container/go/pkg/mod
export GO111MODULE=on
mkdir -p $GOPATH $GOCACHE $GOMODCACHE

# fix permission denied path/filepath - Go butuh cache writable
echo "[*] Setting up Go env: GOPATH=$GOPATH"

if [ ! -f ./fishtracker ]; then
  echo "[*] Binary not found, building..."
  go mod tidy
  go build -o fishtracker -ldflags="-s -w" main.go
  chmod +x fishtracker
  echo "[*] Build done"
fi

# pterodactyl port
if [ -z "$SERVER_PORT" ]; then
  if [ -n "$PORT" ]; then
    SERVER_PORT=$PORT
  else
    SERVER_PORT=8080
  fi
fi

export SERVER_PORT
echo "[*] Starting FishTracker Golang on :$SERVER_PORT (3GB optimized, 1000 titik)"
./fishtracker
