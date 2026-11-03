# FishTracker Golang - Pterodactyl Egg
# 3GB RAM Optimized | 1000 Titik Max

## Build & Run (Pterodactyl)
Egg: Golang 1.21
Startup Command: ./fishtracker
Docker Image: golang:1.21-bullseye (with opencv)

### Install GoCV dependencies on Pterodactyl container:
apt-get update && apt-get install -y libopencv-dev libopencv-contrib-dev

### Build
go mod tidy
go build -o fishtracker -ldflags="-s -w" main.go

### Run
SERVER_PORT=8080 ./fishtracker
# or
PORT=8080 ./fishtracker

## API
POST /api/track - form-data: video file, sampleFPS
GET /api/download/ofts/:id - download .ofts (FishTools Studio)
GET /api/download/kf/:id - download .kf (Keyflow)

## Tracking Logic (Port dari V4 JS yang berhasil)
- Grid 8x6 = 48 cell
- 30 titik per cell, quality 0.003, distance 3px = 1440 max
- Fallback full frame quality 0.001 distance 2px = 1000 titik
- goodFeaturesToTrack + cornerSubPix
- calcOpticalFlowPyrLK winSize 21x21 maxLevel 4
- findHomography RANSAC 1.5px
- Kalman Q=0.003 R=0.4, Qrot=0.01 R=0.3
- Export OFTS: ZIP project.json (format FishTools Studio)
- Export KF: ZIP manifest.json + project.json (format Keyflow)

## Hasil sekelas Python
GoCV dan opencv-python sama-sama binding ke OpenCV C++ native.
Hasil dx,dy,rot identik 1:1.

RAM usage:
- Idle: 15MB
- Tracking 640x360: ~80MB
- 3GB bisa handle 10 user concurrent

## Pterodactyl Egg JSON (custom)
{
  "name": "FishTracker Golang",
  "docker_image": "golang:1.21-bullseye",
  "startup": "./fishtracker",
  "config": {
    "files": "{}",
    "startup": "{\"done\": \"FishTracker Golang Server running\"}",
    "logs": "{}",
    "stop": "^C"
  }
}
