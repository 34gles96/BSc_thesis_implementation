#!/bin/bash

mkdir -p /home/pist/.docker-x11

cp ~/.Xauthority /home/pist/.docker-x11/xauth
chmod 644 /home/pist/.docker-x11/xauth

echo "Docker X11 authentication refreshed."