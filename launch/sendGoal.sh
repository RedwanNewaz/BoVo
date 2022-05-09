#!/usr/bin/env bash

publisher()
{
  echo "[sending command] $1 $2"
  rostopic pub /roomba21/goal geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: "map"}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {w: 1.0}}}"
}

publisher $1 $2