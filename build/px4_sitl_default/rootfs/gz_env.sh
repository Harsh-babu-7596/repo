#!/usr/bin/env bash

export PX4_GZ_MODELS=/home/harsh/finale/Tools/simulation/gz/models
export PX4_GZ_WORLDS=/home/harsh/finale/Tools/simulation/gz/worlds

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS
