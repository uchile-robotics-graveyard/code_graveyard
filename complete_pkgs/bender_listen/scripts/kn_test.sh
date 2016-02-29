#!/bin/sh

export PACKAGE_PATH=$BENDER_WORKSPACE/bender_listen
export DEVICE=plughw:1,0
export CONF=$PACKAGE_PATH/conf/kinect_tf.zip
export CONF_MC=$PACKAGE_PATH/conf/microcone_tf.zip
export NET=$PACKAGE_PATH/networks
export BIN=.

#for deg in 000 090 180 270; do
  #echo Display sounds from ${deg} degrees.
  #${BIN}/demoOffline8ch.n ${DATA}/f101_${deg}.wav ${CONF}/music.dat Localization_${deg}.txt
#done

cd $NET

#echo "Online mode"
echo $BIN/kinect_streamer.n ${DEVICE} ${CONF} loc.txt \> log.txt
$BIN/kinect_streamer.n ${DEVICE} ${CONF} loc.txt > log.txt

