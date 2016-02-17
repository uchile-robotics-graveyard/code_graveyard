#!/bin/sh

export BIN=.
export DEVICE=plughw:1,0
export CONF=conf/kinect_tf.zip
export CONF_MC=conf/microcone_tf.zip

#for deg in 000 090 180 270; do
  #echo Display sounds from ${deg} degrees.
  #${BIN}/demoOffline8ch.n ${DATA}/f101_${deg}.wav ${CONF}/music.dat Localization_${deg}.txt
#done


echo ${BIN}/microcone_streamer.n ${DEVICE} ${CONF_MC} loc.txt \> log.txt
${BIN}/microcone_streamer.n ${DEVICE} ${CONF_MC} loc.txt > log.txt
