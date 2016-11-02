#!/bin/bash

set -o errexit


##
## ====================================================
## "Includes"
## ====================================================
##
source "$BENDER_SYSTEM/bash/functions.sh"
source "$(rospack find bender_speech)/utils/funciones.bash"


##
## ====================================================
## checks
## ====================================================
##

# arecord is installed
_bender_check_installed_and_exit arecord

# gst-launch-0.10 is installed
_bender_check_installed_and_exit gst-launch-0.10


##
## ====================================================
## LOOKUP FOR VALID CARDs
## ====================================================
##

## Modo automatico
if [ "$1" = "--auto" ] || [ "$1" = "-a"  ]
then
    export auto_mode=1
fi

## Buscar fuentes de entrada
declare -a source_ids

while read line
do
    echo "line: $line"

    card_number=$(echo "$line"   \
        | sed 's/^tarjeta //g'   \
        | sed 's/^card //g'      \
        | sed 's/:.*//g'         \
    )

    device_number=$(echo "$line"      \
        | sed 's/.* dispositivo //g'  \
        | sed 's/.* device //g'       \
        | sed 's/:.*//g'              \
    )

    name=$(echo "$line"                       \
        | sed 's/.* dispositivo [0-9]\+: //g' \
        | sed 's/.* device [0-9]\+: //g'      \
    )

    alsaname="hw:$card_number,$device_number"

    source_ids+=("$alsaname")
    info_ "Source #${#source_ids[@]} is named '$name' with alsa name: ($alsaname)"

done < <(arecord --list-devices | grep -E '^card|tarjeta [0-9]+: USB')

unset card_number
unset device
unset name
unset alsaname

if [ ${#source_ids[@]} -eq 0 ];
then
    info_bold_ "[Microphone Configuration]: Ups. No configurable USB sound devices were found. "
    exit 0
fi


##
## ====================================================
## DEVICE CONFIGURATION
## ====================================================
##

# source number
info_bold_ "\nSeleccione una de las siguientes fuentes de entrada ..."
prompt_options "ID" "$(( ${#source_ids[@]} - 1 ))" "${!source_ids[@]}"
audio_source=${source_ids[$REPLY]}

# ganancia
info_bold_ "\nSeleccione ganancia de entrada entre -6.0 y 20.0 [dB]"
prompt_range "Ganancia" "10.0" "-6.0" "20.0"
audio_gain=$REPLY

# lowest frequency
info_bold_ "\nSeleccione frecuencia de corte inferior entre 50 y 10000 [Hz]"
prompt_range "Frecuencia" "500" "50" "10000"
audio_lowcut=$REPLY

# highest freq.
info_bold_ "\nSeleccione frecuencia de corte superior entre 50 y 10000 [Hz]"
prompt_range "Frecuencia" "3000" "50" "10000"
audio_highcut=$REPLY

# compression level
info_bold_ "\nSeleccione el nivel de compresion entre 0.1 y 8"
prompt_range "Ratio" "0.25" "0.1" "8"
audio_ratio=$REPLY

# threshold
info_bold_ "\nSeleccione el nivel de threshold entre 0 y 1"
prompt_range "Threshold" "0.5" "0" "1"
audio_threshold=$REPLY



##
## ====================================================
## DEVICE CONFIGURATION
## ====================================================
##

info_bold_ "\nIniciando GStreamer (Termine el proceso con Ctrl+C) ..."
gst-launch-0.10 alsasrc device="$audio_source"                                          \
    ! audioconvert                                                                      \
    ! rgvolume pre-amp="$audio_gain" headroom="$audio_gain"                             \
    ! audiowsincband lower-frequency="$audio_lowcut" upper-frequency="$audio_highcut"   \
    ! audiodynamic ratio="$audio_ratio" threshold="$audio_threshold"                    \
    ! rglimiter                                                                         \
    ! pulsesink


unset audio_source
unset audio_gain
unset audio_lowcut
unset audio_highcut
unset audio_ratio
unset audio_threshold
