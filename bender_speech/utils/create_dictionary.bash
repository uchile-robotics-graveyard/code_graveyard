#!/bin/bash
# Compila diccionarios para reconocimiento de voz
#
# Uso:
# bash create_dictionary.bash mi_diccionario.jsgf
#
# Lo que compilará el <mi_diccionario.jsgf> provisto
# El path al .jsgf debe ser entreado en relación a la carpeta bender_speech
#
# OBS 1: Es necesario instalar LOGIOS y CMUCLMTK (ver install/install.sh)
# OBS 2: Es necesario que exista la variable de entorno: "BENDER_SPEECH_LOGIOS_DIR" (ver install/setup.sh)
# OBS 3: Se pueden obtener diccionarios desde internet
#
# Para obtener el Language Model hay que generar un .txt con las palabras o frases en lineas y subir en
# http://www.speech.cs.cmu.edu/tools/lmtool-new.html
#
# TODO: mejorar los resultados del LMTOOL offline


##
## ====================================================
## "Includes"
## ====================================================
##
source "$BENDER_SYSTEM/bash/functions.sh"


##
## ====================================================
## checks
## ====================================================
##

# sphinx_jsgf2fsg is installed
_bender_check_installed_and_exit sphinx_jsgf2fsg

# perl is installed
_bender_check_installed_and_exit perl

# BENDER_SPEECH_LOGIOS_DIR is set
if ! _bender_check_var_isset "BENDER_SPEECH_LOGIOS_DIR"; then
    echo "Sorry, but environment variable BENDER_SPEECH_LOGIOS_DIR is not set."
    exit 1
fi

# logios tools exists
_logios_tool="$BENDER_SPEECH_LOGIOS_DIR"/Tools/MakeDict/make_pronunciation.pl
if [ ! -f "$_logios_tool" ]; then
    echo "Logios tool NOT FOUND!:"
    echo " - It should be located at: $_logios_tool"
    echo " - This path depends on BENDER_SPEECH_LOGIOS_DIR variable,"
    echo "   which is set to: - $BENDER_SPEECH_LOGIOS_DIR -"
    unset _logios_tool
    exit 1
fi

# input jsgf exists
_jsgf_file="$1"
if [ -z "$_jsgf_file" ]; then
    echo "This script requires 1 argument: <file.jsgf>"
    unset _logios_tool
    unset _jsgf_file
    exit 1
fi
if [ ! -f "$_jsgf_file" ]; then
    echo "File NOT FOUND: $_jsgf_file"
    unset _logios_tool
    unset _jsgf_file
    exit 1
fi

# input file is a jsgf file.
if ! _bender_check_file_ext "$_jsgf_file" "jsgf"; then
    unset _logios_tool
    unset _jsgf_file
    exit 1
fi


##
## ====================================================
## WORK
## ====================================================
##

# prepare
_file_path=$(_bender_get_filepath "$_jsgf_file")
_file_base=$(_bender_get_file_noextname "$_jsgf_file")
_fullname_noext="$_file_path/$_file_base"
_utils_path="$(rospack find bender_speech)"/utils
unset _file_path
unset _file_base

# jsgf  -->   fsg
sphinx_jsgf2fsg -jsgf "$_fullname_noext".jsgf -fsg "$_fullname_noext".fsg

# fsg   -->   words
perl "$_utils_path"/fsg2wlist.pl<"$_fullname_noext".fsg> "$_fullname_noext".words
#text2wfreq < "$_fullname_noext".txt | wfreq2vocab > "$_fullname_noext".vocab
#sed 's:#.*$::g' "$_fullname_noext".vocab > "$_fullname_noext".words

# delete repeated words
perl -ne 'print unless $seen{$_}++' "$_fullname_noext".words > "$_fullname_noext".word

# convierte las palabras a minusculas
perl -pe '$_= lc($_)' "$_fullname_noext".word > "$_fullname_noext".words

# words -->   dic
perl "$_logios_tool" -tools "$BENDER_SPEECH_LOGIOS_DIR"/Tools/ -dictdir . -words "$_fullname_noext".words -dict "$_fullname_noext".dic


##
## ====================================================
## Clean up
## ====================================================
##

rm -f "$_fullname_noext".word
rm -f logios.log
rm -f pronunciation.log

unset _fullname_noext
unset _utils_path

echo -e "Done :)"
