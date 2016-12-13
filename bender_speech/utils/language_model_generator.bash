#!/bin/bash

# Desde un .txt genera un 'language model' para ser utilizado en
# el reconocimiento de voces de pocketsphinx


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

# required tools installed
_bender_check_installed_and_exit text2wfreq
_bender_check_installed_and_exit text2idngram
_bender_check_installed_and_exit idngram2lm
_bender_check_installed_and_exit sphinx_lm_convert


# input txt exists
_txt_file="$1"
if [ -z "$_txt_file" ]; then
    echo "This script requires 1 argument: <file.txt>"
    unset _txt_file
    exit 1
fi
if [ ! -f "$_txt_file" ]; then
    echo "File NOT FOUND: $_txt_file"
    unset _txt_file
    exit 1
fi

# input file is a txt file.
if ! _bender_check_file_ext "$_txt_file" "txt"; then
    unset _txt_file
    exit 1
fi


##
## ====================================================
## WORK
## ====================================================
##

# prepare
_file_path=$(_bender_get_filepath "$_txt_file")
_file_base=$(_bender_get_file_noextname "$_txt_file")
_fullname_noext="$_file_path/$_file_base"
unset _file_path
unset _file_base



# .txt     -> .vocab   : vocabulary list generation
text2wfreq < "$_fullname_noext".txt | wfreq2vocab > "$_fullname_noext".vocab

# .vocab   -> .idngram : WTF?
text2idngram -vocab "$_fullname_noext".vocab -idngram "$_fullname_noext".idngram < "$_fullname_noext".txt

# .idngram -> .arpa    : ??
idngram2lm -vocab_type 0 -idngram "$_fullname_noext".idngram -vocab "$_fullname_noext".vocab -arpa "$_fullname_noext".arpa

# .arpa    -> .dmp     : ??
sphinx_lm_convert -i "$_fullname_noext".arpa -o "$_fullname_noext".dmp

# .dmp     -> .lm      : ??
sphinx_lm_convert -i "$_fullname_noext".dmp -ifmt dmp -o "$_fullname_noext".lm -ofmt arpa

# ??
#pocketsphinx_continuous -lm 1.lm -dict 1.dic

##
## ====================================================
## Clean up
## ====================================================
##

rm -f "$_fullname_noext".vocab
rm -f "$_fullname_noext".idngram
rm -f "$_fullname_noext".arpa
rm -f "$_fullname_noext".dmp

unset _fullname_noext

echo -e "Done :)"