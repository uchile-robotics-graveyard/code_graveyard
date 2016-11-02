#!/bin/sh

## Configurar el entorno con las variables necesarias para el package.
# utilizados para ??
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

# directorio con instalación de LOGIOS (compilación de diccionarios)
export BENDER_SPEECH_LOGIOS_DIR="$BENDER_WS"/install/soft/hri/speech/cmusphinx-code/logios
