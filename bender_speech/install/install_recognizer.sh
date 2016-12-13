#!/bin/bash
#
# prefer running: 
# > cdb bender_speech
# > bash install/install.sh
#
# OBS: Si se encuentran problemas de compilación revisar paquetes de PERL faltantes

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
install_space="$BENDER_WS"/install/soft/hri/speech
mkdir -p "$install_space" && cd "$install_space"


# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #


## Download and install dependences

# speech recognizer (pocketsphinxs)
sudo apt-get install gstreamer0.10-plugins-base
sudo apt-get install gstreamer0.10-pocketsphinx
sudo apt-get install bison
sudo apt-get install sphinxbase-utils

# other tools
sudo apt-get install unzip
sudo apt-get install autoconf automake


# Installing the speech recognition system
# ==========================================

pocketsphinx_folder="$install_space"/files/pocketsphinx/
if [ -d "$pocketsphinx_folder" ]; then

	install_token="$install_space"/INSTALLED_POCKETSPHINX
	if [ ! -e "$install_token" ]; then
		# Untar pocketsphinx
		cd "$pocketsphinx_folder"
		tar xvf pocketsphinx-0.8.tar.gz
		tar xvf sphinxbase-0.8.tar.gz

		# Configure sphinxbase
		cd "$pocketsphinx_folder"/sphinxbase-0.8/
		./autogen.sh
		./configure
		make
		sudo make install

		# Configure pocketphinx
		cd "$pocketsphinx_folder"/pocketsphinx-0.8
		./configure
		make
		sudo make install

		# Delete unnecessary folders
		cd "$pocketsphinx_folder"
		sudo rm -rf sphinxbase-0.8/
		sudo rm -rf pocketsphinx-0.8/

		# mark as installed
		touch "$install_token"
	else
		echo " - pocketsphinx is already installed"
	fi
	
else
	echo " - pocketphinx folder not found!: $pocketsphinx_folder"
	echo "   please run install.sh"
	exit 1
fi

# ------------------------------------------

# LOGIOS & CMUCLMTK tools (for offline compiling of dictionaries

# requerido
sudo apt-get install subversion

# download source
# TODO: fix this. when trying to install this from command line (using this install.sh script)
# some svn refs are not properly downloaded.
cd "$install_space"
install_token="$install_space"/INSTALLED_CMUSPHINX
if [ ! -d "$install_space"/cmusphinx-code ] || [ ! -e "$install_token" ]; then
	rm -rf "$install_space"/cmusphinx-code
	svn checkout --non-recursive http://svn.code.sf.net/p/cmusphinx/code/trunk cmusphinx-code
	svn update --set-depth infinity cmusphinx-code/logios
	svn update --set-depth infinity cmusphinx-code/cmuclmtk

	# make
	cd "$install_space"/cmusphinx-code/cmuclmtk/
	./autogen.sh
	./configure
	sudo make install

	# mark as installed
	touch "$install_token"
else
	echo " - cmusphinx-code is already installed"
fi

echo ""
echo "Done. ;)"
echo ""
# :)
