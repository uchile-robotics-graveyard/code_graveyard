#!bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

source $BENDER_CONFIG/setup.bash ;

# Useful Variables
pkg_name="bender_arm";
install_path=$(rospack find $pkg_name)/install;
install_files=$install_path/files;

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

roscd;

# Dynamixel motor
sudo apt-get install ros-indigo-dynamixel-motor ;

# ros-control
sudo apt-get install ros-indigo-ros-control ;


# - - - - - - - Install Rules - - - - -

sudo cp -f $install_files/right_arm.rules /etc/udev/rules.d/right_arm.rules;

sudo cp -f $install_files/left_arm.rules /etc/udev/rules.d/left_arm.rules;

sudo cp -f $install_files/base_arm.rules /etc/udev/rules.d/base_arm.rules;

# Permisos para lectura de puertos
var_group=$(groups $USER | grep -o -w -c dialout);
if [ $var_group = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\nPort permissions\n";
  sudo usermod -a -G dialout $USER;

  echo "The computer must be restarted in order to complete the installation";
  echo -e "... Do you want to reboot the pc now?\n";
  echo "(y)es (n)o:";
  read var_reboot;

  if [ $var_reboot = "y" ]; then

    echo -e "\nDone.\n ;)\n";
    sudo shutdown -r now;

  else
    echo "... Remember to reboot the computer!!!...";
    echo -e "\nDone.\n ;)\n";
  fi 
fi

# R E L L E N A R !!!!!!



echo -e "\nDone.\n ;)\n";
# :)
