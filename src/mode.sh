#!/bin/bash
read -p "input file name " file_name
echo 'changing the mode to 755 of these file' $file_name
chmod 755 $file_name
echo
read -p "want to MAKE" -n 1 -r option
if [[ $option =~ ^[Yy]$ ]]
then
  cd ../bin
  make
  echo "MAKE"
else
  echo "NO MAKE"
fi

