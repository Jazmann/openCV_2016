ECHO "Provide super user password please."
sudo ECHO "Hello superuser!"
find ./opencv -iname "CMakeCache.txt" -exec rm -rf {} \;
find ./opencv -iname "cmake_install.cmake" -exec rm -rf {} \;
find ./opencv -iname "CMakeFiles" -exec rm -rf {} \;
find ./opencv -iname "*.orig" -exec rm -rf {} \;
sudo rm -rf ./opencv/build

sudo rm -rf osx;
sudo rm -rf unix;

sudo rm -rf install/lib/pkgconfig
sudo rm -rf install/lib
sudo rm -rf install/bin
sudo rm -rf install/include

sudo rm  log_osx.txt;

