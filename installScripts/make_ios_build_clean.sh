find ./opencv -iname "CMakeCache.txt" -exec rm -rf {} \;
find ./opencv -iname "cmake_install.cmake" -exec rm -rf {} \;
find ./opencv -iname "CMakeFiles" -exec rm -rf {} \;
find ./opencv -iname "*.orig" -exec rm -rf {} \;
sudo rm -rf ./opencv/build

ECHO "Removing opencv framework for iOS";
sudo rm -rf ios;
sudo rm -rf install/Library/Frameworks;
sudo rm -rf install/Library;

sudo rm  log_ios.txt;

