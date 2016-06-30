ECHO "Provide super user password please."
sudo ECHO "Hello superuser!"
ECHO "Cleaning";
./make_osx_clean.sh;
./make_ios_clean.sh;
ECHO "Building OSX";
./make_osx.sh
ECHO "Building iOS";
./make_iOS.sh
ECHO "Copying over to install local directory";
./makeinstall.sh
ECHO "Installing";
./install.sh
ECHO "Running tests";
./run_test.sh