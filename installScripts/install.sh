ECHO "Provide super user password please."
sudo ECHO "Hello superuser!"
./make_osx_install_clean.sh;
./make_ios_install_clean.sh;


sudo cp -R ./install/include /opt/local
sudo cp -R ./install/lib/*.a /opt/local/lib
if [ -d "/opt/local/lib/pkgconfig/" ]; then 
  echo "Directory /opt/local/lib/pkgconfig/ exists." ;
  sudo cp ./install/lib/pkgconfig/opencv.pc /opt/local/lib/pkgconfig/;
else 
  echo "Directory /opt/local/lib/pkgconfig/ does not exists."
fi
sudo find ./install/lib/libopencv*.dylib -execdir ./change_install_name.sh {} /opt/local/lib/ \;
sudo cp -R ./install/bin /opt/local

sudo cp -R ./install/include /usr/local
sudo cp -R ./install/lib/*.a /usr/local/lib
if [ -d "/usr/local/lib/pkgconfig/" ]; then
  echo "Directory /usr/local/lib/pkgconfig/ exists." ;
  sudo cp ./install/lib/pkgconfig/opencv.pc /usr/local/lib/pkgconfig/
else
  echo "Directory /usr/local/lib/pkgconfig/ does not exists."
fi
sudo find ./install/lib/libopencv*.dylib -execdir ./change_install_name.sh {} /usr/local/lib/ \;
sudo cp -R ./install/bin /usr/local


./make_ios_install_clean.sh;
sudo cp -R ./install/Library/Frameworks/opencv2.framework /Library/Frameworks


ECHO "opencv install files copied to /opt/local/";

