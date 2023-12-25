# LieGroupRoboticsControl
```bash
   sudo apt-get install build-essential cmake
   sudo apt-get install libeigen3-dev liburdfdom-dev
```

# robotpkg install
http://robotpkg.openrobots.org/debian.html
```bash
sudo mkdir -p /etc/apt/keyrings

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc |
   sudo tee /etc/apt/keyrings/robotpkg.asc

sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub focal robotpkg
EOF
```
# pinocchio install
https://stack-of-tasks.github.io/pinocchio/download.html
```bash
   sudo apt install -qqy lsb-release curl
   sudo mkdir -p /etc/apt/keyrings
   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
       | sudo tee /etc/apt/keyrings/robotpkg.asc
   echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
       | sudo tee /etc/apt/sources.list.d/robotpkg.list
   sudo apt update
   sudo apt install -qqy robotpkg-py3*-pinocchio
   echo 'export PATH=/opt/openrobots/bin:$PATH
         export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
         export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
         export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
         export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc
```
# ruckig install
```bash
   git clone https://github.com/pantor/ruckig.git
   cd ruckig
   export RUCKIG_HOME=$PWD
   mkdir build
   cd build
   cmake ..
   make -j$(nproc)
   sudo make install
   cd /opt
   sudo  ln -s $RUCKIG_HOME .
```
# install
```bash
  git clone https://github.com/MinchangSung0223/LieGroupRoboticsControl.git
  cd LieGroupRoboticsControl
  mkdir build
  cd build
  cmake ..
  make -j$(nproc)
  sudo make install
```

