---
hide_title: true
sidebar_label: Qt
---

# Qt

## Installation on Ubuntu (18.04/16.04)

Download Qt 5.12.11, used the same logic to download more Qt versions if needed.
```bash
sudo apt-get install qt5-default # Default Qt5
cd Downloads/ # Download run file into Downloads
wget http://download.qt.io/official_releases/qt/5.12/5.12.11/qt-opensource-linux-x64-5.12.11.run # Download run file into Downloads
chmod +x qt-opensource-linux-x64-5.12.11.run # Give permission to execute
sudo ./qt-opensource-linux-x64-5.12.11.run # Execute file and download packages to /home/$USER/ preferably
```

```bash
sudo update-alternatives --install /usr/bin/qmake qmake /usr/lib/x86_64-linux-gnu/qt5/bin/qmake 10 # Default Qt to use
sudo update-alternatives --install /usr/bin/qmake qmake /home/nvidia/Qt5.12.11/5.12.11/gcc_64/bin/qmake 20 # Add more Qt versions to use
sudo update-alternatives --config qmake # Choose which Qt version to use
```

```bash
qmake -v # Check to see whether qmake is the right version
```

why is this important, for example `rpi-imager` requires Qt > 5.12.0 hence default ubuntu is 5.9.5, also `rpi-imager` can only be downloaded with `snap` and not as `debian` (`apt-get`) package