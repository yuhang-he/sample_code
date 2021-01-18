# SpotQtGUI

SpotQtGUI is a graphical user interface for controlling and monitoring Boston Dynamic's Spot built using Qt's
widget toolkit.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

The GUI is built using Qt for Python. You can install Pyside2 via PyPi, using Qt-servers or by building the source package.
The full instruction for install Qt for Python can be found on Qt's [website](https://wiki.qt.io/Qt_for_Python/GettingStarted).

In additon to Qt for Python, Boston Dynamic's APIs are required to communicate with Spot. Follow the instruction from
Boston Dynamics to install their APIs.

### Installing

Before running the code, the following Python packages are required. You can install them using pip.

* Pillow
* PySide2
* bosdyn-api
* bosdyn-client
* bosdyn-core
* grpcio
* imutils
* numpy
* opencv-python
* protobuf
* requests
* tornado

For example
```
pip install Pyside2
```

#### Faro scanner control

To properly use Faro scanner control module, the FaroScanServer codes must be build using Visual Studio separately. After building a release version of the code with FaroScanServer.exe executable, the code must be changed to include the path to the executable.

Within SpotQtGUI/src/faro_scan.py, line 17 - 19 must be modified to include the path
```
subprocess.check_call('FaroScanServer.exe', shell=True,
                          cwd='C:\\Users\\<path to executable>',
                          close_fds=True)
```

#### Before execution

Before executing the GUI, the computer must be on the same network as Spot. The following are required to succesfully
communicate with Spot.

* IP Address of Spot on the network
* User name and password to access Spot
* A working token from BD and full path to the token

To start the Qt GUI, navigate to the folder containing spot_gui.py and execute in terminal

```
python spot_gui.py --user user --password passwword --app-token C:\Users\<path to token> 192.168.80.3 <hostname or IP address>
```
python spot_gui.py --user user --password client_user --app-token C:\Users\mjezyk\.bosdyn\tesla.token 192.168.80.3


### Usage

Steps for controlling Spot
* Connect: Acuquiring robot ID, lease, and setup Estop
* Control: Start async robot state update
* Estop: Toggle E-stop
* Power: Powering on motor
* Keyboard Control: Open a separate window to receive keyboard commands (keep it focused to receive commands)
* Shutdown: Stop time sync, stop Estop keepalive, return lease
* Safe Power Off: Start safe power off command (sit then power off motor)
* Set Stand Height: Change Spot stand height. Height of 0.0 represent nomial height. Max: 0.12, Min: -0.17.
                                                        Start with 0.0 before increasing or decreasing height.

### Test

If you do not have access to Spot, you can run test_interface.py in test folder to test if GUI can be run.

Change directory to test
```
python test_interface.py
```
