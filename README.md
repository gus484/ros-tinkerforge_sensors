### Tinkerforge Sensors

Mit diesem Paket können Tinkerforge Sensoren im ROS genutzt werden. Auch Sensoren des gleichen Typs werden erkannt. Durch Nutzung des "robusten Ansatzes" ist die Angabe von UIDs nicht notwendig. Die Sensorwerte werden in entsprechende ROS-Messages über automatisch generierte Topics verbreitet. Unter Verwendung eines Launch-Files können Topic und andere Sensorparameter per UID manuell festgelegt werden.

This package allows you to use TinkerForge sensors with ROS. Sensors of same type will be supported. Package used the "Rugged Approach", so it's not necessary to configure the sensor UIDs. Sensor values will be published over automatic generated topics with typical ROS message types. With a Launchfile a topic and other sensor parameters can be bind on an UID.

##### Hinweise / Hints

Es wird nicht empfohlen zwei Instanzen des Programmes zu starten (z.B. zwei Master Bricks), da sich die Sensor Topics sonst überlagern.

It's not recommended to run more than one instance of the program, because different sensors of same type will be published into one topic.

### Unterstütze Geräte / Supported Devices

Bricklets:

* Ambient Light => sensor_msgs/Illuminance
* Ambient Light 2.0 => sensor_msgs/Illuminance
* Distance IR => sensor_msgs/Range
* Distance US => sensor_msgs/Range
* GPS (Test) => sensor_msgs/NavSatFix
* Humidity => sensor_msgs/Humidity
* Motion Detector (Test)
* Temperature => sensor_msgs/Temperature
* Temperature IR => sensor_msgs/Temperature

Bricks:

* IMU => sensor_msgs/Imu
* IMU 2.0 => sensor_msgs/Imu

### Installation

Herunterladen des Git-Repository in das catkin workspace Verzeichnis.

clone the git repository to catkin workspace.

`git clone https://github.com/gus484/ros-tinkerforge_sensors`

Workspace kompilieren / build workspace

`catkin_make`

### Ausführen / run node

`rosrun tinkerforge_sensors tinkerforge_sensors_node`

### Launchfile

Argumente
* port (int) *Tinkerforge Port*
* ip (string) *Tinkerforge IP*

`roslaunch tinkerforge_sensors tinkerforge_sensors.launch`

#### Sensorparameter / sensor parameters

Einige Sensoren benötigen zusätzliche Parameter, damit sie korrekt funktionieren, z.B. die Reichweitenangaben des Distance IR Bricklets. Diese Parameter werden über die UID dem Programm mitgeteilt und in einer yaml-Datei (conf.yaml) gespeichert.

Some sensors require additional parameters in order to work properly, e.g. the range data of the Distance IR Bricklet. These parameters are stored in an yaml file (conf.yaml) and are read by the program over the UID. 

Aufbau einer Zeile / structure of a line:

`uid : {parameter1 = wert1, parameter2 = wert2, ...}`

Beispiel / example:

`n3J : {frame_id: 'base_ultrasonic', topic : 'range_us', max : 4.0, min : 0.02}`

*Siehe auch conf.yaml / see also conf.yaml*

Unterstützte Parameter / supported parameters

* all => topic (string) ; frame_id (string)
* Distance IR / Distance US => max (double) ; min (double)

### ToDo

* mehr Sensoren unterstützen / suport more sensors
* Kalibrierung für Distance US Sensor / calibration for Distance US sensor
* mehr Konfigurationsmöglichkeiten / more config options
* Verwaltung mehrerer Master / handle more than one master
