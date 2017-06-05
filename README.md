# DIY VR Controller
Simple DIY VR Controller study project with arduino  
Based on [i2cdev`s example](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino)


## Packet
| 0 |   1  2  3  4   |   5  6  7  8   |   9 10 11 12   |  13 14 15 16   |      |
|:-:|:--------------:|:--------------:|:--------------:|:--------------:|:----:|
|'$'|  Quaternion W  |  Quaternion X  |  Quaternion Y  |  Quaternion Z  |      |
|   |** 17 18 19 20**|** 21 22 23 24**|** 25 26 27 28**|** 29 30 31 32**|**33**|
|   |   Position W   |   Position X   |   Position Y   | Joystrick data |  '#' |
## Sensor
|   IMU    |  Trigger  |  Bluetooth  |
|:--------:|:---------:|:-----------:|
| MPU09250 | Joystrick |   HC - 05   |


## Connect all Sensor and arduino
![image](http://i.imgur.com/Payf8Nz.jpg)

## License
The content of this project itself is licensed under the [Creative Commons Attribution 3.0 license](http://creativecommons.org/licenses/by/3.0/us/deed.en_US), and the underlying source code used to format and display that content is licensed under the [MIT license](http://opensource.org/licenses/mit-license.php).
