# DIY VR Controller
Simple DIY VR Controller study project with arduino  
Based on [i2cdev`s example](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino)


## Packet
| 0 |   1  2  3  4   |   5  6  7  8   |   9 10 11 12   |  13 14 15 16   |      |
|:-:|:--------------:|:--------------:|:--------------:|:--------------:|:----:|
|'$'|  Quaternion W  |  Quaternion X  |  Quaternion Y  |  Quaternion Z  |      |
|   | **17 18 19 20**| **21 22 23 24**| **25 26 27 28**| **29 30 31 32**|**33**|
|   |   Position W   |   Position X   |   Position Y   | Joystrick data |  '#' |
## Sensor
|   IMU    |  Trigger  |  Bluetooth  |
|:--------:|:---------:|:-----------:|
| MPU09250 | Joystrick |   HC - 05   |


## Connect all Sensor and arduino
![image](http://i.imgur.com/Payf8Nz.jpg)

## License

<img align="right" src="http://opensource.org/trademarks/opensource/OSI-Approved-License-100x137.png">

The class is licensed under the [MIT License](http://opensource.org/licenses/MIT):

Copyright &copy; 2019 [Garage-de-Orca](http://www.github.com/Garage-de-Orca).

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
