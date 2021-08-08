# STM32F103C8-I2C-BMP280

:small_orange_diamond: In the project file, I2C communication is used while receiving data from BMP280 and temperature, pressure and altitude data were got.

<br />:point_right:STM32F103C8 ------> PB10 ------> I2C2_SCL &nbsp;:left_right_arrow: &nbsp; BME280 ------> SCK
<br />:point_right:STM32F103C8 ------> PB11 ------> I2C2_SDA        :left_right_arrow:       BME280 ------> SDI
<br />:point_right:STM32F103C8 ------> 5V&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;:left_right_arrow:BME280 ------> VIN
<br />:point_right:STM32F103C8 ------> GND&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;:left_right_arrow:BME280 ------> GND
<br />:point_right:BME280 ------> SDO ------> LOW (0)

![photo_2021-08-08_16-05-30](https://user-images.githubusercontent.com/75426545/128632939-b723fa30-28d5-488e-bb3e-b8d307fb90e2.jpg)
![Ekran görüntüsü 2021-08-08 161106](https://user-images.githubusercontent.com/75426545/128633095-645b5dfb-a067-4dd2-966c-f34362bd04ec.jpg)
