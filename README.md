**General Information and References**
-

* This project is created and tested with STM32F070RB. When implementing this code to other projects with different MCUs, the configurations of pins, DHT11, timer and UART, and also the timer's prescaler values based on MCU's frequency will need to be changed.
* The measuring period (ranging from 1 to 65536 (2^16) ms) and uart baudrate should be given as a parameter to init_and_start_distance_measuring function (declared in application_layer_driver.c).

**Things to do before trying the project codes**
-

1) HCSR04, timer and UART drivers' header files and source codes are located in /Drivers/Hardware_Drivers /Inc and /Src files. Because STM32CubeIDE doesn't have default paths to these folders, these folders should be added to project's path, in order to resolve the "header files and source codes can't be found" error. To do so, user should right-click to the project, then Properties -> C/C++ Build -> Settings -> MCU GCC Compiler -> Include Paths, then add the /Hardware_Drivers/Inc folder to the path.
2) To use UART's TX and console write functions without errors and to display the float variables succesfully on serial console, user should enable the "Use float with printf from newlib-nano (-u _print_float)" setting. To do so, user should right-click to the project, then Properties -> C/C++ Build -> Settings -> MCU Settings and then enable the tickbox for "Use float with printf from newlib-nano (-u _print_float)".

**Changelog & Updates**
-

17.05.2024
-
* Recreated the timer library to be more efficient and accurate.
* Added a dynamic prescaler and arr register changer function to update the measurement periods and input capture interrupt's frequency while timer is in use.

18.05.2024
-
* Implemented custom made DHT11 library, which is working almost as a non-blocking code thanks to the interrupts, and started to use temperature data when measuring the distance, in order to get more accurate results. 

**Known Bugs and Limitations**
-

* Because of the majority of main functions working with timer interrupts (instead of blocking HAL delays and other delays) and the used timer is a 16-bit timer, the period between measurements parameters' max value is 65535 (2^16) milliseconds, which is 65.5 seconds, in this configuration functions. If a measurement period longer than 65.5 seconds is needed in some applications, a new timer function with a different timer prescaler approach is needed.

**To Do List**
-

1) ~~To improve the accuracy of measurements, a temperature sensor integration will be made. Because sound of speed changes in different temperatures with this formula: 331*(sqrt(1+(temp/273)), the temperature coefficient when finding the [distance = (difference/2)*((331*(sqrt(1+(temp/273))))*0.0001);] distance should be varied on temperature and humidity.~~ Done on 18.05.2024.
2) ~~The measuring frequency's max value is 65536 us (65,536 ms = 2^16 us) for now. Hence, measuring in every 65 ms is the slowest option. It's because of the 2^16 bit limit in prescaler and arr registers. Adding a HAL_Delay isn't recommended because of the risk of slowing the interrupt function and giving incorrect measurements. A dynamic changing prescaler and arr approach will be added later.~~ Done on 17.05.2024.
