**General Information and References**
-

* This project is created with STM32F070RB. When implementing this code to other projects with different MCUs, the pin configurations (especially HCSR04 connections and configs in uart_driver.c and uart_driver.h) will need to be changed.
* To change the measuring frequency (default 60ms), the "MEASURING_FREQ_IN_US" macro's value can be changed in timer_and_hcsr04_combined_driver.h header file. For now, max value is 65536 us (2^16) because of the limits in prescaler and arr registers. A dynamic changing prescaler and arr approach will be added later. 

**Things to do before trying the project codes**
-

1) HCSR04, timer and UART drivers' header files and source codes are located in /Drivers/Hardware_Drivers /Inc and /Src files. Because STM32CubeIDE doesn't have default paths to these folders, these folders should be added to project's path, in order to resolve the "header files and source codes can't be found" error. To do so, user should right-click to the project, then Properties -> C/C++ Build -> Settings -> MCU GCC Compiler -> Include Paths, then add the /Hardware_Drivers/Inc folder to the path.
2) To use UART's TX and console write functions without errors and to display the float variables succesfully on serial console, user should enable the "Use float with printf from newlib-nano (-u _print_float)" setting. To do so, user should right-click to the project, then Properties -> C/C++ Build -> Settings -> MCU Settings and then enable the tickbox for "Use float with printf from newlib-nano (-u _print_float)".

**Known Bugs**
-

**To Do List**
-

1) To improve the accuracy of measurements, a temperature sensor integration will be made. Because sound of speed changes in different temperatures with this formula: 331*(sqrt(1+(temp/273)), the temperature coefficient when finding the [distance = (difference/2)*((331*(sqrt(1+(temp/273))))*0.0001);] distance should be varied on temperature and humidity.
2) The measuring frequency's max value is 65536 us (65,536 ms = 2^16 us) for now. Hence, measuring in every 65 ms is the slowest option. It's because of the 2^16 bit limit in prescaler and arr registers. Adding a HAL_Delay isn't recommended because of the risk of slowing the interrupt function and giving incorrect measurements. A dynamic changing prescaler and arr approach will be added later. 
