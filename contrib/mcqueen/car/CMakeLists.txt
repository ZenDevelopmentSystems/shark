
add_library(car Car.cpp I2cBus.cpp PCA9685.cpp PwmEsc.cpp PwmServo.cpp )

# Make sure the compiler can find include files for our car library
# when other libraries or executables link to Hello
target_include_directories (car PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
