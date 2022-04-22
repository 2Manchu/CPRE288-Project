cmake_minimum_required(VERSION 3.21)
project(CPRE288_Project C)

set(CMAKE_C_STANDARD 11)

include_directories(Libraries)

add_executable(CPRE288_Project
        Libraries/adc.c
        Libraries/adc.h
        Libraries/button.c
        Libraries/button.h
        Libraries/ping.c
        Libraries/ping.h
        Libraries/servo.c
        Libraries/servo.h
        Libraries/uart-interrupt.c
        Libraries/uart-interrupt.h
        template.c
        test_stuff.c
        tm4c123gh6pm_startup_ccs.c
        Libraries/tm4c123gh6pm.h Libraries/scan.c Libraries/scan.h Libraries/movement.c Libraries/movement.h)
