cmake_minimum_required(VERSION 3.25)
project(Regelungstechnik C)

set(CMAKE_C_STANDARD 11)

add_executable(Regelungstechnik main.c Regelungstechnik_lib/Regelungstechnik.c Regelungstechnik_lib/Regelungstechnik.h)
