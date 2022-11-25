# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bme280)

FILE(GLOB app_sources src/*.c)
#target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE  src/main.c services/my_service.c)

zephyr_library_include_directories(.)