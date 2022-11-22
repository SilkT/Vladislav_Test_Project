cmake_minimum_required(VERSION 3.20.0)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)

# NORDIC SDK APP START

target_sources(app PRIVATE
  src/main.c services/my_service.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
