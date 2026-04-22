cmake_minimum_required(VERSION 3.8)
project(operator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(${PROJECT_NAME} src/operator.c)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION bin/${PROJECT_NAME}
)
