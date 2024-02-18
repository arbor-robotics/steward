cmake_minimum_required(VERSION 3.8)
project(route_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# message generation
ament_auto_generate_code()

# Build a shared library
file(GLOB_RECURSE source_filenames "src/*.cpp") # Find all source files in src/
if(source_filenames) # If we have any source files
  ament_auto_add_library(${PROJECT_NAME}_lib SHARED DIRECTORY src) # Then make a library
endif()

# Build all executables
file(GLOB_RECURSE executable_filenames "exe/*.cpp") # Get all source files in exe/
foreach(filename ${executable_filenames}) # Iterate over them
  get_filename_component(executable_name ${filename} NAME_WE) # Pull the name with no extension
  ament_auto_add_executable(${executable_name} ${filename}) # Add an executable by that name
endforeach()

# Build all tests
file(GLOB test_filenames "test/*.cpp") # Find test source files
if(BUILD_TESTING AND test_filenames) # If we have any tests to build
  ament_auto_find_test_dependencies() # Find the test dependencies
  ament_auto_add_gtest(tests ${test_filenames}) # And add our tests
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
