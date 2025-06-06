cmake_minimum_required(VERSION 3.8)
project(paper_evaluation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(timers_only src/timers_only.cpp)

ament_auto_add_executable(timers_only_high_utilization src/timers_only_high_utilization.cpp)

ament_auto_add_executable(timers_only_over_utilization src/timers_only_over_utilization.cpp)

ament_auto_add_executable(sequences src/sequences.cpp)

ament_auto_add_executable(sequences_high_utilization src/sequences_high_utilization.cpp)

ament_auto_add_executable(many_to_many_pub src/many_to_many_pub.cpp)

ament_auto_add_executable(many_to_many_sink src/many_to_many_sink.cpp)

# ament_auto_add_executable(sequences_over_utilization src/sequences_over_utilization.cpp)

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


install(TARGETS
  timers_only timers_only_high_utilization timers_only_over_utilization many_to_many_pub
  sequences sequences_high_utilization many_to_many_sink
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
