set(libsocket_INCLUDE_DIRS "/home/wen/Desktop/as project/autonomoussystems2025/ros2_ws/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/wen/Desktop/as project/autonomoussystems2025/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
