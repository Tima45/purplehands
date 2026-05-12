cmake_minimum_required(VERSION 3.10)
project(driver_purplehands VERSION 1.0.0 LANGUAGES CXX)

# Стандарт C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Вшиваем библиотеки MinGW внутрь DLL (чтобы не было Ошибки 126)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -static-libgcc -static-libstdc++")

# Указываем путь к папке openvr
set(OPENVR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/openvr")

# Собираем SHARED библиотеку (DLL)
add_library(${PROJECT_NAME} SHARED driver_main.cpp)

# Подключаем заголовочные файлы
target_include_directories(${PROJECT_NAME} PRIVATE "${OPENVR_DIR}/headers")

# Подключаем системную библиотеку сокетов Windows
target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32)

# Убираем префикс "lib", чтобы на выходе был строго driver_purplehands.dll
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")