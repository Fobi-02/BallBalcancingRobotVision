cmake_minimum_required(VERSION 3.10)  # Correzione della versione di CMake

# Nome del progetto
project(ProvaOpenCV)

# Imposta il percorso di OpenCV se necessario (facoltativo)
set(OpenCV_DIR "C:/opencv/build/install")

# Trova il pacchetto OpenCV
find_package(OpenCV REQUIRED)

# Includi le directory di intestazione di OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Aggiungi il file sorgente
#add_executable(ProvaOpenCV BallBalancingRobot.cpp)
add_executable(ProvaOpenCV BallBalancingRobot.cpp packages/rtfir.cpp)
set_target_properties(ProvaOpenCV PROPERTIES
    OUTPUT_NAME "RunRobot"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/executable"
)

# Collega le librerie di OpenCV al tuo eseguibile
target_link_libraries(ProvaOpenCV ${OpenCV_LIBS} ole32 oleaut32 uuid)