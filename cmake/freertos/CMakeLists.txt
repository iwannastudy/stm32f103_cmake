cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(OS_Inc
    ${CMAKE_SOURCE_DIR}/FreeRTOS/include
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM3
)

set(OS_Common_Src
    ${CMAKE_SOURCE_DIR}/FreeRTOS/croutine.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/event_groups.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/list.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/queue.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/tasks.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/timers.c
)

set(OS_Port_Src
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM3/port.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/MemMang/heap_4.c
)



# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(OS_LINK_LIBS 
    FreeRTOS
    
)

# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${OS_Port_Src} ${OS_Common_Src})
target_include_directories(FreeRTOS PUBLIC ${OS_Inc})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${OS_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
