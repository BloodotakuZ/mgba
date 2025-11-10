set(GEKKONET_FRONTEND_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_executable(${BINARY_NAME}-gekkonet ${GEKKONET_FRONTEND_SRC})
set_target_properties(${BINARY_NAME}-gekkonet PROPERTIES
        COMPILE_DEFINITIONS "${OS_DEFINES};${FEATURE_DEFINES};${FUNCTION_DEFINES}"
)

target_include_directories(${BINARY_NAME}-gekkonet
        PRIVATE
                ${SDL_INCLUDE_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
                ${CMAKE_CURRENT_SOURCE_DIR}/../../../src
                ${CMAKE_CURRENT_SOURCE_DIR}/../../..
                ${GEKKONET_INCLUDE_DIR}
)

target_link_libraries(${BINARY_NAME}-gekkonet
        PRIVATE
                ${BINARY_NAME}
                ${SDL_LIBRARY}
                ${SDLMAIN_LIBRARY}
                ${OPENGL_LIBRARY}
                ${OPENGLES2_LIBRARY}
                GekkoNet::GekkoNet
)

debug_strip(${BINARY_NAME}-gekkonet)

install(TARGETS ${BINARY_NAME}-gekkonet DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ${BINARY_NAME}-gekkonet)
