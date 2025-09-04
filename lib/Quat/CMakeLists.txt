set(SOURCES ./Quat.cpp ./Madgwick.cpp)

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS .
                       REQUIRES utility)
