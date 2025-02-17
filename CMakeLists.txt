cmake_minimum_required(VERSION 3.0)


project(gen_odr)



aux_source_directory(src SRC_LIST)

include_directories(include)

add_executable(${PROJECT_NAME} ${SRC_LIST})


target_link_libraries(${PROJECT_NAME} tinyxml2)