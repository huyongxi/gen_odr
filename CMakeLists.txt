cmake_minimum_required(VERSION 3.0)


project(gen_odr)

set(CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED TRUE)

set(eigen /home/hyx/data/install_dir/include/eigen-3.4.0)
set(nlohmann /home/hyx/data/install_dir/include/json-3.11.3/include)
aux_source_directory(src SRC_LIST)

include_directories(include)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_include_directories(${PROJECT_NAME} 
    PRIVATE ${eigen}
    PRIVATE ${nlohmann}
)

target_link_libraries(${PROJECT_NAME} tinyxml2)