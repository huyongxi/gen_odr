cmake_minimum_required(VERSION 3.0)


project(gen_odr)



set(EigenDir /home/hyx/data/install_dir/include/eigen-3.4.0)
aux_source_directory(src SRC_LIST)

include_directories(include)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_include_directories(${PROJECT_NAME} PRIVATE ${EigenDir})
target_link_libraries(${PROJECT_NAME} tinyxml2)