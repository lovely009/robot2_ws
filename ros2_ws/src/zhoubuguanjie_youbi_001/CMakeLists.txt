cmake_minimum_required(VERSION 3.8)
project(zhoubuguanjie_youbi_001)

find_package(ament_cmake REQUIRED)

# 纯资源包：只有 URDF / mesh / launch / config，无编译目标，安装即完成
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
