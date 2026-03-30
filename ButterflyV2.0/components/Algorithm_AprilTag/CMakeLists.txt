# 注册 AprilTag 算法组件
idf_component_register(
    SRCS
        "Algorithm_AprilTag.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_common
)
