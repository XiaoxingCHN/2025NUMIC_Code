#ifndef __ALGORITHM_APRILTAG_H__
#define __ALGORITHM_APRILTAG_H__

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 配置宏 ========================= */

/** 单帧最多返回的标签数量 */
#ifndef APRILTAG_MAX_DETECTIONS
#define APRILTAG_MAX_DETECTIONS 8
#endif

/**
 * @brief 解码时允许的最大汉明距离（36h11 家族汉明距离为 11，
 *        允许 0 个位错误（精确匹配）可大幅减少误检率）
 */
#ifndef APRILTAG_MAX_HAMMING
#define APRILTAG_MAX_HAMMING 1
#endif

/** 四边形检测时，最小边长（像素） */
#ifndef APRILTAG_MIN_QUAD_SIDE
#define APRILTAG_MIN_QUAD_SIDE 10
#endif

/* =========================================================== */

/**
 * @brief 单个 AprilTag 检测结果
 */
typedef struct {
    int     id;           /*!< 标签 ID（36h11 家族：0-586） */
    int     hamming;      /*!< 解码汉明距离（0 = 无错误）  */
    float   cx;           /*!< 标签中心 X 坐标（像素）     */
    float   cy;           /*!< 标签中心 Y 坐标（像素）     */
    float   corners[4][2]; /*!< 四角坐标 [TL,TR,BR,BL][x,y]（像素） */

    /**
     * @brief 位姿估计结果（需要调用 AprilTag_EstimatePose() 后才有效）
     *
     * pose_t[3] : 相机坐标系下的平移向量 [x, y, z]（单位：与 tag_size 相同）
     * pose_r[3] : Rodrigues 旋转向量                                       */
    float   pose_t[3];
    float   pose_r[3];
    int     pose_valid;   /*!< 位姿是否已计算（0 = 未计算）                */
} AprilTag_Detection_t;

/**
 * @brief 一帧的检测结果集合
 */
typedef struct {
    int                  count;
    AprilTag_Detection_t tags[APRILTAG_MAX_DETECTIONS];
} AprilTag_Result_t;

/**
 * @brief 相机内参（用于位姿估计）
 */
typedef struct {
    float fx;  /*!< 水平焦距（像素）*/
    float fy;  /*!< 垂直焦距（像素）*/
    float cx;  /*!< 主点 X（像素）  */
    float cy;  /*!< 主点 Y（像素）  */
    float tag_size; /*!< 标签实际尺寸（与期望 pose_t 单位相同，如：米）*/
} AprilTag_Camera_t;

/* ========================= API ========================= */

/**
 * @brief 初始化 AprilTag 检测器
 *
 * 分配内部工作缓冲区，只需调用一次。
 *
 * @param width   图像宽度（像素）
 * @param height  图像高度（像素）
 * @return ESP_OK 成功，ESP_ERR_NO_MEM 内存不足
 */
esp_err_t AprilTag_Init(int width, int height);

/**
 * @brief 释放由 AprilTag_Init() 分配的资源
 */
void AprilTag_Deinit(void);

/**
 * @brief 在灰度图像上检测 AprilTag（36h11 家族）
 *
 * @param[in]  gray   灰度图数据（row-major，每像素 1 字节，值 0-255）
 * @param[in]  width  图像宽度（必须与 AprilTag_Init() 一致）
 * @param[in]  height 图像高度（必须与 AprilTag_Init() 一致）
 * @param[out] result 检测结果；调用前无需清零，函数会覆盖 count 字段
 * @return ESP_OK 成功（result->count 可能为 0）
 *         ESP_ERR_INVALID_ARG 参数为 NULL 或尺寸不匹配
 *         ESP_ERR_INVALID_STATE AprilTag_Init() 未调用
 */
esp_err_t AprilTag_Detect(const uint8_t *gray, int width, int height,
                          AprilTag_Result_t *result);

/**
 * @brief 对已检测到的标签进行位姿估计
 *
 * 基于单应矩阵（Homography）与相机内参估计 6-DOF 位姿。
 * 估计完成后 detection->pose_valid 置 1。
 *
 * @param[in,out] detection  待估计的单个检测结果
 * @param[in]     cam        相机内参
 * @return ESP_OK 成功，ESP_ERR_INVALID_ARG 参数为 NULL
 */
esp_err_t AprilTag_EstimatePose(AprilTag_Detection_t *detection,
                                const AprilTag_Camera_t *cam);

#ifdef __cplusplus
}
#endif

#endif /* __ALGORITHM_APRILTAG_H__ */
