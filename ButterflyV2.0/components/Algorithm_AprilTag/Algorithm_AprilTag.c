/**
 * @file Algorithm_AprilTag.c
 * @brief AprilTag 36h11 检测算法（ESP-IDF / ESP32 实现）
 *
 * 检测流程
 * --------
 *  1. 自适应阈值二值化（局部均值法，块大小 TILE_SIZE）
 *  2. 连通域标记（4 连通，Union-Find）
 *  3. 每个连通域的凸包近似 → 四边形筛选
 *  4. 透视变换 → 采样 10×10 标签像素
 *  5. 36 位解码 + 4 旋转 → 与 36h11 码表比对（支持 1 位纠错）
 *  6. 可选位姿估计（单应矩阵分解法）
 */

#include "Algorithm_AprilTag.h"
#include "apriltag_36h11_codes.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "AprilTag";

/* ========================= 配置常量 ========================= */
#define TILE_SIZE      16      /* 自适应阈值块大小（像素）  */
#define TILE_OFFSET    8       /* 偏置，防止平坦区域被二值化（灰度值） */
#define MAX_BLOBS      512     /* 最多追踪的连通域数量 */
#define MAX_CONTOUR    256     /* 轮廓点数上限          */
#define TAG_GRID       10      /* 标签总格数（含边框）   */
#define DATA_GRID      6       /* 数据区格数            */
#define SAMPLE_CELLS   (DATA_GRID * DATA_GRID)  /* 36 数据位  */

/* ========================= 内部数据结构 ========================= */

typedef struct {
    float x, y;
} Vec2f;

/* 连通域（Blob）统计 */
typedef struct {
    int   label;
    int   pixel_count;
    float sum_x, sum_y;
    int   min_x, max_x, min_y, max_y;
} Blob_t;

/* 静态工作缓冲区（避免栈溢出） */
static uint8_t *s_binary   = NULL; /* 二值图 */
static int     *s_labels   = NULL; /* 标签图 */
static int     *s_uf_parent= NULL; /* Union-Find 父节点 */
static Blob_t  *s_blobs    = NULL; /* 连通域数组 */
static int      s_width    = 0;
static int      s_height   = 0;
static int      s_initialized = 0;

/* ========================= Union-Find ========================= */

static int uf_find(int *parent, int x)
{
    while (parent[x] != x) {
        parent[x] = parent[parent[x]]; /* 路径压缩 */
        x = parent[x];
    }
    return x;
}

static void uf_union(int *parent, int a, int b)
{
    int ra = uf_find(parent, a);
    int rb = uf_find(parent, b);
    if (ra != rb) parent[ra] = rb;
}

/* ========================= 自适应阈值 ========================= */

/**
 * 将灰度图 gray 二值化结果写入 s_binary。
 * 每个 TILE_SIZE×TILE_SIZE 块内计算局部均值，低于均值 - TILE_OFFSET 的
 * 像素标记为前景（黑色目标，值 0），其余为背景（值 1）。
 */
static void adaptive_threshold(const uint8_t *gray, int w, int h)
{
    for (int ty = 0; ty < h; ty += TILE_SIZE) {
        for (int tx = 0; tx < w; tx += TILE_SIZE) {
            int x0 = tx, y0 = ty;
            int x1 = (tx + TILE_SIZE < w) ? tx + TILE_SIZE : w;
            int y1 = (ty + TILE_SIZE < h) ? ty + TILE_SIZE : h;

            /* 块内均值 */
            long sum = 0;
            int  cnt = 0;
            for (int y = y0; y < y1; y++) {
                for (int x = x0; x < x1; x++) {
                    sum += gray[y * w + x];
                    cnt++;
                }
            }
            uint8_t mean = (cnt > 0) ? (uint8_t)(sum / cnt) : 128u;
            uint8_t thresh = (mean > TILE_OFFSET) ? (mean - TILE_OFFSET) : 0;

            for (int y = y0; y < y1; y++) {
                for (int x = x0; x < x1; x++) {
                    s_binary[y * w + x] = (gray[y * w + x] < thresh) ? 0u : 1u;
                }
            }
        }
    }
}

/* ========================= 连通域标记 ========================= */

/**
 * 在 s_binary 上做 4 连通 Union-Find 标记，结果写入 s_labels。
 * 返回：连通域数量（不含背景）
 */
static int connected_components(int w, int h)
{
    int num_pixels = w * h;
    for (int i = 0; i < num_pixels; i++) s_uf_parent[i] = i;

    /* 第一遍：赋临时标签并合并 */
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            if (s_binary[idx] != 0) {
                s_labels[idx] = -1; /* 背景 */
                continue;
            }
            /* 检查左邻和上邻 */
            if (x > 0 && s_binary[idx - 1] == 0) {
                uf_union(s_uf_parent, idx, idx - 1);
            }
            if (y > 0 && s_binary[idx - w] == 0) {
                uf_union(s_uf_parent, idx, idx - w);
            }
        }
    }

    /* 第二遍：路径压缩 + 重新编号 */
    int label_map[MAX_BLOBS];
    memset(label_map, -1, sizeof(label_map));
    int next_label = 0;

    for (int i = 0; i < num_pixels; i++) {
        if (s_binary[i] != 0) {
            s_labels[i] = -1;
            continue;
        }
        int root = uf_find(s_uf_parent, i);
        /* 只给不同 root 分配递增标签，上限 MAX_BLOBS */
        int lbl = -1;
        /* 线性搜索 label_map（像素数/块数有限，OK） */
        for (int k = 0; k < next_label; k++) {
            if (label_map[k] == root) { lbl = k; break; }
        }
        if (lbl < 0) {
            if (next_label < MAX_BLOBS) {
                lbl = next_label;
                label_map[next_label++] = root;
            } else {
                lbl = 0; /* 溢出时归到 0，该 blob 可能不完整但不崩溃 */
            }
        }
        s_labels[i] = lbl;
    }
    return next_label;
}

/* ========================= 四边形检测 ========================= */

/**
 * 对一个 bbox 内的 blob 像素做简化的角点提取：
 * 找到距中心最远的 4 个"极值"方向像素作为四角候选。
 *
 * @param label  目标 blob 标签
 * @param blob   blob 统计信息
 * @param corners 输出四角（TL TR BR BL），各为 (x,y)
 * @return 1 = 成功，0 = blob 太小或不像四边形
 */
static int extract_quad(int label, const Blob_t *blob, Vec2f corners[4])
{
    int x0 = blob->min_x, x1 = blob->max_x;
    int y0 = blob->min_y, y1 = blob->max_y;
    int bw = x1 - x0 + 1;
    int bh = y1 - y0 + 1;

    if (bw < APRILTAG_MIN_QUAD_SIDE || bh < APRILTAG_MIN_QUAD_SIDE) return 0;
    /* 宽高比过于极端则跳过 */
    float aspect = (bw > bh) ? ((float)bw / bh) : ((float)bh / bw);
    if (aspect > 4.0f) return 0;

    float cx = blob->sum_x / blob->pixel_count;
    float cy = blob->sum_y / blob->pixel_count;

    /* 搜索 4 个角方向：TL(-,-), TR(+,-), BR(+,+), BL(-,+) */
    float best_d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float best_x[4] = {cx, cx, cx, cx};
    float best_y[4] = {cy, cy, cy, cy};

    int w = s_width;
    for (int y = y0; y <= y1 && y < s_height; y++) {
        for (int x = x0; x <= x1 && x < w; x++) {
            if (s_labels[y * w + x] != label) continue;
            float dx = x - cx;
            float dy = y - cy;
            /* 分象限 */
            int q;
            if      (dx <= 0 && dy <= 0) q = 0; /* TL */
            else if (dx >  0 && dy <= 0) q = 1; /* TR */
            else if (dx >  0 && dy >  0) q = 2; /* BR */
            else                          q = 3; /* BL */
            float d = dx * dx + dy * dy;
            if (d > best_d[q]) {
                best_d[q] = d;
                best_x[q] = (float)x;
                best_y[q] = (float)y;
            }
        }
    }

    /* 验证：4 角间的最小距离需大于最小边长 */
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        float dx = best_x[j] - best_x[i];
        float dy = best_y[j] - best_y[i];
        float side = sqrtf(dx * dx + dy * dy);
        if (side < (float)APRILTAG_MIN_QUAD_SIDE) return 0;
    }

    for (int i = 0; i < 4; i++) {
        corners[i].x = best_x[i];
        corners[i].y = best_y[i];
    }
    return 1;
}

/* ========================= 透视变换采样 ========================= */

/**
 * 计算从单位正方形 [0,1]^2 到四边形 dst[4] 的单应矩阵 H（3x3）。
 * 四角顺序：TL(0,0) TR(1,0) BR(1,1) BL(0,1)。
 */
static void compute_homography(const Vec2f corners[4], float H[9])
{
    /* 目标点（图像坐标）*/
    float x0 = corners[0].x, y0 = corners[0].y; /* TL */
    float x1 = corners[1].x, y1 = corners[1].y; /* TR */
    float x2 = corners[2].x, y2 = corners[2].y; /* BR */
    float x3 = corners[3].x, y3 = corners[3].y; /* BL */

    /* 源点（单位正方形坐标）*/
    /* (0,0)->(x0,y0), (1,0)->(x1,y1), (1,1)->(x2,y2), (0,1)->(x3,y3) */

    /* 使用 DLT（Direct Linear Transform）8 方程求解 8 个自由度 */
    /* Ah = 0，A 为 8×8 矩阵，h 为 8 元素向量（H33=1 归一化后） */

    /* 简化版本：用闭合公式直接计算四点到四点单应 */
    /* 参考：Hartley & Zisserman "Multiple View Geometry" App. 4 */
    float A[8][8] = {0};
    float b[8] = {x0, y0, x1, y1, x2, y2, x3, y3};
    /* 源点 */
    static const float SX[4] = {0, 1, 1, 0};
    static const float SY[4] = {0, 0, 1, 1};

    for (int i = 0; i < 4; i++) {
        float sx = SX[i], sy = SY[i];
        float dx = b[i * 2], dy = b[i * 2 + 1];
        /* 第 2i 行 */
        A[i*2][0] = sx;  A[i*2][1] = sy;  A[i*2][2] = 1;
        A[i*2][3] = 0;   A[i*2][4] = 0;   A[i*2][5] = 0;
        A[i*2][6] = -dx * sx; A[i*2][7] = -dx * sy;
        b[i*2] = dx;
        /* 第 2i+1 行 */
        A[i*2+1][0] = 0; A[i*2+1][1] = 0; A[i*2+1][2] = 0;
        A[i*2+1][3] = sx;A[i*2+1][4] = sy;A[i*2+1][5] = 1;
        A[i*2+1][6] = -dy * sx; A[i*2+1][7] = -dy * sy;
        b[i*2+1] = dy;
    }

    /* 高斯消元（8×8）求解 */
    for (int col = 0; col < 8; col++) {
        /* 选主元 */
        int pivot = col;
        float max_val = fabsf(A[col][col]);
        for (int row = col + 1; row < 8; row++) {
            if (fabsf(A[row][col]) > max_val) {
                max_val = fabsf(A[row][col]);
                pivot = row;
            }
        }
        /* 交换行 */
        if (pivot != col) {
            for (int k = 0; k < 8; k++) {
                float tmp = A[col][k]; A[col][k] = A[pivot][k]; A[pivot][k] = tmp;
            }
            float tmp = b[col]; b[col] = b[pivot]; b[pivot] = tmp;
        }
        if (fabsf(A[col][col]) < 1e-8f) continue;
        float inv = 1.0f / A[col][col];
        for (int row = 0; row < 8; row++) {
            if (row == col) continue;
            float factor = A[row][col] * inv;
            for (int k = col; k < 8; k++) A[row][k] -= factor * A[col][k];
            b[row] -= factor * b[col];
        }
        /* 归一化主元行 */
        for (int k = col; k < 8; k++) A[col][k] *= inv;
        b[col] *= inv;
    }

    H[0] = b[0]; H[1] = b[1]; H[2] = b[2];
    H[3] = b[3]; H[4] = b[4]; H[5] = b[5];
    H[6] = b[6]; H[7] = b[7]; H[8] = 1.0f;
}

/**
 * 利用单应矩阵 H 将归一化坐标 (nx, ny) 映射到图像坐标。
 */
static void homography_project(const float H[9], float nx, float ny,
                               float *ix, float *iy)
{
    float w = H[6] * nx + H[7] * ny + H[8];
    if (fabsf(w) < 1e-9f) w = 1e-9f;
    *ix = (H[0] * nx + H[1] * ny + H[2]) / w;
    *iy = (H[3] * nx + H[4] * ny + H[5]) / w;
}

/**
 * 采样 10×10 标签格，返回 36 个数据位组成的 uint64_t（bit 0 = 第一个数据格）。
 * 采样点顺序：从左上到右下，行优先。
 */
static uint64_t sample_tag_bits(const float H[9],
                                const uint8_t *gray, int w, int h)
{
    uint64_t code = 0;
    int bit_idx = 0;

    for (int row = 0; row < DATA_GRID && bit_idx < 36; row++) {
        for (int col = 0; col < DATA_GRID && bit_idx < 36; col++) {
            /* 数据格在 10×10 中的归一化坐标（跳过 2 层边框） */
            float nx = (col + 2.5f) / TAG_GRID;
            float ny = (row + 2.5f) / TAG_GRID;

            float ix, iy;
            homography_project(H, nx, ny, &ix, &iy);

            int px = (int)(ix + 0.5f);
            int py = (int)(iy + 0.5f);
            if (px < 0 || px >= w || py < 0 || py >= h) continue;

            uint8_t pixel = gray[py * w + px];
            if (pixel < 128) {
                code |= ((uint64_t)1 << bit_idx);
            }
            bit_idx++;
        }
    }
    return code;
}

/* ========================= 旋转 36 位码 ========================= */

static uint64_t rotate_code_90(uint64_t code)
{
    /* 36 位 6×6 矩阵旋转 90°（顺时针）
     * 原始 (row, col) -> 旋转后 (col, 5-row)
     * bit 位置 = row * 6 + col */
    uint64_t rot = 0;
    for (int row = 0; row < DATA_GRID; row++) {
        for (int col = 0; col < DATA_GRID; col++) {
            int src_bit = row * DATA_GRID + col;
            int dst_row = col;
            int dst_col = (DATA_GRID - 1) - row;
            int dst_bit = dst_row * DATA_GRID + dst_col;
            if ((code >> src_bit) & 1) {
                rot |= ((uint64_t)1 << dst_bit);
            }
        }
    }
    return rot;
}

/* ========================= 码表查找（支持 1 位汉明纠错） ========================= */

/**
 * 计算两个 36 位码之间的汉明距离。
 */
static int hamming_distance_36(uint64_t a, uint64_t b)
{
    uint64_t diff = (a ^ b) & 0x0FFFFFFFFFULL; /* 仅比较低 36 位 */
    return __builtin_popcountll(diff);
}

/**
 * 在 36h11 码表中查找与 code 最接近的条目。
 *
 * @param code       待查找的 36 位码（仅低 36 位有效）
 * @param[out] best_id  找到的标签 ID
 * @param[out] best_ham 汉明距离
 * @return 1 = 找到（距离 <= APRILTAG_MAX_HAMMING），0 = 未找到
 */
static int lookup_code(uint64_t code, int *best_id, int *best_ham)
{
    uint64_t masked = code & 0x0FFFFFFFFFULL;
    int min_ham = APRILTAG_MAX_HAMMING + 1;
    int min_id  = -1;

    for (int i = 0; i < (int)TAG36H11_NUM_CODES; i++) {
        int ham = hamming_distance_36(masked, TAG36H11_CODES[i]);
        if (ham < min_ham) {
            min_ham = ham;
            min_id  = i;
            if (ham == 0) break; /* 精确匹配，提前退出 */
        }
    }

    *best_id  = min_id;
    *best_ham = min_ham;
    return (min_ham <= APRILTAG_MAX_HAMMING) ? 1 : 0;
}

/* ========================= 公开 API ========================= */

esp_err_t AprilTag_Init(int width, int height)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "AprilTag already initialized, re-initializing");
        AprilTag_Deinit();
    }

    int num_pixels = width * height;

    s_binary    = (uint8_t *)malloc((size_t)num_pixels * sizeof(uint8_t));
    s_labels    = (int *)    malloc((size_t)num_pixels * sizeof(int));
    s_uf_parent = (int *)    malloc((size_t)num_pixels * sizeof(int));
    s_blobs     = (Blob_t *) calloc(MAX_BLOBS, sizeof(Blob_t));

    if (!s_binary || !s_labels || !s_uf_parent || !s_blobs) {
        ESP_LOGE(TAG, "AprilTag_Init: malloc failed");
        AprilTag_Deinit();
        return ESP_ERR_NO_MEM;
    }

    s_width       = width;
    s_height      = height;
    s_initialized = 1;

    ESP_LOGI(TAG, "AprilTag initialized (%dx%d)", width, height);
    return ESP_OK;
}

void AprilTag_Deinit(void)
{
    free(s_binary);    s_binary    = NULL;
    free(s_labels);    s_labels    = NULL;
    free(s_uf_parent); s_uf_parent = NULL;
    free(s_blobs);     s_blobs     = NULL;
    s_width = s_height = s_initialized = 0;
    ESP_LOGI(TAG, "AprilTag deinitialized");
}

esp_err_t AprilTag_Detect(const uint8_t *gray, int width, int height,
                          AprilTag_Result_t *result)
{
    if (!gray || !result) return ESP_ERR_INVALID_ARG;
    if (!s_initialized)   return ESP_ERR_INVALID_STATE;
    if (width != s_width || height != s_height) return ESP_ERR_INVALID_ARG;

    result->count = 0;

    /* ---- 步骤 1：自适应阈值 ---- */
    adaptive_threshold(gray, width, height);

    /* ---- 步骤 2：连通域标记 ---- */
    int num_blobs = connected_components(width, height);
    if (num_blobs <= 0) return ESP_OK;

    /* ---- 统计各 blob ---- */
    memset(s_blobs, 0, (size_t)MAX_BLOBS * sizeof(Blob_t));
    for (int i = 0; i < MAX_BLOBS; i++) {
        s_blobs[i].min_x = width;
        s_blobs[i].max_x = 0;
        s_blobs[i].min_y = height;
        s_blobs[i].max_y = 0;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int lbl = s_labels[y * width + x];
            if (lbl < 0 || lbl >= MAX_BLOBS) continue;
            Blob_t *b = &s_blobs[lbl];
            b->pixel_count++;
            b->sum_x += x;
            b->sum_y += y;
            if (x < b->min_x) b->min_x = x;
            if (x > b->max_x) b->max_x = x;
            if (y < b->min_y) b->min_y = y;
            if (y > b->max_y) b->max_y = y;
        }
    }

    /* ---- 步骤 3 & 4 & 5：四边形 → 采样 → 解码 ---- */
    for (int lbl = 0; lbl < num_blobs && result->count < APRILTAG_MAX_DETECTIONS; lbl++) {
        Blob_t *blob = &s_blobs[lbl];
        if (blob->pixel_count < APRILTAG_MIN_QUAD_SIDE * APRILTAG_MIN_QUAD_SIDE / 4) continue;

        Vec2f corners[4];
        if (!extract_quad(lbl, blob, corners)) continue;

        /* 单应矩阵 */
        float H[9];
        compute_homography(corners, H);

        /* 采样 36 个数据位 */
        uint64_t raw_code = sample_tag_bits(H, gray, width, height);

        /* 尝试 4 种旋转 */
        uint64_t rotated = raw_code;
        for (int rot = 0; rot < 4; rot++) {
            int id, ham;
            if (lookup_code(rotated, &id, &ham)) {
                AprilTag_Detection_t *det = &result->tags[result->count++];
                det->id      = id;
                det->hamming = ham;
                det->cx      = (corners[0].x + corners[1].x +
                                corners[2].x + corners[3].x) * 0.25f;
                det->cy      = (corners[0].y + corners[1].y +
                                corners[2].y + corners[3].y) * 0.25f;
                for (int c = 0; c < 4; c++) {
                    det->corners[c][0] = corners[c].x;
                    det->corners[c][1] = corners[c].y;
                }
                det->pose_valid = 0;
                break; /* 找到一个有效解后不再尝试其他旋转 */
            }
            rotated = rotate_code_90(rotated);
        }
    }

    if (result->count > 0) {
        ESP_LOGD(TAG, "Detected %d tag(s)", result->count);
    }
    return ESP_OK;
}

/* ========================= 位姿估计 ========================= */

/**
 * 基于标签四角的 2D-3D 对应关系，用单应矩阵法计算旋转向量和平移向量。
 *
 * 世界坐标系以标签中心为原点，标签平面为 XY 平面（Z=0）：
 *   TL = (-s/2, -s/2, 0)
 *   TR = ( s/2, -s/2, 0)
 *   BR = ( s/2,  s/2, 0)
 *   BL = (-s/2,  s/2, 0)
 * 其中 s = tag_size。
 */
esp_err_t AprilTag_EstimatePose(AprilTag_Detection_t *detection,
                                const AprilTag_Camera_t *cam)
{
    if (!detection || !cam) return ESP_ERR_INVALID_ARG;

    float s  = cam->tag_size * 0.5f;
    float fx = cam->fx, fy = cam->fy;
    float cx = cam->cx, cy = cam->cy;

    /* 图像坐标（像素）→ 归一化相机坐标 */
    float img[4][2];
    for (int i = 0; i < 4; i++) {
        img[i][0] = (detection->corners[i][0] - cx) / fx;
        img[i][1] = (detection->corners[i][1] - cy) / fy;
    }

    /* 世界坐标（标签坐标系，z=0） */
    const float world[4][2] = {
        {-s, -s}, { s, -s}, { s, s}, {-s, s}
    };

    /* 计算单应矩阵 H（从标签平面到归一化图像平面） */
    /* 重用 compute_homography，将 world 坐标归一化到 [0,1]  */
    /* 实际上需要直接用 [world_x, world_y] -> [img_x, img_y] */
    /* 这里构建 DLT 8×8 系统 */
    float A[8][8] = {0};
    float bv[8]   = {0};

    for (int i = 0; i < 4; i++) {
        float wx = world[i][0], wy = world[i][1];
        float ux = img[i][0],   uy = img[i][1];
        A[i*2][0] = wx;  A[i*2][1] = wy;  A[i*2][2] = 1;
        A[i*2][3] = 0;   A[i*2][4] = 0;   A[i*2][5] = 0;
        A[i*2][6] = -ux * wx; A[i*2][7] = -ux * wy;
        bv[i*2]   = ux;
        A[i*2+1][0] = 0; A[i*2+1][1] = 0; A[i*2+1][2] = 0;
        A[i*2+1][3] = wx;A[i*2+1][4] = wy;A[i*2+1][5] = 1;
        A[i*2+1][6] = -uy * wx; A[i*2+1][7] = -uy * wy;
        bv[i*2+1] = uy;
    }

    /* 高斯消元 */
    for (int col = 0; col < 8; col++) {
        int pivot = col;
        float max_val = fabsf(A[col][col]);
        for (int row = col + 1; row < 8; row++) {
            if (fabsf(A[row][col]) > max_val) {
                max_val = fabsf(A[row][col]);
                pivot = row;
            }
        }
        if (pivot != col) {
            for (int k = 0; k < 8; k++) {
                float tmp = A[col][k]; A[col][k] = A[pivot][k]; A[pivot][k] = tmp;
            }
            float tmp = bv[col]; bv[col] = bv[pivot]; bv[pivot] = tmp;
        }
        if (fabsf(A[col][col]) < 1e-8f) continue;
        float inv = 1.0f / A[col][col];
        for (int row = 0; row < 8; row++) {
            if (row == col) continue;
            float factor = A[row][col] * inv;
            for (int k = col; k < 8; k++) A[row][k] -= factor * A[col][k];
            bv[row] -= factor * bv[col];
        }
        for (int k = col; k < 8; k++) A[col][k] *= inv;
        bv[col] *= inv;
    }

    /* H = [h0 h1 h2; h3 h4 h5; h6 h7 1] */
    float h0=bv[0], h1=bv[1], h2=bv[2];
    float h3=bv[3], h4=bv[4], h5=bv[5];
    float h6=bv[6], h7=bv[7];

    /* 从 H 中恢复 R 和 t：
     * r1 = K^{-1} * h1_col（已经是归一化坐标，所以直接用）
     * 列向量：c1 = [h0,h3,h6], c2 = [h1,h4,h7]
     */
    float c1[3] = {h0, h3, h6};
    float c2[3] = {h1, h4, h7};
    float t[3]  = {h2, h5, 1.0f};

    /* 归一化（去除尺度因子 lambda） */
    float lam = 1.0f / sqrtf(c1[0]*c1[0] + c1[1]*c1[1] + c1[2]*c1[2]);
    for (int i = 0; i < 3; i++) { c1[i] *= lam; c2[i] *= lam; t[i] *= lam; }

    /* r3 = r1 × r2 */
    float r3[3] = {
        c1[1]*c2[2] - c1[2]*c2[1],
        c1[2]*c2[0] - c1[0]*c2[2],
        c1[0]*c2[1] - c1[1]*c2[0]
    };

    /* 旋转矩阵 R = [r1 | r2 | r3] */
    float R[9] = {
        c1[0], c2[0], r3[0],
        c1[1], c2[1], r3[1],
        c1[2], c2[2], r3[2]
    };

    /* 转为 Rodrigues 旋转向量 */
    float trace = R[0] + R[4] + R[8];
    float theta = acosf(fmaxf(-1.0f, fminf(1.0f, (trace - 1.0f) * 0.5f)));
    float sin_t = sinf(theta);
    if (fabsf(sin_t) < 1e-6f) {
        detection->pose_r[0] = detection->pose_r[1] = detection->pose_r[2] = 0.0f;
    } else {
        float k = theta / (2.0f * sin_t);
        detection->pose_r[0] = k * (R[7] - R[5]);
        detection->pose_r[1] = k * (R[2] - R[6]);
        detection->pose_r[2] = k * (R[3] - R[1]);
    }

    detection->pose_t[0] = t[0];
    detection->pose_t[1] = t[1];
    detection->pose_t[2] = t[2];
    detection->pose_valid = 1;

    return ESP_OK;
}
