
#ifndef CODE_MY_MATH_H_
#define CODE_MY_MATH_H_


#include "zf_common_headfile.h"


/* ========= 常量定义 ========= */

/* 实数类型定义（便于类型替换） */
#define REAL   float

/* 快速atan2相关常量 */
#define TAN_MAP_RES     0.003921569f     /* 查找表最小分辨率（1/255） */
#define RAD_PER_DEG     0.017453293f     /* 角度转弧度系数：π/180 */
#define TAN_MAP_SIZE    256              /* 查找表大小 */
#define MY_PPPIII   3.14159f             /* π的近似值 */
#define MY_PPPIII_HALF   1.570796f       /* π/2的近似值 */
#define GRAVITY_ACCEL   9.8f                           // 重力加速度(m/s²)
#define HALF_SQRT_2     0.70710678118654757f           // √2/2
/* ========= 实用宏定义 ========= */

/* 向量范数计算 */
#define my_3_norm(x, y, z) (my_sqrt(my_pow((x)) + my_pow((y)) + my_pow((z))))  // 三维向量模长
#define my_2_norm(x, y) (my_sqrt(my_pow((x)) + my_pow((y))))                    // 二维向量模长

/* 平方计算 */
#define my_pow(a) ((a)*(a))  // 计算平方，比powf(a,2)更快

/* 安全除法：避免除以零 */
#define safe_div(numerator, denominator, safe_value) \
    ((denominator == 0) ? (safe_value) : ((numerator)/(denominator)))


/* 限幅宏：将x限制在[min, max]范围内 */
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* 最小值/最大值宏（多版本兼容） */
#define _MIN(a, b) ((a) < (b) ? (a) : (b))
#define _MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define Max(a, b) ((a) > (b) ? (a) : (b))  // 另一种命名
#define Min(a, b) ((a) < (b) ? (a) : (b))  // 另一种命名

/* 平方曲线函数：in * ((1-a) + a*|in/max|) */
#define my_pow_2_curve(in, a, max) (((1.0f - (a)) + (a) * ABS((in) / (max))) * in)

/* ========= 函数声明 ========= */

/* 基本数学函数 */
float my_abs(float f);                      // 绝对值
REAL fast_atan2(REAL y, REAL x);           // 快速四象限反正切
float my_sqrt(float number);               // 快速平方根
double mx_sin(double rad);                 // 核心正弦函数（[-π,π]范围）
double my_sin(double rad);                 // 正弦函数（任意角度）
float my_cos(double rad);                  // 余弦函数（任意角度）

/* 信号处理函数 */
float my_deadzone(float x, float ref, float zoom);      // 死区处理（类型1）
float my_deadzone_2(float x, float ref, float zoom);    // 死区处理（类型2）


/* 角度处理函数 */
float To_180_degrees(float x);                         // 角度规约到[-180,180]度
double To_180_degrees_db(double x);                    // 双精度版本



float SafeArcsin(float v);
float ConstrainFloat(float amt, float low, float high);
int16_t ConstrainInt16(int16_t amt, int16_t low, int16_t high);
uint16_t ConstrainUint16(uint16_t amt, uint16_t low, uint16_t high);
int32_t ConstrainInt32(int32_t amt, int32_t low, int32_t high);

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband);
float ApplyDeadbandFloat(float value, float deadband);

float Sq(float v);
float Pythagorous2(float a, float b);
float Pythagorous3(float a, float b, float c);
float Pythagorous4(float a, float b, float c, float d);

float WrapDegree360(float angle);
float WrapRadiansPi(float radians);




#endif /* CODE_MY_MATH_H_ */
