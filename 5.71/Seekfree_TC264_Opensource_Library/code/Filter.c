
#include "Filter.h"

/*
 * 功能：各种滤波器实现文件
 * 包含：卡尔曼滤波器、巴特沃斯滤波器、移动平均滤波器等
*/

/**
 * @brief 一维卡尔曼滤波器算法
 * @param kf 卡尔曼滤波器结构体指针
 * @param input 当前的测量值（Measurement value）
 * @return float 滤波后的最优估计值
 *
 * @note 算法包含两个主要步骤：
 *       1. 预测（Predict）：基于上一时刻状态预测当前状态
 *       2. 更新（Update）：结合预测值和测量值得到最优估计
 */
float Com_Kalman_Filter(KalmanFilter_Struct *kf, float input)
{
    // ---------- 预测步骤（Prediction / Time Update）----------
    // 计算先验估计误差协方差：P(k|k-1) = P(k-1|k-1) + Q
    // 将上一时刻的后验估计误差协方差加上过程噪声，得到当前时刻的先验估计误差协方差
    kf->NowP = kf->LastP + kf->Q;

    // ---------- 更新步骤（Update / Measurement Update）----------
    // 计算卡尔曼增益：Kg = P(k|k-1) / [P(k|k-1) + R]
    // 卡尔曼增益决定了应该更相信预测值还是测量值：
    // - 当R较小时（测量准确），Kg接近1，更信任测量值
    // - 当R较大时（测量噪声大），Kg接近0，更信任预测值
    kf->Kg = kf->NowP / (kf->NowP + kf->R);

    // 计算后验状态估计：X(k|k) = X(k|k-1) + Kg * [Z(k) - X(k|k-1)]
    // 其中：
    // - X(k|k-1) 是上一时刻的输出（预测值），即 kf->out
    // - Z(k) 是当前测量值，即 input
    // - [Z(k) - X(k|k-1)] 是测量残差（Innovation）
    // 通过预测值和测量残差的加权平均，得到最优估计
    kf->out = kf->out + kf->Kg * (input - kf->out);

    // 计算后验估计误差协方差：P(k|k) = (1 - Kg) * P(k|k-1)
    // 更新估计误差协方差，用于下一次迭代
    kf->LastP = (1 - kf->Kg) * kf->NowP;

    // 返回当前时刻的最优估计值
    return kf->out;
}


/* ========= 宏定义 ========= */

/* 限幅宏：将x限制在[min, max]范围内 */
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


/**
 * @brief 2阶巴特沃斯低通滤波器
 * @param curr_input 当前输入值
 * @param Buffer 滤波器缓冲区指针
 * @param Parameter 滤波器参数指针
 * @return float 滤波后的输出值
 *
 * 功能：实现2阶IIR巴特沃斯低通滤波
 *
 * 差分方程：
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * 缓冲区说明：
 * Input_Butter[0], [1], [2]: 分别对应x[n-2], x[n-1], x[n]
 * Output_Butter[0], [1], [2]: 分别对应y[n-2], y[n-1], y[n]
 *
 * 启动阶段：前100个采样点直接通过，避免初始瞬态
 */
float LPButterworth(float curr_input, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    static int LPF_Cnt = 0;  // 滤波器启动计数器

    /* 1. 存储当前输入到缓冲区[2]位置 */
    Buffer->Input_Butter[2] = curr_input;

    /* 2. 检查是否完成启动阶段（前100个点） */
    if (LPF_Cnt >= 100)
    {
        /* 正常滤波阶段：应用2阶巴特沃斯差分方程 */
        Buffer->Output_Butter[2] =
            Parameter->b[0] * Buffer->Input_Butter[2] +   // b0*x[n]
            Parameter->b[1] * Buffer->Input_Butter[1] +   // b1*x[n-1]
            Parameter->b[2] * Buffer->Input_Butter[0] -   // b2*x[n-2]
            Parameter->a[1] * Buffer->Output_Butter[1] -  // a1*y[n-1]
            Parameter->a[2] * Buffer->Output_Butter[0];   // a2*y[n-2]
    }
    else
    {
        /* 启动阶段：直接通过输入值，避免初始瞬态响应 */
        Buffer->Output_Butter[2] = Buffer->Input_Butter[2];
        LPF_Cnt++;  // 递增启动计数器
    }

    /* 3. 更新输入历史缓冲区（滑动窗口） */
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];  // x[n-2] = x[n-1]
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];  // x[n-1] = x[n]

    /* 4. 更新输出历史缓冲区 */
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];  // y[n-2] = y[n-1]
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];  // y[n-1] = y[n]

    /* 5. 返回当前滤波输出 */
    return Buffer->Output_Butter[2];
}

/**
 * @brief 计算一阶低通滤波器的alpha参数
 * @param cutoff_frequency 截止频率（Hz）
 * @param time_step 时间步长（秒）
 * @return float alpha参数（0-1之间）
 *
 * 功能：计算一阶低通滤波器的alpha参数
 *
 * 一阶低通滤波器公式：
 * y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *
 * alpha计算公式：
 * alpha = dt / (dt + RC)
 * 其中RC = 1/(2*pi*fc)，fc为截止频率
 *
 * alpha取值范围：0-1
 * alpha越大，滤波效果越弱（跟随越快）
 * alpha越小，滤波效果越强（跟随越慢）
 */
float set_lpf_alpha(int16_t cutoff_frequency, float time_step)
{
    float lpf_alpha;      // 返回的alpha值
    float rc;             // RC时间常数

    /* 计算RC时间常数：RC = 1/(2*pi*fc) */
    rc = 1 / (2 * 3.14 * cutoff_frequency);

    /* 计算alpha：alpha = dt / (dt + RC) */
    lpf_alpha = time_step / (time_step + rc);

    return lpf_alpha;
}

/**
 * @brief 设置巴特沃斯滤波器截止频率
 * @param sample_frequent 采样频率（Hz）
 * @param cutoff_frequent 截止频率（Hz）
 * @param LPF 滤波器参数结构体指针
 *
 * 功能：根据采样频率和截止频率计算2阶巴特沃斯滤波器的系数
 *
 * 设计原理：
 * 1. 将模拟滤波器转换为数字滤波器（双线性变换）
 * 2. 2阶巴特沃斯模拟滤波器的传递函数：
 *    H(s) = 1/(s^2 + sqrt(2)*s + 1)
 * 3. 经过双线性变换得到数字滤波器系数
 *
 * 系数说明：
 * b[0], b[1], b[2]: 分子系数（前向路径）
 * a[0], a[1], a[2]: 分母系数（反馈路径），a[0]始终为1
 */
void set_cutoff_frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF)
{
    float fr;    // 归一化频率：采样频率/截止频率
    float ohm;   // 预扭曲频率：tan(pi/(2*fr))
    float c;     // 归一化系数

    /* 计算归一化频率 */
    fr = sample_frequent / cutoff_frequent;

    /* 预扭曲频率：双线性变换需要的频率预扭曲 */
    ohm = tanf(M_PI_F / fr);

    /* 计算归一化系数 */
    c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

    /* 检查截止频率是否有效 */
    if (cutoff_frequent <= 0.0f) {
        return;  // 无效截止频率，不设置系数
    }

    /* 计算数字滤波器系数（双线性变换结果） */
    LPF->b[0] = ohm * ohm / c;                     // b0
    LPF->b[1] = 2.0f * LPF->b[0];                  // b1
    LPF->b[2] = LPF->b[0];                         // b2

    LPF->a[0] = 1.0f;                              // a0（始终为1）
    LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;     // a1
    LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;  // a2
}

/**
 * @brief 最速下降法滤波器（用于寻找数据序列的最佳匹配点）
 * @param arr[] 输入数据数组（环形缓冲区）
 * @param len 数组长度
 * @param steepest 最速下降法状态结构体指针
 * @param step_num 搜索步数
 * @param in 当前输入值
 *
 * 功能：使用最速下降法在数据序列中找到最佳输出点
 *
 * 算法原理：
 * 1. 将当前输入存入环形缓冲区
 * 2. 计算从上次输出点到当前点的梯度（斜率）
 * 3. 沿着梯度方向搜索，找到使平方和最小的点
 * 4. 更新输出值和速度估计
 *
 * 应用场景：用于信号平滑、趋势提取、噪声抑制
 * 特点：可以处理非平稳信号，自适应调整
 */
void steepest_descend(int32 arr[], uint8 len, _steepest_st *steepest, uint8 step_num, int32 in)
{
    uint8 updw = 1;          // 方向标志：0=下降，1=上升
    int16 i;                 // 循环计数器
    uint8 step_cnt = 0;      // 步数计数器
    uint8 step_slope_factor = 1;  // 步长缩放因子
    uint8 on = 1;            // 循环控制标志
    int8 pn = 1;             // 搜索方向：1=正向，-1=反向
    float step = 0;          // 梯度步长
    int32 start_point = 0;   // 搜索起点
    int32 pow_sum = 0;       // 平方和

    /* 1. 保存上一次的输出值 */
    steepest->lst_out = steepest->now_out;

    /* 2. 更新环形缓冲区索引 */
    if (++(steepest->cnt) >= len)
    {
        (steepest->cnt) = 0;  // 环形缓冲，回到起点
    }

    /* 3. 将当前输入存入缓冲区 */
    arr[(steepest->cnt)] = in;

    /* 4. 计算梯度：当前点与上次输出的差值除以步数 */
    step = (float)(in - steepest->lst_out) / step_num;

    /* 5. 梯度阈值处理：过小的梯度视为零 */
    if (ABS(step) < 1)  // 对于整数数据，小于1的梯度可能无意义
    {
        if (ABS(step) * step_num < 2)  // 总变化小于2
        {
            step = 0;  // 梯度为零
        }
        else
        {
            step = (step > 0) ? 1.0f : -1.0f;  // 归一化为±1
        }
    }

    /* 6. 初始化搜索起点为上次输出 */
    start_point = steepest->lst_out;

    /* 7. 最速下降搜索循环 */
    do
    {
        /* 7.1 计算从当前起点到所有数据点的平方和 */
        pow_sum = 0;
        for (i = 0; i < len; i++)
        {
            /* 计算差值平方并累加 */
            pow_sum += my_pow(arr[i] - start_point);
        }

        /* 7.2 判断平方和变化趋势 */
        if (pow_sum - steepest->lst_pow_sum > 0)
        {
            /* 平方和增加：说明当前方向错误 */
            if (updw == 0)  // 如果之前是下降趋势
            {
                on = 0;  // 停止搜索
            }
            updw = 1;  // 标记为上升趋势
            pn = (pn == 1) ? -1 : 1;  // 反转搜索方向
        }
        else
        {
            /* 平方和减小：说明当前方向正确 */
            updw = 0;  // 标记为下降趋势
            if (step_slope_factor < step_num)  // 增加步长缩放因子
            {
                step_slope_factor++;
            }
        }

        /* 7.3 保存当前平方和 */
        steepest->lst_pow_sum = pow_sum;

        /* 7.4 沿搜索方向移动一步 */
        start_point += pn * step;

        /* 7.5 检查是否达到最大搜索步数 */
        if (++step_cnt > step_num)  // 超过最大步数
        {
            on = 0;  // 停止搜索
        }

        /* 7.6 可选：限制下降次数以节省计算时间（增加滞后） */
        if (step_slope_factor >= 2)  // 只允许一次下降
        {
            on = 0;  // 停止搜索
        }
    } while (on == 1);  // 继续搜索直到满足停止条件

    /* 8. 更新输出值和速度估计 */
    steepest->now_out = start_point;  // 当前输出值
    // 可选：输出值取起点和终点的平均值，增加平滑性
    // steepest->now_out = 0.5f * (start_point + steepest->lst_out);

    /* 9. 计算速度估计（输出变化率） */
    steepest->now_velocity_xdt = steepest->now_out - steepest->lst_out;
}

/**
 * @brief 移动平均滤波器
 * @param moavarray[] 移动平均数组（环形缓冲区）
 * @param len 数组长度（窗口大小）
 * @param fil_cnt 滤波器计数器指针
 * @param in 当前输入值
 * @param out 滤波输出值指针
 *
 * 功能：实现滑动窗口平均滤波
 *
 * 算法原理：
 * 1. 维护一个长度为len的环形缓冲区
 * 2. 每次新数据到来时，替换最旧的数据
 * 3. 输出是所有数据的平均值
 *
 * 高效实现技巧：
 * 不重新计算所有数据和，而是：
 * 新平均值 = 旧平均值 + (新数据 - 最旧数据)/窗口大小
 *
 * 特点：计算效率高，适合实时处理
 */
void Moving_Average(float moavarray[], uint16 len, uint16 *fil_cnt, float in, float *out)
{
    uint16 width_num;  // 窗口大小
    float last;        // 被替换的最旧数据值

    width_num = len;  // 窗口大小

    /* 1. 更新环形缓冲区索引 */
    if (++(*fil_cnt) >= width_num)
    {
        *fil_cnt = 0;  // 环形缓冲，回到起点
    }

    /* 2. 保存将被替换的最旧数据 */
    last = moavarray[*fil_cnt];

    /* 3. 将新数据存入缓冲区 */
    moavarray[*fil_cnt] = in;

    /* 4. 高效更新平均值：
     * 新平均值 = 旧平均值 + (新数据 - 最旧数据) / 窗口大小 */
    *out += (in - last) / (float)(width_num);

    /* 5. 数值稳定性修正：防止累积误差
     * 添加微小修正项，限制修正幅度避免过冲 */
    // *out += 0.00001f * (in - *out);  // 基础版本
    *out += 0.00001f * LIMIT((in - *out), -1, 1);  // 限幅版本，更稳定
}

/**
 * @brief 一阶低通滤波器
 * @param hz 截止频率（Hz）
 * @param time 时间常数相关参数
 * @param in 输入值
 * @param out 输出值指针（会被更新）
 *
 * 功能：实现标准一阶低通滤波器
 *
 * 滤波方程：
 * out[n] = out[n-1] + alpha * (in[n] - out[n-1])
 *
 * alpha计算公式：
 * alpha = 1 / (1 + 1/(2*pi*f*t))
 *
 * 其中：f=hz（截止频率），t=time（时间常数）
 */
void LPF_1(float hz, float time, float in, float *out)
{
    /* 计算alpha系数并更新输出 */
    *out += (1 / (1 + 1 / (hz * 6.28f * time))) * (in - *out);
}

/**
 * @brief 限幅滤波器
 * @param T 时间常数
 * @param hz 截止频率
 * @param data 滤波器状态结构体指针
 * @param in 输入值
 *
 * 功能：将输入限制在滤波后的绝对值范围内
 *
 * 处理流程：
 * 1. 对输入进行一阶低通滤波
 * 2. 取滤波输出的绝对值作为限制边界
 * 3. 将原始输入限制在[-abs, abs]范围内
 *
 * 应用场景：用于限制信号幅度，同时保持平滑性
 */
void limit_filter(float T, float hz, _lf_t *data, float in)
{
    float abs_t;  // 限制边界（绝对值）

    /* 1. 对输入进行一阶低通滤波 */
    LPF_1(hz, T, in, &(data->lpf_1));

    /* 2. 计算滤波输出的绝对值作为限制边界 */
    abs_t = ABS(data->lpf_1);

    /* 3. 将原始输入限制在[-abs_t, abs_t]范围内 */
    data->out = LIMIT(in, -abs_t, abs_t);
}

/**
 * @brief 固定积分滤波器（带修正的积分器）
 * @param dT 时间步长
 * @param data 滤波器状态结构体指针
 *
 * 功能：实现带误差修正的积分滤波器
 *
 * 方程：
 * 输出 = 输出 + (估计输入变化率 + 误差修正) * dT
 * 误差 = Kp * (观测值 - 输出)
 *
 * 相当于：积分器 + 比例反馈修正
 *
 * 应用场景：速度/位置估计，带修正的积分
 */
void fix_inte_filter(float dT, _fix_inte_filter_st *data)
{
    /* 1. 积分更新：输出 = 输出 + (估计变化率 + 误差) * 时间步长 */
    data->out += (data->in_est_d + data->e) * dT;

    /* 2. 计算误差：误差 = Kp * (观测值 - 当前输出) */
    data->e = data->fix_kp * (data->in_obs - data->out);

    /* 3. 误差限幅（如果启用） */
    if (data->e_limit > 0)
    {
        data->e = LIMIT(data->e, -data->e_limit, data->e_limit);
    }
}



/**********************************************************************************************************
*函 数 名: LowPassFilter1st
*功能说明: 一阶低通滤波器
*形    参: data - 指向历史数据的指针, newData - 指向新数据的指针, coff - 滤波系数(0~1)
*返 回 值: 滤波后的值
**********************************************************************************************************/
float LowPassFilter1st(const float* data, const float* newData, const float coff)
{
    float old_data = *data;    // 获取历史值
    float new_data = *newData; // 获取新值

    // 一阶低通滤波器差分方程：
    // y[k] = (1-α)*y[k-1] + α*x[k]
    // 其中α=coff，0<α<1，α越大对新数据响应越快
    return old_data * (1 - coff) + new_data * coff;
}

/**********************************************************************************************************
*函 数 名: Sliding_weighted_filter
*功能说明: 滑动加权滤波算法，越新的数据权重越大
*形    参: array - 数据数组, array_num - 数组大小, data - 新数据
*返 回 值: 滤波后的值
**********************************************************************************************************/
float Sliding_weighted_filter(float* array, uint8_t array_num, float data)
{
    long array_sum = 0; // 采样队列加权和

    // 滑动窗口：将数组元素向前移动一位
    for(int i=1; i<array_num; i++)
    {
        array[i-1] = array[i];  // 将第i个元素移到i-1位置

        // 计算加权和：第i个元素的权重为i
        array_sum += array[i] * i;
    }

    // 将新数据放入数组末尾
    array[array_num-1] = data;

    // 新数据的权重最大，为array_num
    array_sum += data * array_num;

    // 计算加权平均值
    // 总权重 = 1+2+...+array_num = array_num*(array_num+1)/2
    float filte_value = array_sum / ((array_num+1)*array_num*0.5f);

    return filte_value;
}

/**
 * @brief   计算二阶巴特沃斯低通滤波器系数（双线性变换法）
 * @param   coeff: 滤波器系数结构体指针
 * @param   fs: 采样频率 (Hz)
 * @param   fc: 截止频率 (Hz)，当fc<=0时滤波器被禁用
 * @note    使用双线性变换法将模拟滤波器转换为数字滤波器
 *          二阶巴特沃斯滤波器的模拟原型：H(s) = 1/(s² + √2·s + 1)
 * @retval  void
 */
void cal_irr_coeff(IIR_cceff_Typedef* coeff, float fs, float fc)
{
    float fr = 0;   ///< 频率比
    float ohm = 0;  ///< 预扭曲后的模拟频率
    float c = 0;    ///< 归一化系数

    // 检查截止频率是否有效
    if (fc <= 0.0f)
    {
        // 无效的截止频率，不进行滤波
        // 注意：这里没有设置coeff->fc，可能导致后续判断错误
        return;
    }

    // 计算频率比
    fr = fs / fc;               // fr = 采样频率 / 截止频率

    // 双线性变换的频率预扭曲
    ohm = tanf(PI / fr);    // ω' = tan(π * fc / fs) = tan(π / fr)

    // 计算归一化系数
    c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;

    // 保存截止频率（用于判断滤波器是否启用）
    coeff->fc = fc;

    // 计算数字滤波器系数（双线性变换结果）
    // 分子系数
    coeff->b0 = ohm * ohm / c;
    coeff->b1 = 2.0f * coeff->b0;
    coeff->b2 = coeff->b0;

    // 分母系数
    coeff->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    coeff->a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;


}

/**
 * @brief   IIR滤波器处理函数（直接II型实现）
 * @param   coeff: 滤波器系数与状态结构体指针
 * @param   data: 输入采样数据
 * @note    实现差分方程：y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *          采用直接II型（正准型）结构
 * @retval  滤波后的输出数据
 */
float get_iir_output(IIR_cceff_Typedef* coeff, float data)
{
    // 检查滤波器是否启用
    if (coeff->fc <= 0.0f)
    {
        // 滤波器未启用，直接返回原始数据
        return data;
    }

    // 直接II型结构计算
    // 中间变量 w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
    // 这里 y_1 存储 w[n-1]，y_2 存储 w[n-2]
    float w_n = data - coeff->y_1 * coeff->a1 - coeff->y_2 * coeff->a2;

    // 输出 y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
    float output = w_n * coeff->b0 + coeff->y_1 * coeff->b1 + coeff->y_2 * coeff->b2;

    // 更新延迟单元状态
    coeff->y_2 = coeff->y_1;  // w[n-2] = w[n-1]
    coeff->y_1 = w_n;         // w[n-1] = w[n]

    return output;
}
