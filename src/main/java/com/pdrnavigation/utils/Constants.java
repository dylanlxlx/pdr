package com.pdrnavigation.utils;

/**
 * PDR 系统中使用的常量
 */
public class Constants {
    // 转换因子
    /** 度到弧度的转换因子 */
    public static final double DEG_TO_RAD = Math.PI / 180.0;

    /** 弧度到度的转换因子 */
    public static final double RAD_TO_DEG = 180.0 / Math.PI;

    // 物理常量
    /** 标准重力 (m/s²) */
    public static final double GRAVITY = 9.81;

    // 步态检测参数
    /** 默认步态检测阈值 */
    public static final double DEFAULT_STEP_THRESHOLD = 0.5;

    /** 默认步长常数 K */
    public static final double DEFAULT_STEP_LENGTH_K = 0.5;

    /** 默认 FIR 滤波器阶数 */
    public static final int DEFAULT_FILTER_ORDER = 10;

    /** 默认 FIR 滤波器截止频率（归一化到 0-1） */
    public static final double DEFAULT_CUTOFF_FREQ = 0.06;

    // 航向估计参数
    /** 默认角度检测阈值 (deg/s) */
    public static final double DEFAULT_CORNER_THRESHOLD = 1.0;

    /** 默认航向平滑阈值 (度) */
    public static final double DEFAULT_HEADING_THRESHOLD = 5.0;

    // 默认采样率
    /** 默认采样率 (Hz) */
    public static final double DEFAULT_SAMPLING_RATE = 50.0;

    /** 默认时间步长 (秒) */
    public static final double DEFAULT_TIMESTEP = 1.0 / DEFAULT_SAMPLING_RATE;

    // PDR 参数
    /** 最小有效步长 (m) */
    public static final double MIN_STEP_LENGTH = 0.3;

    /** 最大有效步长 (m) */
    public static final double MAX_STEP_LENGTH = 1.0;

    // 粒子滤波器参数
    /** 默认粒子数量 */
    public static final int DEFAULT_PARTICLE_COUNT = 100;

    /** 默认初始粒子噪声 */
    public static final double DEFAULT_INITIAL_NOISE = 0.5;

    // 私有构造函数，防止实例化
    private Constants() {
        throw new UnsupportedOperationException("Utility classes should not be instantiated");
    }
}
