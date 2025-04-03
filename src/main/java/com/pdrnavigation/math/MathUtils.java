package com.pdrnavigation.math;

/**
 * PDR 算法的基本数学工具类
 */
public class MathUtils {
    /** 度到弧度的转换常量 */
    public static final double DEG_TO_RAD = Math.PI / 180.0;

    /** 弧度到度的转换常量 */
    public static final double RAD_TO_DEG = 180.0 / Math.PI;

    /** 默认重力值 (m/s^2) */
    public static final double GRAVITY = 9.81;

    /**
     * 将角度归一化到范围 [-π, π]
     * @param angle 角度（弧度）
     * @return 归一化后的角度，在范围 [-π, π] 内
     */
    public static double wrapToPI(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * 将角度归一化到范围 [0, 2π]
     * @param angle 角度（弧度）
     * @return 归一化后的角度，在范围 [0, 2π] 内
     */
    public static double wrapTo2PI(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * 将角度归一化到范围 [-180, 180]
     * @param angle 角度（度）
     * @return 归一化后的角度，在范围 [-180, 180] 内
     */
    public static double wrapTo180(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    /**
     * 将角度归一化到范围 [0, 360]
     * @param angle 角度（度）
     * @return 归一化后的角度，在范围 [0, 360] 内
     */
    public static double wrapTo360(double angle) {
        angle = angle % 360.0;
        if (angle < 0) {
            angle += 360.0;
        }
        return angle;
    }

    /**
     * 计算数组的中值
     * @param values 值数组
     * @return 中值
     */
    public static double median(double[] values) {
        if (values == null || values.length == 0) {
            throw new IllegalArgumentException("输入数组不能为空或为空");
        }

        // 创建数组的副本以避免修改原始数组
        double[] sortedValues = values.clone();
        java.util.Arrays.sort(sortedValues);

        int length = sortedValues.length;
        if (length % 2 == 0) {
            // 偶数个元素
            return (sortedValues[length / 2 - 1] + sortedValues[length / 2]) / 2.0;
        } else {
            // 奇数个元素
            return sortedValues[length / 2];
        }
    }

    /**
     * 计算向量的模（欧几里得范数）
     * @param vector 向量分量数组
     * @return 向量的模
     */
    public static double magnitude(double[] vector) {
        double sumOfSquares = 0.0;
        for (double v : vector) {
            sumOfSquares += v * v;
        }
        return Math.sqrt(sumOfSquares);
    }

    /**
     * 计算三维向量的模（欧几里得范数）
     * @param x X 分量
     * @param y Y 分量
     * @param z Z 分量
     * @return 向量的模
     */
    public static double magnitude(double x, double y, double z) {
        return Math.sqrt(x * x + y * y + z * z);
    }
}
