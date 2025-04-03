package com.pdrnavigation.orientation;

import com.pdrnavigation.filters.KalmanFilter;
import com.pdrnavigation.math.MathUtils;

/**
 * PDR 的航向估计器
 * 实现了 map_aid.m 和 yaw_kal.m 中的地图辅助航向估计
 */
public class HeadingEstimator {
    private KalmanFilter kalmanFilter;
    private CornerDetector cornerDetector;
    private double[] headingAngles;
    private double[] filteredHeading;
    private double degToRad;

    /**
     * 创建一个新的航向估计器
     * @param threshold 角度检测阈值
     */
    public HeadingEstimator(double threshold) {
        this.cornerDetector = new CornerDetector(threshold);
        this.degToRad = Math.PI / 180.0;

        // 初始化用于航向估计的卡尔曼滤波器
        // 2 状态滤波器：[航向, 角速度]
        kalmanFilter = new KalmanFilter(2, 2);

        // 设置状态转移矩阵 (A)
        double[][] A = {
                {1, 1}, // heading_{k+1} = heading_k + angular_velocity_k
                {0, 1}  // angular_velocity_{k+1} = angular_velocity_k
        };
        kalmanFilter.setStateTransitionMatrix(A);

        // 设置测量矩阵 (H)
        double[][] H = {
                {1, 0}, // 直接测量航向
                {0, 1}  // 直接测量角速度
        };
        kalmanFilter.setMeasurementMatrix(H);

        // 设置初始过程噪声协方差 (Q)
        double[][] Q = {
                {0.0025, 0},
                {0, 0.01}
        };
        kalmanFilter.setProcessNoiseCovariance(Q);
    }

    /**
     * 使用地图辅助卡尔曼滤波估计航向
     * @param rawHeading 设备提供的原始航向角（单位：度）
     * @param gyroZ 陀螺仪 Z 轴数据（绕 Z 轴的角速度）
     * @return 过滤后的航向角（单位：弧度）
     */
    public double[] estimateHeading(double[] rawHeading, double[] gyroZ) {
        int length = rawHeading.length;
        headingAngles = new double[length];
        filteredHeading = new double[length];

        // 展开航向角以避免在 ±180 度处的跳跃
        headingAngles[0] = rawHeading[0];
        for (int i = 1; i < length; i++) {
            double diff = rawHeading[i] - rawHeading[i - 1];
            if (diff < -180) {
                headingAngles[i] = headingAngles[i - 1] + (diff + 360);
            } else if (diff > 180) {
                headingAngles[i] = headingAngles[i - 1] + (diff - 360);
            } else {
                headingAngles[i] = headingAngles[i - 1] + diff;
            }
        }

        // 检测角度
        int[] cornerIndices = cornerDetector.detectCorners(gyroZ);

        if (cornerIndices.length < 2) {
            // 未检测到角度，直接使用原始航向
            for (int i = 0; i < length; i++) {
                filteredHeading[i] = headingAngles[i] * degToRad;
            }
            return filteredHeading;
        }

        // 第一段（第一个角度之前）
        double referenceHeading = 128.5; // 默认第一段航向（来自 MATLAB 代码）
        kalmanFilter.setState(new double[] {referenceHeading, gyroZ[0]});

        // 根据陀螺仪方差设置测量噪声
        double gyroVariance = calculateVariance(gyroZ);
        double[][] R = {
                {1, 0},
                {0, Math.sqrt(gyroVariance)}
        };
        kalmanFilter.setMeasurementNoiseCovariance(R);

        // 对每个段应用卡尔曼滤波
        int currentSegment = 0;
        for (int i = 0; i < length; i++) {
            // 检查是否到达新的角度
            if (currentSegment + 1 < cornerIndices.length && i == cornerIndices[currentSegment + 1]) {
                currentSegment++;

                // 更新当前段的参考航向
                if (currentSegment == 1) {
                    referenceHeading = 90.0; // 第二段航向（来自 MATLAB 代码）
                } else if (currentSegment >= 2) {
                    referenceHeading = 0.0;  // 第三段及之后的航向
                }

                // 重置卡尔曼滤波器状态以适应新段
                kalmanFilter.setState(new double[] {referenceHeading, gyroZ[i]});
            }

            // 如果在角度转换期间，使用原始航向
            if (isCornerTransition(i, cornerIndices)) {
                filteredHeading[i] = headingAngles[i] * degToRad;
                continue;
            }

            // 应用卡尔曼滤波
            kalmanFilter.predict();
            kalmanFilter.update(new double[] {referenceHeading, gyroZ[i]});

            double[] state = kalmanFilter.getState();
            filteredHeading[i] = state[0] * degToRad;
        }

        return filteredHeading;
    }

    /**
     * 检查是否在角度转换期间
     * @param index 当前索引
     * @param cornerIndices 角度索引数组
     * @return 如果在转换期间，返回 true；否则返回 false
     */
    private boolean isCornerTransition(int index, int[] cornerIndices) {
        for (int i = 0; i < cornerIndices.length - 1; i += 2) {
            if (index >= cornerIndices[i] && index <= cornerIndices[i + 1]) {
                return true;
            }
        }
        return false;
    }

    /**
     * 计算数组的方差
     * @param data 输入数据
     * @return 方差
     */
    private double calculateVariance(double[] data) {
        double mean = 0;
        for (double value : data) {
            mean += value;
        }
        mean /= data.length;

        double variance = 0;
        for (double value : data) {
            variance += (value - mean) * (value - mean);
        }
        variance /= data.length;

        return variance;
    }

    /**
     * 获取过滤后的航向角
     * @return 过滤后的航向角（单位：弧度）
     */
    public double[] getFilteredHeading() {
        return filteredHeading;
    }

    /**
     * 获取展开的航向角
     * @return 展开的航向角（单位：度）
     */
    public double[] getUnwrappedHeading() {
        return headingAngles;
    }
}
