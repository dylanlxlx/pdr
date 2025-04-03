package com.pdrnavigation.steps;

import com.pdrnavigation.filters.FIRFilter;
import com.pdrnavigation.utils.Vector3D;

/**
 * PDR 的步态检测器类
 * 实现了 IMU_firls.m 中与步态检测相关的内容
 */
public class StepDetector {
    private FIRFilter lowPassFilter;
    private PeakValleyDetector peakValleyDetector;
    private double threshold;
    private double[][] peakValleyFlags;
    private double[] filteredData;

    /**
     * 创建一个新的步态检测器
     * @param filterOrder 低通滤波器的阶数
     * @param cutoffFreq 归一化截止频率（0-1）
     * @param threshold 峰值检测阈值
     */
    public StepDetector(int filterOrder, double cutoffFreq, double threshold) {
        this.lowPassFilter = FIRFilter.createLowPass(filterOrder, cutoffFreq);
        this.peakValleyDetector = new PeakValleyDetector(threshold);
        this.threshold = threshold;
    }

    /**
     * 处理加速度计数据以检测步数
     * @param accelerometerData 机体坐标系中的加速度计数据数组
     * @param gravity 机体坐标系中的重力向量
     * @return 检测到的步数
     */
    public int detectSteps(Vector3D[] accelerometerData, Vector3D[] gravity) {
        // 从加速度计数据中去除重力
        Vector3D[] linearAcceleration = new Vector3D[accelerometerData.length];
        for (int i = 0; i < accelerometerData.length; i++) {
            linearAcceleration[i] = new Vector3D(accelerometerData[i]);
            linearAcceleration[i].subtract(gravity[i]);
        }

        // 计算线性加速度的大小
        double[] magnitudes = new double[accelerometerData.length];
        for (int i = 0; i < accelerometerData.length; i++) {
            magnitudes[i] = linearAcceleration[i].magnitude();
        }

        // 应用低通滤波器
        filteredData = lowPassFilter.process(magnitudes);

        // 检测峰值和谷值
        peakValleyDetector.process(filteredData);
        peakValleyFlags = peakValleyDetector.getPeakValleyFlags(filteredData.length);

        return peakValleyDetector.getStepCount();
    }

    /**
     * 处理加速度计大小数据以检测步数
     * @param accelerometerMagnitudes 加速度计大小数组
     * @return 检测到的步数
     */
    public int detectSteps(double[] accelerometerMagnitudes) {
        // 应用低通滤波器
        filteredData = lowPassFilter.process(accelerometerMagnitudes);

        // 检测峰值和谷值
        peakValleyDetector.process(filteredData);
        peakValleyFlags = peakValleyDetector.getPeakValleyFlags(filteredData.length);

        return peakValleyDetector.getStepCount();
    }

    /**
     * 获取检测到的峰值位置
     * @return 峰值位置列表
     */
    public java.util.List<Integer> getPeakPositions() {
        return peakValleyDetector.getPeakPositions();
    }

    /**
     * 获取检测到的谷值位置
     * @return 谷值位置列表
     */
    public java.util.List<Integer> getValleyPositions() {
        return peakValleyDetector.getValleyPositions();
    }

    /**
     * 获取峰值和谷值标记
     * @return 标记峰值和谷值的 2D 数组
     */
    public double[][] getPeakValleyFlags() {
        return peakValleyFlags;
    }

    /**
     * 获取滤波后的数据
     * @return 滤波后的加速度计数据
     */
    public double[] getFilteredData() {
        return filteredData;
    }
}
