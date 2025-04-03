package com.pdrnavigation.steps;

import java.util.ArrayList;
import java.util.List;

/**
 * 步长估计器类
 * 实现了 IMU_firls_stp_len.m 中的功能
 */
public class StepLength {
    public enum Method {
        WEINBERG,   // Weinberg 方法：K * (a_max - a_min)^0.25
        SCARLET,    // Scarlet 方法：K * (a_avg - a_min)/(a_max - a_min)
        KIM         // Kim 方法：K * a_avg^(1/3)
    }

    private double kConstant;
    private Method method;

    /**
     * 使用给定的 K 常数和方法创建一个新的步长估计器
     * @param kConstant 步长估计的 K 常数
     * @param method 用于步长估计的方法
     */
    public StepLength(double kConstant, Method method) {
        this.kConstant = kConstant;
        this.method = method;
    }

    /**
     * 基于峰值和谷值标记以及加速度数据估计步长
     * @param peakValleyFlags 来自 PeakValleyDetector 的峰值和谷值标记数组
     * @param accData 加速度计大小数据
     * @return 估计的步长数组
     */
    public double[] estimateStepLengths(double[][] peakValleyFlags, double[] accData) {
        // 找到峰值和谷值的位置
        List<Integer> peakPositions = new ArrayList<>();
        List<Integer> valleyPositions = new ArrayList<>();

        for (int i = 0; i < peakValleyFlags.length; i++) {
            if (peakValleyFlags[i][0] != 0) {
                peakPositions.add(i);
            }
            if (peakValleyFlags[i][1] != 0) {
                valleyPositions.add(i);
            }
        }

        int stepCount = Math.min(peakPositions.size(), valleyPositions.size());
        double[] stepLengths = new double[stepCount];
        double[] peakValues = new double[stepCount];
        double[] valleyValues = new double[stepCount];
        double[] averageAccs = new double[stepCount];

        // 计算 Weinberg 方法的平均峰值值
        double sumPeaks = 0;
        for (int i = 0; i < stepCount; i++) {
            sumPeaks += peakValleyFlags[peakPositions.get(i)][0];
        }
        double averagePeakValue = sumPeaks / stepCount;

        // 根据平均峰值值调整 K 常数
        // K = 0.087 * averagePeakValue + 0.5（来自 MATLAB 代码）
        double adjustedK = 0.087 * averagePeakValue + 0.5;

        // 使用不同的方法计算步长
        for (int i = 0; i < stepCount; i++) {
            int peakPos = peakPositions.get(i);
            int valleyPos = valleyPositions.get(i);

            // 确保峰值和谷值的顺序正确
            if (peakPos < valleyPos) {
                int temp = peakPos;
                peakPos = valleyPos;
                valleyPos = temp;
            }

            peakValues[i] = peakValleyFlags[peakPos][0];
            valleyValues[i] = Math.abs(peakValleyFlags[valleyPos][1]); // 使值为正用于计算

            // 计算谷值和峰值之间的平均加速度
            double sum = 0;
            int count = 0;
            for (int j = valleyPos; j <= peakPos; j++) {
                sum += accData[j];
                count++;
            }
            averageAccs[i] = sum / count;

            // 根据选定的方法计算步长
            double diffAcc = Math.abs(peakValues[i] - valleyValues[i]);

            switch (method) {
                case WEINBERG:
                    // Weinberg 方法：K * (a_max - a_min)^0.25
                    stepLengths[i] = kConstant * Math.pow(diffAcc - 1.5, 0.25);
                    break;
                case SCARLET:
                    // Scarlet 方法：K * (a_avg - a_min)/(a_max - a_min)
                    stepLengths[i] = kConstant * (averageAccs[i] - valleyValues[i]) / diffAcc;
                    break;
                case KIM:
                    // Kim 方法：K * a_avg^(1/3)
                    stepLengths[i] = kConstant * Math.pow(averageAccs[i], 1.0/3.0);
                    break;
                default:
                    // 默认使用 Weinberg 方法
                    stepLengths[i] = kConstant * Math.pow(diffAcc - 1.5, 0.25);
            }

            // 确保步长为正且在合理范围内
            if (stepLengths[i] < 0.3) {
                stepLengths[i] = 0.3; // 最小步长（典型人类步长）
            } else if (stepLengths[i] > 1.0) {
                stepLengths[i] = 1.0; // 最大步长（典型人类步长）
            }
        }

        return stepLengths;
    }

    /**
     * 计算行走的总距离
     * @param stepLengths 步长数组
     * @return 行走的总距离
     */
    public double calculateTotalDistance(double[] stepLengths) {
        double distance = 0;

        for (double stepLength : stepLengths) {
            distance += stepLength;
        }

        return distance;
    }

    /**
     * 设置 K 常数
     * @param kConstant 新的 K 常数
     */
    public void setKConstant(double kConstant) {
        this.kConstant = kConstant;
    }

    /**
     * 获取当前的 K 常数
     * @return 当前的 K 常数
     */
    public double getKConstant() {
        return kConstant;
    }

    /**
     * 设置估计方法
     * @param method 新的估计方法
     */
    public void setMethod(Method method) {
        this.method = method;
    }

    /**
     * 获取当前的估计方法
     * @return 当前的估计方法
     */
    public Method getMethod() {
        return method;
    }
}
