package com.pdrnavigation.orientation;

import java.util.ArrayList;
import java.util.List;

/**
 * 角度检测器，用于根据陀螺仪数据识别转弯
 * 实现了 corner_detect.m 中的功能
 */
public class CornerDetector {
    private double threshold;

    /**
     * 创建一个新的角度检测器
     * @param threshold 角速度检测阈值
     */
    public CornerDetector(double threshold) {
        this.threshold = threshold;
    }

    /**
     * 从陀螺仪 Z 轴数据检测角度（转弯）
     * @param gyroZData 陀螺仪 Z 轴数据（绕 Z 轴的角速度）
     * @return 检测到角度的索引数组
     */
    public int[] detectCorners(double[] gyroZData) {
        List<Integer> allCornerIndices = new ArrayList<>();
        List<Integer> significantCornerIndices = new ArrayList<>();

        // 找到所有陀螺仪数据超过阈值的索引
        for (int i = 0; i < gyroZData.length; i++) {
            if (Math.abs(gyroZData[i]) > threshold) {
                allCornerIndices.add(i);
            }
        }

        if (allCornerIndices.isEmpty()) {
            return new int[0];
        }

        // 添加第一个角度
        significantCornerIndices.add(allCornerIndices.get(0));

        // 找到显著的角度（间隔超过 1 个采样点）
        for (int i = 1; i < allCornerIndices.size(); i++) {
            if (allCornerIndices.get(i) - allCornerIndices.get(i - 1) > 1) {
                significantCornerIndices.add(allCornerIndices.get(i - 1));
                significantCornerIndices.add(allCornerIndices.get(i));
            }
        }

        // 如果最后一个角度未添加，则添加最后一个角度
        if (!significantCornerIndices.contains(allCornerIndices.get(allCornerIndices.size() - 1))) {
            significantCornerIndices.add(allCornerIndices.get(allCornerIndices.size() - 1));
        }

        // 将列表转换为数组
        int[] result = new int[significantCornerIndices.size()];
        for (int i = 0; i < significantCornerIndices.size(); i++) {
            result[i] = significantCornerIndices.get(i);
        }

        return result;
    }

    /**
     * 设置检测阈值
     * @param threshold 新的检测阈值
     */
    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    /**
     * 获取当前检测阈值
     * @return 当前检测阈值
     */
    public double getThreshold() {
        return threshold;
    }
}
