package com.pdrnavigation.steps;

import java.util.ArrayList;
import java.util.List;

/**
 * 峰值和谷值检测器，用于步态检测
 * 实现了 IMU_firls_stp_num.m 中的功能
 */
public class PeakValleyDetector {
    private double threshold;
    private List<Integer> peakPositions;
    private List<Double> peakValues;
    private List<Integer> valleyPositions;
    private List<Double> valleyValues;

    /**
     * 使用给定的阈值创建一个新的峰值和谷值检测器
     * @param threshold 检测阈值幅度
     */
    public PeakValleyDetector(double threshold) {
        this.threshold = threshold;
        this.peakPositions = new ArrayList<>();
        this.peakValues = new ArrayList<>();
        this.valleyPositions = new ArrayList<>();
        this.valleyValues = new ArrayList<>();
    }

    /**
     * 处理数据数组以检测峰值和谷值
     * @param data 输入信号
     */
    public void process(double[] data) {
        if (data.length < 3) {
            return; // 至少需要 3 个点来检测峰值和谷值
        }

        // 清除之前的检测结果
        peakPositions.clear();
        peakValues.clear();
        valleyPositions.clear();
        valleyValues.clear();

        // 计算导数
        double[] derivative = new double[data.length - 1];
        for (int i = 0; i < derivative.length; i++) {
            derivative[i] = data[i + 1] - data[i];
        }

        int peakNum = 0;
        int valleyNum = 0;
        int flag = 0; // 标志位，用于跟踪检测状态

        // 处理每个点以查找峰值和谷值
        for (int i = 1; i < derivative.length; i++) {
            // 峰值检测（导数从正变负且值大于阈值）
            if (derivative[i - 1] > 0 && derivative[i] < 0 && data[i] > threshold) {
                if (peakNum == 0 && valleyNum == 0) {
                    // 第一个峰值
                    peakPositions.add(i);
                    peakValues.add(data[i]);
                    peakNum++;
                    flag = 1;
                } else if ((flag == 1 && (peakNum - valleyNum) > 1) ||
                        (flag == -1 && (peakNum - valleyNum) >= 1)) {
                    // 与前一个峰值比较
                    double accDiff = data[i] - peakValues.get(peakNum - 1);
                    if (accDiff > 0) {
                        // 新峰值更高，替换前一个峰值
                        peakPositions.set(peakNum - 1, i);
                        peakValues.set(peakNum - 1, data[i]);
                    }
                    // 如果新峰值更低，忽略它
                } else {
                    // 正常峰值
                    peakPositions.add(i);
                    peakValues.add(data[i]);
                    peakNum++;
                }
            }

            // 谷值检测（导数从负变正且值小于 -阈值）
            if (derivative[i - 1] < 0 && derivative[i] > 0 && data[i] < -threshold) {
                if (valleyNum == 0 && peakNum == 0) {
                    // 第一个谷值
                    valleyPositions.add(i);
                    valleyValues.add(data[i]);
                    valleyNum++;
                    flag = -1;
                } else if ((flag == 1 && (peakNum - valleyNum) <= -1) ||
                        (flag == -1 && (peakNum - valleyNum) < -1)) {
                    // 与前一个谷值比较
                    double accDiffVy = data[i] - valleyValues.get(valleyNum - 1);
                    if (accDiffVy < 0) {
                        // 新谷值更低，替换前一个谷值
                        valleyPositions.set(valleyNum - 1, i);
                        valleyValues.set(valleyNum - 1, data[i]);
                    }
                    // 如果新谷值更高，忽略它
                } else {
                    // 正常谷值
                    valleyPositions.add(i);
                    valleyValues.add(data[i]);
                    valleyNum++;
                }
            }
        }
    }

    /**
     * 获取检测到的步数
     * @return 步数
     */
    public int getStepCount() {
        return Math.min(peakPositions.size(), valleyPositions.size());
    }

    /**
     * 获取检测到的峰值位置
     * @return 峰值位置列表
     */
    public List<Integer> getPeakPositions() {
        return new ArrayList<>(peakPositions);
    }

    /**
     * 获取检测到的峰值值
     * @return 峰值值列表
     */
    public List<Double> getPeakValues() {
        return new ArrayList<>(peakValues);
    }

    /**
     * 获取检测到的谷值位置
     * @return 谷值位置列表
     */
    public List<Integer> getValleyPositions() {
        return new ArrayList<>(valleyPositions);
    }

    /**
     * 获取检测到的谷值值
     * @return 谷值值列表
     */
    public List<Double> getValleyValues() {
        return new ArrayList<>(valleyValues);
    }

    /**
     * 获取标记峰值和谷值位置的 2D 数组
     * 每行对应一个样本索引，包含两列：
     * 第 0 列：如果样本是峰值，则为峰值值，否则为 0
     * 第 1 列：如果样本是谷值，则为谷值值，否则为 0
     * @param length 数据长度
     * @return 标记峰值和谷值的 2D 数组
     */
    public double[][] getPeakValleyFlags(int length) {
        double[][] flags = new double[length][2];

        for (int i = 0; i < peakPositions.size(); i++) {
            int pos = peakPositions.get(i);
            if (pos < length) {
                flags[pos][0] = peakValues.get(i);
            }
        }

        for (int i = 0; i < valleyPositions.size(); i++) {
            int pos = valleyPositions.get(i);
            if (pos < length) {
                flags[pos][1] = valleyValues.get(i);
            }
        }

        return flags;
    }
}
