package com.pdrnavigation.filters;

/**
 * 有限脉冲响应 (FIR) 滤波器实现
 * 实现了项目中使用的低通滤波器功能
 */
public class FIRFilter {
    private double[] coefficients;
    private double[] buffer;
    private int bufferIndex;

    /**
     * 使用给定的系数创建一个新的 FIR 滤波器
     * @param coefficients 滤波器系数
     */
    public FIRFilter(double[] coefficients) {
        this.coefficients = coefficients.clone();
        this.buffer = new double[coefficients.length];
        this.bufferIndex = 0;
    }

    /**
     * 使用最小二乘法创建低通 FIR 滤波器
     * 模拟项目中使用的 firls MATLAB 函数
     * @param order 滤波器阶数
     * @param cutoffFreq 归一化截止频率 (0-1)
     * @return 新的 FIR 滤波器
     */
    public static FIRFilter createLowPass(int order, double cutoffFreq) {
        // 使用窗口方法的简单低通滤波器系数实现
        // 对于更精确的 firls 实现，需要更复杂的算法
        double[] coefficients = new double[order + 1];

        for (int i = 0; i <= order; i++) {
            if (i == order / 2) {
                coefficients[i] = 2 * cutoffFreq;
            } else {
                double x = 2 * Math.PI * cutoffFreq * (i - order / 2);
                coefficients[i] = Math.sin(x) / x;
            }

            // 应用汉明窗
            coefficients[i] *= (0.54 - 0.46 * Math.cos(2 * Math.PI * i / order));
        }

        // 归一化系数，使其总和为 1（直流增益为 1）
        double sum = 0;
        for (double c : coefficients) {
            sum += c;
        }

        for (int i = 0; i < coefficients.length; i++) {
            coefficients[i] /= sum;
        }

        return new FIRFilter(coefficients);
    }

    /**
     * 通过滤波器处理单个样本
     * @param input 输入样本
     * @return 滤波后的输出样本
     */
    public double processSample(double input) {
        // 将新样本添加到缓冲区
        buffer[bufferIndex] = input;

        // 计算输出
        double output = 0;
        int index;
        for (int i = 0; i < coefficients.length; i++) {
            index = (bufferIndex - i);
            if (index < 0) {
                index += buffer.length;
            }
            output += coefficients[i] * buffer[index];
        }

        // 更新缓冲区索引
        bufferIndex = (bufferIndex + 1) % buffer.length;

        return output;
    }

    /**
     * 通过滤波器处理样本数组
     * @param input 输入样本数组
     * @return 滤波后的输出样本数组
     */
    public double[] process(double[] input) {
        double[] output = new double[input.length];

        for (int i = 0; i < input.length; i++) {
            output[i] = processSample(input[i]);
        }

        return output;
    }

    /**
     * 重置滤波器状态
     */
    public void reset() {
        for (int i = 0; i < buffer.length; i++) {
            buffer[i] = 0;
        }
        bufferIndex = 0;
    }

    /**
     * 获取滤波器系数
     * @return 滤波器系数
     */
    public double[] getCoefficients() {
        return coefficients.clone();
    }

    /**
     * 获取滤波器阶数
     * @return 滤波器阶数
     */
    public int getOrder() {
        return coefficients.length - 1;
    }
}
