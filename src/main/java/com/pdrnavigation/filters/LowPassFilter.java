package com.pdrnavigation.filters;

/**
 * 简单的低通滤波器实现
 */
public class LowPassFilter {
    private double alpha;
    private double prevOutput;
    private boolean initialized;

    /**
     * 使用给定的 alpha 值创建一个新的低通滤波器
     * @param alpha 滤波器系数（0-1），较小的值给出更平滑的结果
     */
    public LowPassFilter(double alpha) {
        if (alpha < 0 || alpha > 1) {
            throw new IllegalArgumentException("Alpha 必须在 0 和 1 之间");
        }

        this.alpha = alpha;
        this.prevOutput = 0;
        this.initialized = false;
    }

    /**
     * 使用给定的截止频率和采样率创建一个新的低通滤波器
     * @param cutoffFreq 截止频率 (Hz)
     * @param sampleRate 采样率 (Hz)
     * @return 新的低通滤波器
     */
    public static LowPassFilter fromFrequency(double cutoffFreq, double sampleRate) {
        double dt = 1.0 / sampleRate;
        double rc = 1.0 / (2 * Math.PI * cutoffFreq);
        double alpha = dt / (rc + dt);

        return new LowPassFilter(alpha);
    }

    /**
     * 通过滤波器处理单个样本
     * @param input 输入样本
     * @return 滤波后的输出样本
     */
    public double processSample(double input) {
        if (!initialized) {
            prevOutput = input;
            initialized = true;
            return input;
        }

        double output = alpha * input + (1 - alpha) * prevOutput;
        prevOutput = output;

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
        prevOutput = 0;
        initialized = false;
    }

    /**
     * 设置滤波器系数
     * @param alpha 新的 alpha 值
     */
    public void setAlpha(double alpha) {
        if (alpha < 0 || alpha > 1) {
            throw new IllegalArgumentException("Alpha 必须在 0 和 1 之间");
        }

        this.alpha = alpha;
    }

    /**
     * 获取当前滤波器系数
     * @return 当前的 alpha 值
     */
    public double getAlpha() {
        return alpha;
    }
}
