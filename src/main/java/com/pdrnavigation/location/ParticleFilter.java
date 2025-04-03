package com.pdrnavigation.location;

import java.util.Random;

/**
 * 粒子滤波器实现用于位置跟踪
 * 实现了 part.m 中的功能
 */
public class ParticleFilter {
    private int numParticles;
    private int stateDim;
    private double[][] particles;
    private double[] weights;
    private double[][] processNoise;
    private double[][] measurementNoise;
    private Random random;

    /**
     * 创建一个新的粒子滤波器
     * @param numParticles 粒子数量
     * @param stateDim 状态向量的维度
     */
    public ParticleFilter(int numParticles, int stateDim) {
        this.numParticles = numParticles;
        this.stateDim = stateDim;

        particles = new double[numParticles][stateDim];
        weights = new double[numParticles];
        processNoise = new double[stateDim][stateDim];
        measurementNoise = new double[2][2]; // 假设 2D 位置测量

        // 初始化权重为均匀分布
        double initialWeight = 1.0 / numParticles;
        for (int i = 0; i < numParticles; i++) {
            weights[i] = initialWeight;
        }

        random = new Random();
    }

    /**
     * 根据给定的初始状态初始化粒子
     * @param initialState 初始状态向量
     * @param initialNoise 初始噪声标准差
     */
    public void initializeParticles(double[] initialState, double initialNoise) {
        for (int i = 0; i < numParticles; i++) {
            for (int j = 0; j < stateDim; j++) {
                particles[i][j] = initialState[j] + random.nextGaussian() * initialNoise;
            }
        }
    }

    /**
     * 设置过程噪声协方差矩阵
     * @param noise 过程噪声矩阵 (stateDim x stateDim)
     */
    public void setProcessNoise(double[][] noise) {
        if (noise.length != stateDim || noise[0].length != stateDim) {
            throw new IllegalArgumentException("过程噪声矩阵必须是 " + stateDim + "x" + stateDim);
        }

        this.processNoise = noise;
    }

    /**
     * 设置测量噪声协方差矩阵
     * @param noise 测量噪声矩阵 (2x2 用于 2D 位置)
     */
    public void setMeasurementNoise(double[][] noise) {
        if (noise.length != 2 || noise[0].length != 2) {
            throw new IllegalArgumentException("测量噪声矩阵必须是 2x2");
        }

        this.measurementNoise = noise;
    }

    /**
     * 使用给定的运动模型预测粒子状态
     * @param motionModel 用于预测的运动模型
     */
    public void predict(MotionModel motionModel) {
        for (int i = 0; i < numParticles; i++) {
            // 应用运动模型
            double[] newState = motionModel.predict(particles[i]);

            // 添加过程噪声
            for (int j = 0; j < stateDim; j++) {
                double noise = 0;
                for (int k = 0; k < stateDim; k++) {
                    noise += random.nextGaussian() * processNoise[j][k];
                }
                particles[i][j] = newState[j] + noise;
            }
        }
    }

    /**
     * 根据测量更新粒子权重
     * @param measurement 2D 位置测量 [北, 东]
     * @param mapConstraints 地图约束函数
     */
    public void update(double[] measurement, MapConstraints mapConstraints) {
        double totalWeight = 0;

        for (int i = 0; i < numParticles; i++) {
            // 计算测量似然度
            double northError = measurement[0] - particles[i][0];
            double eastError = measurement[1] - particles[i][1];

            double likelihood = Math.exp(-(
                    (northError * northError) / (2 * measurementNoise[0][0]) +
                            (eastError * eastError) / (2 * measurementNoise[1][1])
            ));

            // 应用地图约束（如果粒子在地图中有效）
            if (mapConstraints.isValidPosition(particles[i][0], particles[i][1])) {
                weights[i] *= likelihood;
            } else {
                weights[i] *= 0.01 * likelihood; // 对于无效位置减少权重
            }

            totalWeight += weights[i];
        }

        // 归一化权重
        if (totalWeight > 0) {
            for (int i = 0; i < numParticles; i++) {
                weights[i] /= totalWeight;
            }
        } else {
            // 如果所有权重为零，重置为均匀分布
            for (int i = 0; i < numParticles; i++) {
                weights[i] = 1.0 / numParticles;
            }
        }
    }

    /**
     * 执行系统重采样以避免粒子退化
     */
    public void resample() {
        double[][] newParticles = new double[numParticles][stateDim];

        // 计算累积分布函数
        double[] cdf = new double[numParticles];
        cdf[0] = weights[0];
        for (int i = 1; i < numParticles; i++) {
            cdf[i] = cdf[i - 1] + weights[i];
        }

        // 系统重采样
        double u = random.nextDouble() / numParticles;
        int j = 0;

        for (int i = 0; i < numParticles; i++) {
            while (j < numParticles - 1 && u > cdf[j]) {
                j++;
            }

            // 复制粒子
            for (int k = 0; k < stateDim; k++) {
                newParticles[i][k] = particles[j][k];
            }

            u += 1.0 / numParticles;
        }

        // 更新粒子
        particles = newParticles;

        // 重置权重为均匀分布
        for (int i = 0; i < numParticles; i++) {
            weights[i] = 1.0 / numParticles;
        }
    }

    /**
     * 获取当前状态估计（粒子的加权平均）
     * @return 状态估计
     */
    public double[] getStateEstimate() {
        double[] estimate = new double[stateDim];

        for (int i = 0; i < numParticles; i++) {
            for (int j = 0; j < stateDim; j++) {
                estimate[j] += particles[i][j] * weights[i];
            }
        }

        return estimate;
    }

    /**
     * 获取所有粒子
     * @return 粒子数组
     */
    public double[][] getParticles() {
        return particles;
    }

    /**
     * 获取粒子权重
     * @return 权重数组
     */
    public double[] getWeights() {
        return weights;
    }

    /**
     * 用于粒子预测的运动模型接口
     */
    public interface MotionModel {
        /**
         * 根据当前状态预测下一个状态
         * @param state 当前状态
         * @return 下一个状态
         */
        double[] predict(double[] state);
    }

    /**
     * 地图约束接口
     */
    public interface MapConstraints {
        /**
         * 检查位置是否在地图中有效
         * @param north 北坐标
         * @param east 东坐标
         * @return 如果位置有效，返回 true；否则返回 false
         */
        boolean isValidPosition(double north, double east);
    }
}
