package com.pdrnavigation.filters;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * KalmanFilter 类的单元测试
 */
public class KalmanFilterTest {
    private KalmanFilter filter;

    @BeforeEach
    public void setUp() {
        // 创建一个 2 状态的卡尔曼滤波器
        filter = new KalmanFilter(2, 1);

        // 设置初始状态 [位置, 速度]
        filter.setState(new double[] {0.0, 0.0});

        // 设置状态转移矩阵
        // [1 dt]
        // [0  1]
        double[][] A = {
                {1.0, 1.0},
                {0.0, 1.0}
        };
        filter.setStateTransitionMatrix(A);

        // 设置测量矩阵
        // [1 0]
        double[][] H = {
                {1.0, 0.0}
        };
        filter.setMeasurementMatrix(H);

        // 设置过程噪声协方差
        double[][] Q = {
                {0.01, 0.0},
                {0.0, 0.01}
        };
        filter.setProcessNoiseCovariance(Q);

        // 设置测量噪声协方差
        double[][] R = {
                {0.1}
        };
        filter.setMeasurementNoiseCovariance(R);

        // 设置误差协方差
        double[][] P = {
                {1.0, 0.0},
                {0.0, 1.0}
        };
        filter.setErrorCovariance(P);
    }

    @Test
    public void testPredictStep() {
        // 初始状态是 [0, 0]
        double[] initialState = filter.getState();
        assertEquals(0.0, initialState[0], 1e-6);
        assertEquals(0.0, initialState[1], 1e-6);

        // 预测（没有测量值）
        filter.predict();

        // 预测后，位置应根据速度更新
        // 由于速度为 0，位置应仍为 0
        double[] state = filter.getState();
        assertEquals(0.0, state[0], 1e-6);
        assertEquals(0.0, state[1], 1e-6);

        // 设置速度为 1.0
        filter.setState(new double[] {0.0, 1.0});

        // 再次预测
        filter.predict();

        // 现在位置应根据速度 * dt 更新（即 1.0 * 1.0）
        state = filter.getState();
        assertEquals(1.0, state[0], 1e-6);
        assertEquals(1.0, state[1], 1e-6);
    }

    @Test
    public void testUpdateStep() {
        // 设置初始状态为 [0, 1]（位置 = 0, 速度 = 1）
        filter.setState(new double[] {0.0, 1.0});

        // 预测
        filter.predict();

        // 预测后，状态应为 [1, 1]
        double[] state = filter.getState();
        assertEquals(1.0, state[0], 1e-6);
        assertEquals(1.0, state[1], 1e-6);

        // 现在使用位置 = 1.2 的测量值进行更新
        filter.update(new double[] {1.2});

        // 位置应向测量值调整
        // 速度也可能由于卡尔曼增益而略有变化
        state = filter.getState();
        assertTrue(state[0] > 1.0 && state[0] < 1.2, "位置应向测量值移动");

        // 执行多个预测/更新周期
        for (int i = 0; i < 10; i++) {
            filter.predict();
            filter.update(new double[] {i + 2.0});
        }

        // 确保状态仍然有效
        state = filter.getState();
        assertTrue(state[0] >= 0.0, "位置不应为负");
        assertTrue(state[1] >= 0.0, "速度不应为负");
    }

    @Test
    public void testNonLinearUpdate() {
        // 设置初始状态为 [0, 1]
        filter.setState(new double[] {0.0, 1.0});

        // 预测
        filter.predict();

        // 创建一个非线性测量函数
        // h(x) = x^2 (位置的平方)
        double[] h = new double[] {1.0}; // 1.0^2

        // h(x) 关于状态的雅可比矩阵
        // J = [2x, 0]
        double[][] H = {
                {2.0, 0.0} // 2 * 位置
        };

        // 使用非线性测量进行更新
        filter.updateNonLinear(new double[] {1.1}, h, H);

        // 状态应被更新
        double[] state = filter.getState();
        assertNotEquals(1.0, state[0], "位置应在更新后改变");
    }
}
