package com.pdrnavigation.steps;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * StepDetector 类的单元测试
 */
public class StepDetectorTest {
    private StepDetector stepDetector;

    @BeforeEach
    public void setUp() {
        // 创建一个阶数为 10，截止频率为 0.06，阈值为 0.5 的步态检测器
        stepDetector = new StepDetector(10, 0.06, 0.5);
    }

    @Test
    public void testBasicStepDetection() {
        // 创建一个简单的正弦波，模拟行走时的加速度数据
        // 步态模式：每个周期为一个步态
        double[] accelerometerData = new double[100];
        for (int i = 0; i < accelerometerData.length; i++) {
            // 幅度为 2.0（大于阈值 0.5）
            // 频率使得样本中有大约 10 个步态
            accelerometerData[i] = 2.0 * Math.sin(i * Math.PI / 10.0);
        }

        // 检测步态
        int stepCount = stepDetector.detectSteps(accelerometerData);

        // 应该检测到大约 10 个步态（考虑滤波器效应有一定的余量）
        assertTrue(stepCount >= 8 && stepCount <= 12,
                "预期大约 10 个步态，但检测到 " + stepCount);
    }

    @Test
    public void testNoSteps() {
        // 创建一个低于阈值的平滑信号
        double[] flatData = new double[100];
        for (int i = 0; i < flatData.length; i++) {
            flatData[i] = 0.1; // 低于阈值
        }

        // 检测步态
        int stepCount = stepDetector.detectSteps(flatData);

        // 应该检测到 0 个步态
        assertEquals(0, stepCount, "预期平滑数据低于阈值时检测到 0 个步态");
    }

    @Test
    public void testHighFrequencyNoise() {
        // 创建一个包含高频噪声的信号（应被滤波器滤除）
        double[] noisyData = new double[100];
        for (int i = 0; i < noisyData.length; i++) {
            // 低频行走模式
            double walkingSignal = 2.0 * Math.sin(i * Math.PI / 10.0);

            // 高频噪声（应被滤波器滤除）
            double noise = 0.3 * Math.sin(i * Math.PI);

            noisyData[i] = walkingSignal + noise;
        }

        // 检测步态
        int stepCount = stepDetector.detectSteps(noisyData);

        // 尽管有噪声，仍然应该检测到大约 10 个步态
        assertTrue(stepCount >= 8 && stepCount <= 12,
                "尽管有噪声，预期大约 10 个步态，但检测到 " + stepCount);
    }
}
