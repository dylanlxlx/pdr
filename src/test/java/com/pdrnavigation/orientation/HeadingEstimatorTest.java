package com.pdrnavigation.orientation;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * HeadingEstimator 类的单元测试
 */
public class HeadingEstimatorTest {
    private HeadingEstimator headingEstimator;
    private static final double DEG_TO_RAD = Math.PI / 180.0;

    @BeforeEach
    public void setUp() {
        // 创建一个阈值为 1.0 的航向估计器
        headingEstimator = new HeadingEstimator(1.0);
    }

    @Test
    public void testConstantHeading() {
        // 创建恒定航向数据
        double[] heading = new double[100];
        double[] gyroZ = new double[100];

        // 所有航向为 90 度
        for (int i = 0; i < heading.length; i++) {
            heading[i] = 90.0;
            gyroZ[i] = 0.0; // 无旋转
        }

        // 估计航向
        double[] filtered = headingEstimator.estimateHeading(heading, gyroZ);

        // 过滤后的航向应约为 90 度（弧度）
        for (int i = 0; i < filtered.length; i++) {
            assertEquals(90.0 * DEG_TO_RAD, filtered[i], 0.01, "航向应保持恒定");
        }
    }

    @Test
    public void testHeadingJump() {
        // 创建带有跳变的航向数据
        double[] heading = new double[100];
        double[] gyroZ = new double[100];

        // 前半部分为 90 度
        for (int i = 0; i < 50; i++) {
            heading[i] = 90.0;
            gyroZ[i] = 0.0; // 无旋转
        }

        // 后半部分为 -90 度（应展开为 270 度）
        for (int i = 50; i < 100; i++) {
            heading[i] = -90.0;
            gyroZ[i] = 0.0; // 无旋转
        }

        // 在跳变处添加高陀螺仪值以模拟转弯
        gyroZ[49] = 10.0; // 高角速度
        gyroZ[50] = 10.0;

        // 估计航向
        double[] filtered = headingEstimator.estimateHeading(heading, gyroZ);

        // 获取展开的航向
        double[] unwrapped = headingEstimator.getUnwrappedHeading();

        // 检查展开 - 跳变应被处理
        assertEquals(90.0, unwrapped[49], 0.01, "跳变前的航向");
        assertEquals(270.0, unwrapped[50], 0.01, "跳变后的航向应被展开");

        // 检查过滤后的航向 - 跳变应被保留但经过滤波
        assertTrue(filtered[49] < filtered[50], "跳变处航向应增加");
    }

    @Test
    public void testCornerDetection() {
        // 创建带有渐变转弯的航向数据
        double[] heading = new double[100];
        double[] gyroZ = new double[100];

        // 前部分稳定在 0 度
        for (int i = 0; i < 40; i++) {
            heading[i] = 0.0;
            gyroZ[i] = 0.0; // 无旋转
        }

        // 转弯期间具有高陀螺仪值
        for (int i = 40; i < 60; i++) {
            heading[i] = (i - 40) * 4.5; // 逐渐增加航向
            gyroZ[i] = 4.5; // 高角速度
        }

        // 后部分稳定在 90 度
        for (int i = 60; i < 100; i++) {
            heading[i] = 90.0;
            gyroZ[i] = 0.0; // 无旋转
        }

        // 估计航向
        double[] filtered = headingEstimator.estimateHeading(heading, gyroZ);

        // 检查过滤后的航向 - 应接近原始航向但更平滑
        assertEquals(0.0, filtered[0] / DEG_TO_RAD, 1.0, "初始航向应接近 0");
        assertEquals(90.0, filtered[99] / DEG_TO_RAD, 1.0, "最终航向应接近 90");

        // 中间部分应显示平滑过渡
        for (int i = 41; i < 59; i++) {
            assertTrue(filtered[i] > filtered[i-1],
                    "转弯期间航向应一致增加");
        }
    }
}
