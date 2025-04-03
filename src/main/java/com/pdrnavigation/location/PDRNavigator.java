package com.pdrnavigation.location;

import com.pdrnavigation.data.NavigationResult;
import com.pdrnavigation.data.SensorData;
import com.pdrnavigation.filters.FIRFilter;
import com.pdrnavigation.math.DCM;
import com.pdrnavigation.math.MathUtils;
import com.pdrnavigation.orientation.HeadingEstimator;
import com.pdrnavigation.steps.PeakValleyDetector;
import com.pdrnavigation.steps.StepDetector;
import com.pdrnavigation.steps.StepLength;
import com.pdrnavigation.utils.Vector3D;

/**
 * 用于行人航位推算的 PDR 导航器
 * 该类整合了 PDR 系统的所有组件
 */
public class PDRNavigator {
    private StepDetector stepDetector;
    private StepLength stepLength;
    private HeadingEstimator headingEstimator;
    private FIRFilter lowPassFilter;

    private double initialNorth;
    private double initialEast;
    private double degToRad;

    /**
     * 创建一个新的 PDR 导航器
     * @param initialNorth 初始北坐标（米）
     * @param initialEast 初始东坐标（米）
     */
    public PDRNavigator(double initialNorth, double initialEast) {
        this.initialNorth = initialNorth;
        this.initialEast = initialEast;
        this.degToRad = Math.PI / 180.0;

        // 初始化组件
        stepDetector = new StepDetector(10, 0.06, 0.5);  // 滤波器阶数，截止频率，阈值
        stepLength = new StepLength(0.5, StepLength.Method.WEINBERG); // K 常数，方法
        headingEstimator = new HeadingEstimator(1.0); // 角检测阈值
        lowPassFilter = FIRFilter.createLowPass(10, 0.06); // 滤波器阶数，截止频率
    }

    /**
     * 处理传感器数据以执行 PDR
     * @param sensorData 传感器数据（加速度计、陀螺仪、方向）
     * @return 包含步进信息和位置的导航结果
     */
    public NavigationResult navigate(SensorData sensorData) {
        // 提取传感器数据
        double[] accX = sensorData.getAccelerometerX();
        double[] accY = sensorData.getAccelerometerY();
        double[] accZ = sensorData.getAccelerometerZ();

        double[] gyroZ = sensorData.getGyroscopeZ();

        double[] roll = sensorData.getRoll();
        double[] pitch = sensorData.getPitch();
        double[] yaw = sensorData.getYaw();

        int dataLength = accX.length;

        // 将方向角转换为弧度
        for (int i = 0; i < dataLength; i++) {
            roll[i] *= degToRad;
            pitch[i] *= degToRad;
            yaw[i] *= degToRad;
        }

        // 在机体坐标系中创建重力向量
        Vector3D[] gravity = new Vector3D[dataLength];
        Vector3D[] linearAcc = new Vector3D[dataLength];
        Vector3D g0 = new Vector3D(0, 0, MathUtils.GRAVITY);

        for (int i = 0; i < dataLength; i++) {
            // 使用欧拉角创建 DCM
            DCM dcm = DCM.fromEulerAngles(roll[i], pitch[i], yaw[i]);

            // 将重力从导航坐标系转换到机体坐标系
            gravity[i] = dcm.transformToBodyFrame(g0);

            // 通过去除重力计算线性加速度
            linearAcc[i] = new Vector3D(accX[i], accY[i], accZ[i]);
            linearAcc[i].subtract(gravity[i]);
        }

        // 计算加速度大小
        double[] accMagnitude = new double[dataLength];
        for (int i = 0; i < dataLength; i++) {
            accMagnitude[i] = linearAcc[i].magnitude();
        }

        // 滤波加速度数据
        double[] filteredAcc = lowPassFilter.process(accMagnitude);

        // 检测步进
        int stepCount = stepDetector.detectSteps(filteredAcc);
        double[][] peakValleyFlags = stepDetector.getPeakValleyFlags();

        // 估计步长
        double[] stepLengths = stepLength.estimateStepLengths(peakValleyFlags, filteredAcc);

        // 估计方向
        double[] filteredHeading = headingEstimator.estimateHeading(yaw, gyroZ);

        // 使用步长和方向计算位置
        double[] north = new double[stepCount + 1];
        double[] east = new double[stepCount + 1];

        north[0] = initialNorth;
        east[0] = initialEast;

        // 获取峰值和谷值位置以计算步进方向
        java.util.List<Integer> peakPositions = stepDetector.getPeakPositions();
        java.util.List<Integer> valleyPositions = stepDetector.getValleyPositions();

        // 计算每个步进的方向
        double[] stepOrientation = new double[stepCount];
        double[] medianHeadings = new double[stepCount];

        for (int i = 0; i < stepCount; i++) {
            int peakPos = peakPositions.get(i);
            int valleyPos = valleyPositions.get(i);

            int startPos = Math.min(peakPos, valleyPos);
            int endPos = Math.max(peakPos, valleyPos);

            // 计算此步进的中值方向
            double[] stepHeadings = new double[endPos - startPos + 1];
            for (int j = startPos, k = 0; j <= endPos; j++, k++) {
                stepHeadings[k] = filteredHeading[j];
            }

            // 使用此步进的中值方向
            medianHeadings[i] = MathUtils.median(stepHeadings);

            // 分配步进方向，使用简单的基于阈值的平滑
            if (i == 0) {
                stepOrientation[i] = medianHeadings[i];
            } else {
                double headingDiff = Math.abs(medianHeadings[i] - medianHeadings[i - 1]);

                // 如果方向变化较小，使用前一个方向来平滑路径
                if (headingDiff < 5 * degToRad) { // 5 度阈值
                    stepOrientation[i] = stepOrientation[i - 1];
                } else {
                    stepOrientation[i] = medianHeadings[i];
                }
            }

            // 使用步长和方向计算位置
            north[i + 1] = north[i] + stepLengths[i] * Math.cos(stepOrientation[i]);
            east[i + 1] = east[i] + stepLengths[i] * Math.sin(stepOrientation[i]);
        }

        // 创建导航结果
        NavigationResult result = new NavigationResult();
        result.setStepCount(stepCount);
        result.setStepLengths(stepLengths);
        result.setStepOrientations(stepOrientation);
        result.setNorthPositions(north);
        result.setEastPositions(east);
        result.setTotalDistance(stepLength.calculateTotalDistance(stepLengths));

        return result;
    }

    /**
     * 设置步进检测阈值
     * @param threshold 新的阈值
     */
    public void setStepDetectionThreshold(double threshold) {
        this.stepDetector = new StepDetector(10, 0.06, threshold);
    }

    /**
     * 设置步长常数 K
     * @param kConstant 新的 K 常数
     */
    public void setStepLengthConstant(double kConstant) {
        this.stepLength.setKConstant(kConstant);
    }

    /**
     * 设置步长估算方法
     * @param method 新的方法
     */
    public void setStepLengthMethod(StepLength.Method method) {
        this.stepLength.setMethod(method);
    }

    /**
     * 设置角检测阈值
     * @param threshold 新的阈值
     */
    public void setCornerDetectionThreshold(double threshold) {
        this.headingEstimator = new HeadingEstimator(threshold);
    }

    /**
     * 设置初始位置
     * @param north 初始北坐标（米）
     * @param east 初始东坐标（米）
     */
    public void setInitialPosition(double north, double east) {
        this.initialNorth = north;
        this.initialEast = east;
    }
}
