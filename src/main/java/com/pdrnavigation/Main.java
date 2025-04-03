package com.pdrnavigation;

import com.pdrnavigation.data.NavigationResult;
import com.pdrnavigation.data.SensorData;
import com.pdrnavigation.data.SensorDataLoader;
import com.pdrnavigation.location.PDRNavigator;
import com.pdrnavigation.steps.StepLength;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;


public class Main {
    public static void main(String[] args) {
        try {
            // 解析命令行参数
            String dataDir = ".";
            if (args.length > 0) {
                dataDir = args[0];
            }

            // 默认文件路径
            String imudataDir = "resources/imudata10";
            String accFile = imudataDir + File.separator + "acc.txt";
            String gyroFile = imudataDir + File.separator + "gyr.txt";
            String oriFile = imudataDir + File.separator + "ori.txt";
            String outputFile = dataDir + File.separator + "navigation_result.txt";
            System.out.println("PDR 导航系统");
            System.out.println("-------------------");
            System.out.println("加速度计数据: " + accFile);
            System.out.println("陀螺仪数据: " + gyroFile);
            System.out.println("方向数据: " + oriFile);
            System.out.println("Output file: " + outputFile);

            // 加载传感器数据
            System.out.println("\n正在加载传感器数据...");
            SensorData sensorData = SensorDataLoader.loadFromFiles(accFile, gyroFile, oriFile, 50.0);
            System.out.println("加载了 " + sensorData.getDataLength() + " 个数据点");

            // 初始化 PDR 导航器
            double initialNorth = 40.0;
            double initialEast = 0.0;

            System.out.println("\n正在初始化 PDR 导航器...");
            PDRNavigator navigator = new PDRNavigator(initialNorth, initialEast);

            // 配置导航器参数（可以调整这些参数以获得更好的性能）
            navigator.setStepDetectionThreshold(0.5);    // 步数检测阈值
            navigator.setStepLengthConstant(0.5);        // 步长 K 常数
            navigator.setStepLengthMethod(StepLength.Method.WEINBERG);  // 步长法
            navigator.setCornerDetectionThreshold(1.0);  // 拐角检测阈值

            // 执行 PDR 导航
            System.out.println("正在执行 PDR 导航...");
            NavigationResult result = navigator.navigate(sensorData);

            // 显示结果
            System.out.println("\n导航结果:");
            System.out.println(result.toString());

            // 将结果保存到文件
            saveResultsToFile(result, outputFile);
            System.out.println("\n结果保存到" + outputFile);

        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * 将导航结果保存到文本文件
     * @param result 导航结果
     * @param filePath 输出文件路径
     * @throws IOException 如果发生 I/O 错误
     */
    private static void saveResultsToFile(NavigationResult result, String filePath) throws IOException {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath))) {
            // 写入 header
            writer.println("# PDR Navigation Results");
            writer.println("# Step Count: " + result.getStepCount());
            writer.println("# Total Distance: " + String.format("%.2f", result.getTotalDistance()) + " m");
            writer.println("#");
            writer.println("# Format: Step, North (m), East (m), StepLength (m), Orientation (rad)");

            // 写入 data
            double[] north = result.getNorthPositions();
            double[] east = result.getEastPositions();
            double[] stepLengths = result.getStepLengths();
            double[] orientations = result.getStepOrientations();

            // 写入初始位置
            writer.println("0, " + north[0] + ", " + east[0] + ", 0.0, 0.0");

            // 写入step数据
            for (int i = 0; i < result.getStepCount(); i++) {
                writer.println((i+1) + ", " +
                        north[i+1] + ", " +
                        east[i+1] + ", " +
                        stepLengths[i] + ", " +
                        orientations[i]);
            }
        }
    }
}