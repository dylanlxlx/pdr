package com.pdrnavigation.data;

/**
 * 用于存储 PDR 导航结果的类
 */
public class NavigationResult {
    private int stepCount;
    private double[] stepLengths;
    private double[] stepOrientations;
    private double[] northPositions;
    private double[] eastPositions;
    private double totalDistance;

    public NavigationResult() {
        this.stepCount = 0;
        this.stepLengths = new double[0];
        this.stepOrientations = new double[0];
        this.northPositions = new double[0];
        this.eastPositions = new double[0];
        this.totalDistance = 0.0;
    }

    /**
     * 使用给定数据创建新的导航结果
     * @param stepCount 步数
     * @param stepLengths 步长
     * @param stepOrientations 步向（以弧度为单位）
     * @param northPositions 北方位置
     * @param eastPositions 东方位置
     * @param totalDistance 总距离
     */
    public NavigationResult(int stepCount, double[] stepLengths, double[] stepOrientations,
                            double[] northPositions, double[] eastPositions, double totalDistance) {
        this.stepCount = stepCount;
        this.stepLengths = stepLengths.clone();
        this.stepOrientations = stepOrientations.clone();
        this.northPositions = northPositions.clone();
        this.eastPositions = eastPositions.clone();
        this.totalDistance = totalDistance;
    }

    /**
     * 获取步数
     * @return 步数
     */
    public int getStepCount() {
        return stepCount;
    }

    /**
     * 设置步数
     * @param stepCount 新的步数
     */
    public void setStepCount(int stepCount) {
        this.stepCount = stepCount;
    }

    /**
     * 获取步长
     * @return 步长数组
     */
    public double[] getStepLengths() {
        return stepLengths;
    }

    /**
     * 设置步长
     * @param stepLengths 新的步长数组
     */
    public void setStepLengths(double[] stepLengths) {
        this.stepLengths = stepLengths.clone();
    }

    /**
     * 获取步向
     * @return 步向数组（以弧度为单位）
     */
    public double[] getStepOrientations() {
        return stepOrientations;
    }

    /**
     * 设置步向
     * @param stepOrientations 新的步向数组（以弧度为单位）
     */
    public void setStepOrientations(double[] stepOrientations) {
        this.stepOrientations = stepOrientations.clone();
    }

    /**
     * 获取north
     * @return north数组
     */
    public double[] getNorthPositions() {
        return northPositions;
    }

    /**
     * 设置north
     * @param northPositions 新的north数组
     */
    public void setNorthPositions(double[] northPositions) {
        this.northPositions = northPositions.clone();
    }

    /**
     * 获取east
     * @return east数组
     */
    public double[] getEastPositions() {
        return eastPositions;
    }

    /**
     * 设置east位置
     * @param eastPositions 新east数组
     */
    public void setEastPositions(double[] eastPositions) {
        this.eastPositions = eastPositions.clone();
    }

    /**
     * 获取总距离
     * @return 总距离
     */
    public double getTotalDistance() {
        return totalDistance;
    }

    /**
     * 设置总距离
     * @param totalDistance 新的总距离
     */
    public void setTotalDistance(double totalDistance) {
        this.totalDistance = totalDistance;
    }

    /**
     * 获取位置数组
     * @return 2D 数组，包含 [north, east] 位置
     */
    public double[][] getPositionsArray() {
        double[][] positions = new double[northPositions.length][2];

        for (int i = 0; i < northPositions.length; i++) {
            positions[i][0] = northPositions[i];
            positions[i][1] = eastPositions[i];
        }

        return positions;
    }

    /**
     * 获取导航结果的摘要
     * @return 摘要字符串
     */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("导航结果:\n");
        sb.append("步数: ").append(stepCount).append("\n");
        sb.append("总距离: ").append(String.format("%.2f", totalDistance)).append(" 米\n");

        if (northPositions.length > 0) {
            sb.append("起始位置: [").
                    append(String.format("%.2f", northPositions[0])).append(", ").
                    append(String.format("%.2f", eastPositions[0])).append("]\n");

            sb.append("结束位置: [").
                    append(String.format("%.2f", northPositions[northPositions.length - 1])).append(", ").
                    append(String.format("%.2f", eastPositions[eastPositions.length - 1])).append("]\n");
        }

        return sb.toString();
    }
}
