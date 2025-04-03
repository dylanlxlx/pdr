package com.pdrnavigation.location;

import java.util.ArrayList;
import java.util.List;

/**
 * PDR 地图匹配类
 * 用于将 PDR 轨迹匹配到室内地图
 */
public class MapMatcher {
    private List<Wall> walls;
    private List<Rectangle> rooms;
    private double snappingDistance;

    /**
     * 创建一个新的地图匹配器
     * @param snappingDistance 最大吸附到墙壁的距离（米）
     */
    public MapMatcher(double snappingDistance) {
        this.walls = new ArrayList<>();
        this.rooms = new ArrayList<>();
        this.snappingDistance = snappingDistance;
    }

    /**
     * 向地图添加一个墙壁
     * @param startNorth 墙壁起点的北坐标
     * @param startEast 墙壁起点的东坐标
     * @param endNorth 墙壁终点的北坐标
     * @param endEast 墙壁终点的东坐标
     */
    public void addWall(double startNorth, double startEast, double endNorth, double endEast) {
        walls.add(new Wall(startNorth, startEast, endNorth, endEast));
    }

    /**
     * 向地图添加一个房间
     * @param north 房间的中心北坐标
     * @param east 房间的中心东坐标
     * @param width 房间的宽度（东西方向）
     * @param height 房间的高度（南北方向）
     */
    public void addRoom(double north, double east, double width, double height) {
        rooms.add(new Rectangle(north, east, width, height));
    }

    /**
     * 将 PDR 轨迹匹配到地图
     * @param northPositions PDR 的北坐标数组
     * @param eastPositions PDR 的东坐标数组
     * @return 匹配后的坐标数组 [北, 东]
     */
    public double[][] matchTrajectory(double[] northPositions, double[] eastPositions) {
        int length = northPositions.length;
        double[][] matchedPositions = new double[length][2];

        // 复制初始位置
        matchedPositions[0][0] = northPositions[0];
        matchedPositions[0][1] = eastPositions[0];

        for (int i = 1; i < length; i++) {
            // 获取当前和前一个位置
            double prevNorth = matchedPositions[i - 1][0];
            double prevEast = matchedPositions[i - 1][1];
            double currNorth = northPositions[i];
            double currEast = eastPositions[i];

            // 计算移动向量
            double deltaForth = currNorth - prevNorth;
            double deltaEast = currEast - prevEast;

            // 检查位置是否在房间内
            boolean insideRoom = false;
            for (Rectangle room : rooms) {
                if (room.contains(currNorth, currEast)) {
                    insideRoom = true;
                    break;
                }
            }

            if (insideRoom) {
                // 如果在房间内，保持原始位置
                matchedPositions[i][0] = currNorth;
                matchedPositions[i][1] = currEast;
            } else {
                // 检查墙壁约束
                boolean wallConstraint = false;
                double bestNorth = currNorth;
                double bestEast = currEast;
                double minDistance = Double.MAX_VALUE;

                for (Wall wall : walls) {
                    // 计算点到墙壁的垂直距离
                    double distance = wall.distanceToPoint(currNorth, currEast);

                    if (distance < snappingDistance && distance < minDistance) {
                        // 将点投影到墙壁上
                        double[] projection = wall.projectPoint(currNorth, currEast);

                        // 检查投影是否在墙壁段上
                        if (wall.containsProjection(projection[0], projection[1])) {
                            bestNorth = projection[0];
                            bestEast = projection[1];
                            minDistance = distance;
                            wallConstraint = true;
                        }
                    }
                }

                if (wallConstraint) {
                    // 如果接近墙壁，吸附到墙壁上
                    matchedPositions[i][0] = bestNorth;
                    matchedPositions[i][1] = bestEast;
                } else {
                    // 否则，保持原始位置
                    matchedPositions[i][0] = currNorth;
                    matchedPositions[i][1] = currEast;
                }
            }
        }

        return matchedPositions;
    }

    /**
     * 检查位置是否在地图中有效（在房间内或远离墙壁）
     * @param north 北坐标
     * @param east 东坐标
     * @return 如果位置有效，返回 true；否则返回 false
     */
    public boolean isValidPosition(double north, double east) {
        // 检查是否在任何房间内
        for (Rectangle room : rooms) {
            if (room.contains(north, east)) {
                return true;
            }
        }

        // 检查是否太接近任何墙壁
        for (Wall wall : walls) {
            if (wall.distanceToPoint(north, east) < 0.3) { // 30cm 以内为无效
                return false;
            }
        }

        return true;
    }

    /**
     * 表示墙壁段的 Wall 类
     */
    public static class Wall {
        public double startNorth;
        public double startEast;
        public double endNorth;
        public double endEast;

        /**
         * 创建一个新的墙壁段
         * @param startNorth 墙壁起点的北坐标
         * @param startEast 墙壁起点的东坐标
         * @param endNorth 墙壁终点的北坐标
         * @param endEast 墙壁终点的东坐标
         */
        public Wall(double startNorth, double startEast, double endNorth, double endEast) {
            this.startNorth = startNorth;
            this.startEast = startEast;
            this.endNorth = endNorth;
            this.endEast = endEast;
        }

        /**
         * 计算点到墙壁的垂直距离
         * @param north 点的北坐标
         * @param east 点的东坐标
         * @return 到墙壁的距离
         */
        public double distanceToPoint(double north, double east) {
            // 计算点到线的垂直距离
            double dx = endEast - startEast;
            double dy = endNorth - startNorth;
            double magnitude = Math.sqrt(dx * dx + dy * dy);

            if (magnitude == 0) {
                // 起点和终点相同
                return Math.sqrt((north - startNorth) * (north - startNorth) +
                        (east - startEast) * (east - startEast));
            }

            // 距离公式: |Ax + By + C| / sqrt(A² + B²)
            // 其中线方程为 Ax + By + C = 0
            double A = dy;
            double B = -dx;
            double C = startEast * endNorth - endEast * startNorth;

            return Math.abs(A * east + B * north + C) / magnitude;
        }

        /**
         * 将点投影到墙壁上
         * @param north 点的北坐标
         * @param east 点的东坐标
         * @return 投影点 [北, 东]
         */
        public double[] projectPoint(double north, double east) {
            double dx = endEast - startEast;
            double dy = endNorth - startNorth;
            double magnitude = dx * dx + dy * dy;

            if (magnitude == 0) {
                // 起点和终点相同
                return new double[] {startNorth, startEast};
            }

            // 使用点积计算投影
            double t = ((east - startEast) * dx + (north - startNorth) * dy) / magnitude;

            double projectedEast = startEast + t * dx;
            double projectedNorth = startNorth + t * dy;

            return new double[] {projectedNorth, projectedEast};
        }

        /**
         * 检查投影点是否在墙壁段上
         * @param north 投影点的北坐标
         * @param east 投影点的东坐标
         * @return 如果投影在墙壁段上，返回 true；否则返回 false
         */
        public boolean containsProjection(double north, double east) {
            // 检查投影是否在起点和终点之间
            double minNorth = Math.min(startNorth, endNorth);
            double maxNorth = Math.max(startNorth, endNorth);
            double minEast = Math.min(startEast, endEast);
            double maxEast = Math.max(startEast, endEast);

            return (north >= minNorth - 1e-6 && north <= maxNorth + 1e-6 &&
                    east >= minEast - 1e-6 && east <= maxEast + 1e-6);
        }
    }

    /**
     * 表示房间的 Rectangle 类
     */
    public static class Rectangle {
        public double centerNorth;
        public double centerEast;
        public double width;
        public double height;

        /**
         * 创建一个新的矩形
         * @param centerNorth 矩形中心的北坐标
         * @param centerEast 矩形中心的东坐标
         * @param width 矩形的宽度（东西方向）
         * @param height 矩形的高度（南北方向）
         */
        public Rectangle(double centerNorth, double centerEast, double width, double height) {
            this.centerNorth = centerNorth;
            this.centerEast = centerEast;
            this.width = width;
            this.height = height;
        }

        /**
         * 检查点是否在矩形内
         * @param north 点的北坐标
         * @param east 点的东坐标
         * @return 如果点在矩形内，返回 true；否则返回 false
         */
        public boolean contains(double north, double east) {
            double halfWidth = width / 2;
            double halfHeight = height / 2;

            return (east >= centerEast - halfWidth && east <= centerEast + halfWidth &&
                    north >= centerNorth - halfHeight && north <= centerNorth + halfHeight);
        }
    }
}
