package org.firstinspires.ftc.teamcode.drive.traj;

import org.opencv.core.Point;
import java.util.ArrayList;

public class Util {
    //Keeps angle in range -180 < x < 180
    public static double AngleWrap(double angle) {
        return angle % Math.PI;
    }
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double A = 1 + Math.pow(m1, 2);
        double B = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double C = Math.pow(m1, 2) * Math.pow(x1, 2) - 2 * y1 * x1 * m1 + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();
        try {
            double xRoot1 = (-B + Math.sqrt(Math.pow(B, 2) - (4 * A * C))) / (2 * A) + circleCenter.x;
            double yRoot1 = m1 * (xRoot1 - x1) + y1 + circleCenter.y;
            double minX = Math.min(linePoint1.x, linePoint2.x);
            double maxX = Math.max(linePoint1.x, linePoint2.x);

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-B - Math.sqrt(Math.pow(B, 2) - (4 * A * C))) / (2 * A) + circleCenter.x;
            double yRoot2 = m1 * (xRoot1 - x1) + y1 + circleCenter.y;

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception e) {

        }
        return allPoints;
    }
}
