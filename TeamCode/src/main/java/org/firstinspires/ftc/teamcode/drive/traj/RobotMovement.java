package org.firstinspires.ftc.teamcode.drive.traj;

import static org.firstinspires.ftc.teamcode.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.drive.traj.Util.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.traj.Util.lineCircleIntersection;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        for (int i = 0; i < allPoints.size() - 1; i++) {

        }
    }
    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation,
                                                double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);
            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;
            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(Util.AngleWrap(angle - worldAngle_rad));
                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

        }

        return followMe;
    }
    public static double[] goToPosition(double x, double y, double movementSpeed, double turnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + preferredAngle;
        if (distanceToTarget < 5) turnSpeed = 0;

        return new double[]{movementSpeed * relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)),
                            movementSpeed * relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)),
                            turnSpeed * Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1)};
    }
}
