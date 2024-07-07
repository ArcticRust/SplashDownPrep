package org.firstinspires.ftc.teamcode.drive.traj;

import static org.firstinspires.ftc.teamcode.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.drive.traj.Util.AngleWrap;

import com.qualcomm.robotcore.util.Range;

public class RobotMovement {
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
