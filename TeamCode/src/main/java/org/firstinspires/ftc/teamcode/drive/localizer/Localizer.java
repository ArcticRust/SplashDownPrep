/**
 * A pose is a 3x1 column matrix with x, y, theta.
 */
package org.firstinspires.ftc.teamcode.drive.localizer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Localizer {
    private Encoder leftEncoder, rightEncoder, perpEncoder;
    private final double sideDeviance = 6.4759 / 1.03123063222; //how sideways the parallel encoders are from the center of rotation
    private final double backOffset = 6 * 1.01174159766; //how far back the perp encoder is from the center of rotation
    private final double wheelRadius = 0.7644 * 0.9038301095; //inches, like everything else
    private Matrix pose;
    double c;
    /*
    tuning these should be done in this order (can be derived from formulas):
    wheelradius: move forwards 60 in and make sure it does that
    sidedeviance: spin 10 times and make sure it does that
    backoffset: strafe 60 in and make sure it does that
     */
    public Localizer(HardwareMap hardwareMap, Matrix startPose) {
        //Init encoders and set standard settings
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftOne"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        perpEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

        leftEncoder.reverse();

        pose = startPose;
        c = 2 * wheelRadius * Math.PI / 8192;
    }

    public void update() {
        double l = leftEncoder.getChange();
        double r = rightEncoder.getChange();
        double p = perpEncoder.getChange();

        //basic kinematics formulas, remember x is forward/back
        //as it turns out how forwards the parallel wheels is inconsequential
        double deltaX = c * (l + r) / 2.0;
        double deltaY = c * (backOffset * (l - r) / (2 * sideDeviance) + p);
        double deltaA = c / (2 * sideDeviance) * (r - l);

        //odometry rotation shennanigans
        Matrix deltaPose = new Matrix(new double[]{deltaX, deltaY, deltaA});
        pose = Matrix.toMatrix(pose.add(deltaPose.poseExp(deltaA).rotate(pose.getEntry(2,0))));
        if (pose.getEntry(2, 0) <= -2 * Math.PI) {
            pose.setEntry(2, 0, pose.getEntry(2,0) + 2 * Math.PI);
        } else if (pose.getEntry(2, 0) >= 2 * Math.PI) {
            pose.setEntry(2, 0, pose.getEntry(2,0) - 2 * Math.PI);
        }
    }
    public Matrix getPose() { return pose; }
    public void setPose(Matrix pose) { this.pose = pose; }
}
