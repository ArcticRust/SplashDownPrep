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
    private DcMotorEx leftEncoder, rightEncoder, perpEncoder;
    private final double sideDeviance = 6.4759; //how sideways the parallel encoders are from the center of rotation
    private final double backOffset = 6; //how far back the perp encoder is from the center of rotation
    private final double wheelRadius = 0.7644; //inches, like everything else
    private Matrix pose;
    private double pl, pr, pp;
    /*
    tuning these should be done in this order (can be derived from formulas):
    wheelradius: move forwards 60 in and make sure it does that
    sidedeviance: spin 10 times and make sure it does that
    backoffset: strafe 60 in and make sure it does that
     */
    public Localizer(HardwareMap hardwareMap, Matrix startPose) {
        //Init encoders and set standard settings
        leftEncoder = hardwareMap.get(DcMotorEx.class, "liftOne");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightRear");
        perpEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setDirection(DcMotor.Direction.REVERSE);

        pose = startPose;

        pl = leftEncoder.getCurrentPosition();
        pr = rightEncoder.getCurrentPosition();
        pp = perpEncoder.getCurrentPosition();
    }

    public void update() {
        double l = leftEncoder.getCurrentPosition();
        double r = rightEncoder.getCurrentPosition();
        double p = perpEncoder.getCurrentPosition();

        //basic kinematics formulas, remember x is forward/back
        //as it turns out how forwards the parallel wheels is inconsequential
        double deltaX = wheelRadius * (l - pl + r - pr) / 2.0;
        double deltaY = wheelRadius * (backOffset * (l - pl - r + pr) / (2 * sideDeviance) + p - pp);
        double deltaA = wheelRadius / (2 * sideDeviance) * (r - pr - l + pl);

        //odometry rotation shennanigans
        Matrix deltaPose = new Matrix(new double[]{deltaX, deltaY, deltaA});
        pose = Matrix.toMatrix(pose.add(deltaPose.poseExp(deltaA).rotate(pose.getEntry(2,0))));

        pl = l;
        pr = r;
        pp = p;
    }
    public Matrix getPose() { return pose; }
    public void setPose(Matrix pose) { this.pose = pose; }
}
