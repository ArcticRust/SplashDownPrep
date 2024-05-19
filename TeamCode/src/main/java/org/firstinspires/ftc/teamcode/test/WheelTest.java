package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.drive.localizer.Matrix;

@TeleOp(name="wheel test")
public class WheelTest extends OpMode {
    private DcMotorEx leftEncoder, rightEncoder, perpEncoder;
    private final double sideDeviance = 6.4759; //how sideways the parallel encoders are from the center of rotation
    private final double backOffset = 6; //how far back the perp encoder is from the center of rotation
    private final double wheelRadius = 0.7644; //inches, like everything else
    private Matrix pose;
    private double pl, pr, pp;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftEncoder = hardwareMap.get(DcMotorEx.class, "liftOne");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightRear");
        perpEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setDirection(DcMotor.Direction.REVERSE);

        pose = new Matrix();

        pl = leftEncoder.getCurrentPosition();
        pr = rightEncoder.getCurrentPosition();
        pp = perpEncoder.getCurrentPosition();

        telemetry.addLine("Initizalized!");
        telemetry.update();
    }
    @Override
    public void loop() {
        double l = leftEncoder.getCurrentPosition();
        double r = rightEncoder.getCurrentPosition();
        double p = perpEncoder.getCurrentPosition();

        //basic kinematics formulas, remember x is forward/back
        //as it turns out how forwards the parallel wheels is inconsequential
        double deltaX = wheelRadius * (l - pl + r - pr) / 2.0;
        double deltaY = wheelRadius * (backOffset * (l - pl - r + pr) / (2 * sideDeviance) + p - pp);
        double deltaA = wheelRadius / (2 * sideDeviance) * (r - pr - l + pl);

        Matrix deltaPose = new Matrix(new double[]{deltaX, deltaY, deltaA});
        pose = Matrix.toMatrix(pose.add(deltaPose.poseExp(deltaA).rotate(pose.getEntry(2,0))));

        pl = l;
        pr = r;
        pp = p;

        telemetry.addData("left", leftEncoder.getCurrentPosition());
        telemetry.addData("right", rightEncoder.getCurrentPosition());
        telemetry.addData("perp", perpEncoder.getCurrentPosition());

        telemetry.addData("deltax", deltaX);
        telemetry.addData("deltay", deltaY);
        telemetry.addData("deltaa", deltaA);

        telemetry.addData("x", pose.getEntry(0,0));
        telemetry.addData("y", pose.getEntry(1,0));
        telemetry.addData("heading (deg)", pose.getEntry(2,0));

        telemetry.update();
    }
}
