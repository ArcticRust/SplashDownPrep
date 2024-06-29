package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.drive.localizer.Matrix;

@TeleOp(name="default drive")
public class DefaultDriveTest extends OpMode {
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initizalized!");
        telemetry.update();
    }
    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        leftFront.setPower(forward + strafe + turn);
        rightFront.setPower(forward - strafe - turn);
        leftRear.setPower(forward - strafe + turn);
        rightRear.setPower(forward + strafe - turn);

        telemetry.addData("Left Front Power", leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front Power", rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Rear Power", leftRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Rear Power", rightRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Sum", leftFront.getCurrent(CurrentUnit.AMPS) + leftRear.getCurrent(CurrentUnit.AMPS) + rightFront.getCurrent(CurrentUnit.AMPS) + rightRear.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }
}
