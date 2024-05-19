package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ExtensionCurrent extends LinearOpMode {
    DcMotorEx extension;
    double x1, y1, x2;
    @Override
    public void runOpMode() {
        extension = hardwareMap.get(DcMotorEx.class, "extension");

        waitForStart(); // init blocker
        // random code 2
        while (opModeIsActive()) {

           /*leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightRear.setDirection(DcMotor.Direction.FORWARD);*/
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;
            extension.setPower(x1);

            telemetry.addData("extension current: ", extension.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}