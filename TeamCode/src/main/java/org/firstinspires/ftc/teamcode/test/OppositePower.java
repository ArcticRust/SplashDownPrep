package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Opposite Power")
public class OppositePower extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motorOne, motorTwo;
        motorOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            motorOne.setPower(power);
            motorTwo.setPower(power);
            telemetry.addData("motor one current:", motorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor two current:", motorTwo.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
