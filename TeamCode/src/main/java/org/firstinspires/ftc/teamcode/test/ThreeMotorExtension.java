package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Three motor extension")
public class ThreeMotorExtension extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorOne, motorTwo, pitch;
        motorOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");
        pitch = hardwareMap.get(DcMotorEx.class, "pitch");
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            double pitchPower = gamepad1.right_stick_y;
            motorOne.setPower(power);
            motorTwo.setPower(power);
            pitch.setPower(pitchPower);
            telemetry.addData("motor one current:", motorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor two current:", motorTwo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("pitch current:", pitch.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
