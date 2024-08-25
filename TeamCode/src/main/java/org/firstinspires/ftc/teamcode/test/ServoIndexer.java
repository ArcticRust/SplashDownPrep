package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Indexer")
public class ServoIndexer extends LinearOpMode {
    Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "Srvo");
        double servoPosition = 0;
        servo.setPosition(servoPosition);
        waitForStart();
        boolean pad_up = false;
        boolean pad_down = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !pad_up) {
                servoPosition += 0.5;
            }
            if (gamepad1.dpad_down && !pad_down) {
                servoPosition -= 0.5;
            }
            pad_up = gamepad1.dpad_up;
            pad_down = gamepad2.dpad_down;

            servo.setPosition(servoPosition);
            telemetry.addData("Position:", servoPosition);
            telemetry.update();
        }
    }
}
