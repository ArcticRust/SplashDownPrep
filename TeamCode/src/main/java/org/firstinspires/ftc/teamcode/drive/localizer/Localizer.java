package org.firstinspires.ftc.teamcode.drive.localizer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Localizer {
    DcMotorEx leftEncoder, rightEncoder, perpEncoder;
    public Localizer(HardwareMap hardwareMap) {
        //Init encoders and set standard settings
        leftEncoder = hardwareMap.get(DcMotorEx.class, "name1");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "name2");
        perpEncoder = hardwareMap.get(DcMotorEx.class, "name3");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //TODO: change as needed
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update() {

    }
}
