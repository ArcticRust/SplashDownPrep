package org.firstinspires.ftc.teamcode.drive.localizer;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {
    private final DcMotor encoder;
    private int offset = 1;
    private int ticks;

    public Encoder(DcMotor encoder) {
        this.encoder = encoder;
        if (encoder.getDirection().equals(DcMotor.Direction.REVERSE)) {
            offset = offset * -1;
        }
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticks = encoder.getCurrentPosition();
    }

    public void reverse() {
        offset = offset * -1;
    }

    public int getChange() {
        int l = ticks;
        ticks = offset * encoder.getCurrentPosition();
        return ticks - l;
    }

    public int getPosition() {
        return ticks;
    }

    public void reset() {
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticks = 0;
    }
}
