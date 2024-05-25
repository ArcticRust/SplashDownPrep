package org.firstinspires.ftc.teamcode.drive.localizer;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {
    private final DcMotor encoder;
    private int offset = 1;
    private int lastTicks, ticks;

    public Encoder(DcMotor encoder) {
        this.encoder = encoder;
        if (encoder.getDirection().equals(DcMotor.Direction.REVERSE)) {
            offset = offset * -1;
        }
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTicks = encoder.getCurrentPosition();
        ticks = encoder.getCurrentPosition();
    }

    public void reverse() {
        offset = offset * -1;
    }

    public void update() {
        lastTicks = offset * encoder.getCurrentPosition();
    }

    public int getChange() {
        ticks = offset * encoder.getCurrentPosition();
        return lastTicks - ticks;
    }

    public int getPosition() {
        return lastTicks;
    }

    public void reset() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastTicks = 0;
        ticks = 0;
    }
}
