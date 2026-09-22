package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public void opModeIsActive() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft.setPower(tgtPower);

        /*
        double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower = -this.gamepad2.left_stick_y;
            frontLeft.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", frontLeft.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }*/
    }
}
