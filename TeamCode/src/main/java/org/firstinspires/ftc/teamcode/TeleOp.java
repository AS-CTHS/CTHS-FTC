package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // weather to reverse the direction of the pair power (this can change depending on how the robot is built)
    boolean WHEEL_PAIR_REVERSED = true;


    /* Moves the robot (does not turn)
     * @param direction is the direction the robot should move (360 degrees from the front going clockwise)
     * @param power determines the final power of the motors (between 0 and 1)
     */
    private void move(double direction, double power) {
        // initial definitions
        double wheelsPairOnePower = 0.0;
        double wheelsPairTwoPower = 0.0;

        // Figure out the diagonal direction and set the power accordingly
        if (direction <= 90) { // 0-90 deg
            // Set the power
            wheelsPairOnePower = 1; // front left and back right wheels
            wheelsPairTwoPower = (45.0 - direction) / 45.0; // front right and back left wheels
        } else if (direction <= 180) { // 90-180 deg
            // Set the power
            wheelsPairOnePower = (45.0 - (direction - 90)) / 45.0; // front left and back right wheels
            wheelsPairTwoPower = -1; // front right and back left wheels
        } else if (direction <= 270) { // 0-90 deg
            // Set the power
            wheelsPairOnePower = -1; // front left and back right wheels
            wheelsPairTwoPower = (45.0 - (direction - 180)) / -45.0; // front right and back left wheels
        } else if (direction <= 360) { // 90-180 deg
            // Set the power
            wheelsPairOnePower = (45.0 - (direction - 270)) / -45.0; // front left and back right wheels
            wheelsPairTwoPower = 1; // front right and back left wheels
        } else {
            throw new java.lang.RuntimeException("Invalid input for direction. Must be between 0 and 360. Got: " + direction);
        }

        // Change the power to be relative the input power
        wheelsPairOnePower *= power;
        wheelsPairTwoPower *= power;

        // Set the motors to start moving
        if (WHEEL_PAIR_REVERSED) {
            frontLeft.setPower(wheelsPairTwoPower);
            backRight.setPower(wheelsPairTwoPower);
            frontRight.setPower(wheelsPairOnePower);
            backLeft.setPower(wheelsPairOnePower);
        } else{
            frontLeft.setPower(wheelsPairOnePower);
            backRight.setPower(wheelsPairOnePower);
            frontRight.setPower(wheelsPairTwoPower);
            backLeft.setPower(wheelsPairTwoPower);
        }
    }


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double stickAngle = 0;
        while (opModeIsActive()) {
            // Calculate the angle of the stick {offset + convert_radians_to_degrees(atan(slope))}
            stickAngle = 90 + Math.toDegrees(Math.atan(this.gamepad2.left_stick_y / this.gamepad2.left_stick_x));
            // figure out if it's the left part of the stick, if so correct it by adding 180
            if (this.gamepad2.left_stick_x < 0) {
                stickAngle += 180;
            }
            // check if it's a number, if not, set it to 0
            if (Double.isNaN(stickAngle)) {
                stickAngle = 0;
            }

            // Calculate the power
            tgtPower = Math.sqrt(Math.pow(this.gamepad2.left_stick_x, 2) + Math.pow(this.gamepad2.left_stick_y, 2));
            move(stickAngle, tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Target Direction", stickAngle);
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}