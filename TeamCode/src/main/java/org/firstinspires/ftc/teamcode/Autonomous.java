package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private ElapsedTime runtime = new ElapsedTime();

    // weather to reverse the direction of the pair power (this can change depending on how the robot is built)
    boolean WHEEL_PAIR_REVERSED = true;


    /* Moves the robot (does not turn)
    * @param direction is the direction the robot should move (360 degrees from the front going clockwise)
    * @param time is the time in seconds that it should move that direction
    * @param power determines the final power of the motors (between 0 and 1)
    */
    private void move(double direction, double time, double power) {
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
            throw new java.lang.RuntimeException("Invalid input for direction.");
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
        } else {
            frontLeft.setPower(wheelsPairOnePower);
            backRight.setPower(wheelsPairOnePower);
            frontRight.setPower(wheelsPairTwoPower);
            backLeft.setPower(wheelsPairTwoPower);
        }

        // Keep them running for the desired amount of time
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {}

        // Stop the motors
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
    }


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Drive forward for 5 seconds at max power
        move(0, 5.0, 1);
        // Drive backward for 5 seconds at max power
        move(180, 5.0, 1);
    }
}
