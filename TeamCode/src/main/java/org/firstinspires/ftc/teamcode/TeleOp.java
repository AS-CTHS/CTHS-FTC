/*
* This is the main code for the Autobots team that allows for human driving during the teleop phase.
* Controls (controller 1 only (currently)):
* Left Stick - moving
* Right Stick (left and right) - rotation
* Triangle - Reset heading (used for movement)
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private IMU imu;

    private ElapsedTime lastTurningTime = new ElapsedTime();

    // Constants
    // weather to reverse the direction of the pair power (this can change depending on how the robot is built)
    static final boolean WHEEL_PAIR_REVERSED = true;
    // the amount of direction correction
    static final double PROPORTIONAL_GAIN = 0.05;
    // time in seconds before resetting our heading after you finish turning
    static final double TIME_BEFORE_HEADING_RESET = 0.5;


    /**
     * Converts the input degrees to a positive degree (e.g. -45 to 315).
     * @param inputDegrees is the degrees to convert
     * @return the converted degrees
     */
    private double toPositiveDegrees(double inputDegrees) {
        // Check if it's negative. If so, add 360 to convert it
        if (inputDegrees < 0) {
            return inputDegrees + 360;
        } else {
            return inputDegrees;
        }
    }


    /** Moves the robot (does not turn)
     * @param direction is the direction the robot should move (360 degrees from the front going clockwise)
     * @param referenceDirection is the direction that this assumes the robot is facing
     * @param power determines the final power of the motors (between 0 and 1)
     */
    private void move(double direction, double referenceDirection, double power) {
        // Change the direction to the corrected target direction
        direction = toPositiveDegrees(referenceDirection - direction);

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
        } else {
            frontLeft.setPower(wheelsPairOnePower);
            backRight.setPower(wheelsPairOnePower);
            frontRight.setPower(wheelsPairTwoPower);
            backLeft.setPower(wheelsPairTwoPower);
        }
    }


    /**
     * Adds rotation to the current power.
     * @param power is the amount to add. Positive numbers move right, negative numbers move left.
     */
    private void add_rotation(double power) {
        // Add the power to the wheels
        frontLeft.setPower(frontLeft.getPower() + power);
        backLeft.setPower(backLeft.getPower() + power);
        frontRight.setPower(frontRight.getPower() - power);
        backRight.setPower(backRight.getPower() - power);
    }


    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    /** Determines the amount of correction needed to get the robot to face the desired direction
     * @param desiredHeading is the heading we want to face
     */
    public void AddSteeringCorrection(double desiredHeading) {
        // Determine the heading error
        double headingError = desiredHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double correction = -1 * Range.clip(headingError * PROPORTIONAL_GAIN, -1, 1);

        telemetry.addData("Correction", correction);
        // Add the rotation
        add_rotation(correction);
    }


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse any wheels that need to be reversed
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the imu
        // define the gyro orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // initialize it with this orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Reset the heading
        imu.resetYaw();

        // Set the initial desired heading
        double desiredHeading = getHeading();

        // Set the initial team (WARNING: This is currently unused but can be used later for april tag alignment)
        boolean isBlueAlliance = true;
        // change the controller LED to match
        this.gamepad1.setLedColor(0, 0, 255, 300000);

        // Run until the end of the match (driver presses STOP)
        // initial variables
        double tgtPower = 0;
        double stickAngle = 0;
        boolean waitingForSpinningEnd = false;
        while (opModeIsActive()) {
            // Calculate the angle of the stick {offset + convert_radians_to_degrees(atan(slope))}
            stickAngle = 90 + Math.toDegrees(Math.atan(this.gamepad1.left_stick_y / this.gamepad1.left_stick_x));
            // figure out if it's the left part of the stick, if so correct it by adding 180
            if (this.gamepad1.left_stick_x < 0) {
                stickAngle += 180;
            }
            // check if it's a number, if not, set it to 0
            if (Double.isNaN(stickAngle)) {
                stickAngle = 0;
            }

            // Calculate the power
            tgtPower = Math.sqrt(Math.pow(this.gamepad1.left_stick_x, 2) + Math.pow(this.gamepad1.left_stick_y, 2));
            // Move that direction
            move(stickAngle, getHeading(), tgtPower);

            // Check if the user is wanting to turn
            if (this.gamepad1.right_stick_x != 0) {
                // turn at the target power
                add_rotation(this.gamepad1.right_stick_x);
                // reset heading
                desiredHeading = getHeading();

                // reset the time since we last turned
                lastTurningTime.reset();
                // set that we are waiting to stop spinning
                waitingForSpinningEnd = true;

            // Check if we are waiting to reset the desired direction
            } else if (waitingForSpinningEnd & lastTurningTime.seconds() > TIME_BEFORE_HEADING_RESET) {
                // reset the desired direction
                desiredHeading = getHeading();
                // set that we are no longer waiting to stop spinning
                waitingForSpinningEnd = false;

            // Steering Correction
            } else if (lastTurningTime.seconds() > TIME_BEFORE_HEADING_RESET) {
                // add steering correction
                AddSteeringCorrection(desiredHeading);
            }

            // Check if the user wants to reset the rotation
            if (this.gamepad1.triangleWasReleased()) {
                // reset the heading
                imu.resetYaw();
                // reset the desired direction
                desiredHeading = getHeading();
            }

            // Check if the user is trying to change teams
            if (this.gamepad1.shareWasReleased()) {
                // check if we're on blue
                if (isBlueAlliance) {
                    // change to red
                    isBlueAlliance = false;
                    // match the controller LED
                    this.gamepad1.setLedColor(255, 0, 0, 300000);
                } else {
                    // change to blue
                    isBlueAlliance = true;
                    // match the controller LED
                    this.gamepad1.setLedColor(0, 0, 255, 300000);
                }
            }

            telemetry.addData("Direction (yaw)", getHeading());
            telemetry.addData("Desired Heading", desiredHeading);
            telemetry.addData("On Blue Alliance", isBlueAlliance);
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}