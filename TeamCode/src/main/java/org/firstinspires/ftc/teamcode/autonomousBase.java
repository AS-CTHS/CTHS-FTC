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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class autonomousBase extends LinearOpMode {
    // Initial definition
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    double desiredHeading;

    // Constants
    // weather to reverse the direction of the pair power (this can change depending on how the robot is built)
    static final boolean WHEEL_PAIR_REVERSED = true;
    // the amount of direction correction
    static final double PROPORTIONAL_GAIN = 0.05;


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
     * @param time is the time in seconds that it should move that direction
     * @param power determines the final power of the motors (between 0 and 1)
     */
    private void move(double direction, double referenceDirection, double time, double power) {
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
        while (opModeIsActive() && runtime.seconds() < time) {
            // Steering correction
            AddSteeringCorrection(desiredHeading);
        }

        // Stop the motors
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
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
        desiredHeading = getHeading();

        // Drive forward for 1 seconds at max power
        move(0, getHeading(), 1, 1);
        // Drive backward for 5 seconds at max power
        move(180, getHeading(), 1, 1);
    }
}
