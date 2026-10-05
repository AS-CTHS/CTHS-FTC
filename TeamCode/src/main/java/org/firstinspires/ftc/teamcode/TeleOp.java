package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
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

    // weather to reverse the direction of the pair power (this can change depending on how the robot is built)
    static final boolean WHEEL_PAIR_REVERSED = true;
    // the amount of direction correction
    static final double PROPORTIONAL_GAIN = 0.03;


    /** Moves the robot (does not turn)
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
        } else {
            frontLeft.setPower(wheelsPairOnePower);
            backRight.setPower(wheelsPairOnePower);
            frontRight.setPower(wheelsPairTwoPower);
            backLeft.setPower(wheelsPairTwoPower);
        }
    }


    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading(IMU imu) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    /** Determines the amount of correction needed to get the robot to face the desired direction
     * @param desiredHeading is the heading we want to face
     */
    public void AddSteeringCorrection(IMU imu, double desiredHeading) {
        // Determine the heading current error
        double headingError = desiredHeading - getHeading(imu);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double correction = Range.clip(headingError * PROPORTIONAL_GAIN, -1, 1);

        // Add the power to the wheels
        frontLeft.setPower(frontLeft.getPower() + correction);
        backLeft.setPower(backLeft.getPower() + correction);
        frontRight.setPower(frontRight.getPower() - correction);
        backRight.setPower(backRight.getPower() - correction);
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
        double desiredHeading = 0;

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
            AddSteeringCorrection(imu, desiredHeading);
            telemetry.addData("Direction (yaw)", getHeading(imu));
            telemetry.addData("Test", imu.getRobotYawPitchRollAngles());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}