package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FieldCentricDrive", group="TeleOp")
public class FieldCentricDrive extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            fieldCentricDriveChassis();
            if(gamepad1.dpad_up){
                turnRobot(0);
            }
            else if(gamepad1.dpad_right){
                turnRobot(90);
            }
            else if(gamepad1.dpad_up){
                turnRobot(180);
            }
            if(gamepad1.dpad_left){
                turnRobot(-90);
            }

        }
    }

    private void initialize() {
        // Initialize motors, IMU and other components here
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //liftmotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    private void fieldCentricDriveChassis() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.85;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1;
        rotY = rotY * 1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void rotateRobot(double targetAngle) {
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double endAngle = startAngle + targetAngle;

        // Set motor power to rotate
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);

        // Rotate until the desired angle is reached
        while (opModeIsActive() && !isStopRequested()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if ((targetAngle > 0 && currentAngle >= endAngle) || (targetAngle < 0 && currentAngle <= endAngle)) {
                break;
            }
            idle();
        }

        // Stop motors after rotation
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void turnRobot(double targetAngleDegrees) {
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = initialHeading + targetAngleDegrees;

        // Normalize the target heading to be within -180 to 180 degrees
        targetHeading = AngleUnit.normalizeDegrees(targetHeading);

        // Turn until the robot reaches the target heading
        while (opModeIsActive() && !onTargetHeading(targetHeading)) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double turnPower = calculateTurnPower(currentHeading, targetHeading);

            frontLeftMotor.setPower(turnPower);
            backLeftMotor.setPower(turnPower);
            frontRightMotor.setPower(-turnPower);
            backRightMotor.setPower(-turnPower);

            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Power", turnPower);
            telemetry.update();
        }

        // Stop the motors after the turn
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private boolean onTargetHeading(double targetHeading) {
        double headingError = targetHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        final double HEADING_THRESHOLD = 1.0; // degrees, adjust as necessary
        return Math.abs(headingError) <= HEADING_THRESHOLD;
    }

    private double calculateTurnPower(double currentHeading, double targetHeading) {
        double headingError = targetHeading - currentHeading;
        final double P_TURN_GAIN = 0.01; // Proportional turn gain, adjust as necessary
        double turnPower = headingError * P_TURN_GAIN;

        // Ensure the turn power is within -1.0 to 1.0
        return Range.clip(turnPower, -1.0, 1.0);
    }
}

