package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    //private BHI260IMU imu = null;
    private BNO055IMU imu;

    public Chassis() {}

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(Gamepad gamepad) {
        //Get the positions of the left stick in terms of x and y
        //Invert y because of the input from the controller
        double stickX = Math.abs(gamepad.left_stick_x) < Constants.STICK_THRESH ? 0 : gamepad.left_stick_x;
        double stickY = Math.abs(gamepad.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad.left_stick_y;
        //get the direction from the IMU
        double angle = imu.getAngularOrientation().firstAngle;
        //rotate the positions to prep for wheel powers
        double newSin = (stickY * Math.cos(-PI / 4 - angle)) + (stickX * Math.sin(-PI / 4-angle));
        double newCos = (stickX * Math.cos(-PI / 4- angle)) - (stickY * Math.sin(-PI / 4-angle));
        //determine how much the robot should turn
        double rotation = (gamepad.left_trigger - gamepad.right_trigger) * Constants.ROTATION_SENSITIVITY + (-gamepad.right_stick_x * Constants.ROTATION_SENSITIVITY);

        //test if the robot should move
        double stickPower = Math.sqrt(newSin * newSin + newCos * newCos);
        boolean areTriggersDown = Math.abs(rotation) > 0;
        boolean areSticksMoved = stickPower > 0;
        if (areSticksMoved || areTriggersDown) {
            //add the rotation to the powers of the wheels
            double motorMax = Math.max(Math.abs(newSin)+rotation, Math.abs(newCos)+rotation);
            double proportion = Math.max(1, motorMax);
            double num = (1 / proportion) * stickPower;
            double flPower = num * newCos - rotation;
            double brPower = num * newCos + rotation;
            double frPower = num * newSin + rotation;
            double blPower = num * newSin - rotation;
            //keep the powers proportional and within a range of -1 to 1
            frontLeft.setPower(flPower / proportion);
            backRight.setPower(brPower / proportion);
            frontRight.setPower(frPower / proportion);
            backLeft.setPower(blPower / proportion);
        } else {
            stopDrive();
        }
        if (gamepad.dpad_up) {
            rotateToZero(0,2);
        }
        if (gamepad.dpad_left) {
            rotateToZero(PI/2,2);
        }
        if (gamepad.dpad_down) {
            rotateToZero(PI,2);
        }
        if (gamepad.dpad_right) {
            rotateToZero(-PI/2,2);
        }

    }

    public void stopDrive() {
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
    }

    public void rue(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateToZero(double angle, double timeout) {
        System.out.println("running");
        rue();
        //use a PID control loop to zero
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double k_p = Math.PI/8;
        double k_i = 0;
        double k_d = 2;
        double current_error = imu.getAngularOrientation().firstAngle - angle;
        double previous_error = current_error;
        double previous_time = 0;
        double current_time;
        double max_i = 0.1;
        //while(timer.seconds() < 5){
        while (Math.abs(imu.getAngularOrientation().firstAngle - angle) > Constants.TOLERANCE && timer.seconds() < timeout) {
            current_time = timer.milliseconds();
            current_error = angle - imu.getAngularOrientation().firstAngle;
            double p = k_p * current_error;
            double i = k_i * (current_error * (current_time - previous_time));
            i = Range.clip(i, -max_i, max_i);
            double d = k_d * ((current_error - previous_error) / (current_time - previous_time));
            double power = p + i + d;
            System.out.println(power);
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
            previous_error = current_error;
            previous_time = current_time;
        }

        stopDrive();
    }
    public double getAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public double getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public double getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    public double getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public double getBackRightPosition() {
        return backRight.getCurrentPosition();
    }
}
