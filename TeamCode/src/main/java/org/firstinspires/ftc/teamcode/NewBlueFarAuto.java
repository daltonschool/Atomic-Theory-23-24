package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="NewBlueFarAuto", group="Robot")

public class NewBlueFarAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webcam;
    OpenCvWebcam frontWebcam;

    MecanumRobot rb = new MecanumRobot();

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* Declare OpMode members. */
    private DcMotor bl = null, br = null, fr = null, fl = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  flSpeed     = 0;
    private double  frSpeed    = 0;
    private double  blSpeed     = 0;
    private double  brSpeed    = 0;
    private int flTarget = 0;
    private int frTarget = 0;
    private int blTarget = 0;
    private int brTarget = 0;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // or 383.6 I think
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    int level;


    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        // The next 3 lines define Hub orientation.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int[] pos = {0, 0};
        int[] finalPos = {0, 0};
        int cameraTicks = 0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        PixelRecognizerNew pipeline = new PixelRecognizerNew();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {


            @Override
            public void onOpened() {
                webcam.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Status", "Webcam on");

                telemetry.update();

            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
            telemetry.addData("Status", "Initializing");

            rb.init(hardwareMap, this);

            rb.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rb.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rb.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Status", "Initialized");
            if (cameraTicks < 120) {
                pos[0] = pipeline.getPixelPosX();
                pos[1] = pipeline.getPixelPosY();
                cameraTicks++;
            }
            else {
                telemetry.addData("Initial Position:", pos[0]);
                telemetry.update();
            }

            // Set the encoders for closed loop speed control, and reset the heading.
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            imu.resetYaw();
        }

        waitForStart();

        runtime.reset();

        cameraTicks = 0;
        while (cameraTicks < 120) {
            finalPos[0] = pipeline.getPixelPosX();
            finalPos[1] = pipeline.getPixelPosY();
            cameraTicks++;
            telemetry.addData("Calibrating...", finalPos[0]);
            telemetry.update();
            sleep(15);
        }

        level = pipeline.getPixelFieldPos(pos, finalPos);
        telemetry.addData("Final Position:", finalPos[0]);
        telemetry.addData("Team Element Location", level);
        telemetry.update();



        if (level == 1) {
            driveStraight(DRIVE_SPEED, -20.5, 0.0);

            turnToHeading(TURN_SPEED, 90.0);
            driveStraight(DRIVE_SPEED, -5.0, 90.0);

            driveStraight(DRIVE_SPEED, 5.0, 90.0);

            turnToHeading(TURN_SPEED, 0);

            driveStraight(DRIVE_SPEED, 18.0, 0);

            turnToHeading(TURN_SPEED, 90.0);
            driveStraight(DRIVE_SPEED, -50.0, 90.0);
            strafeRightFixed(DRIVE_SPEED, -15.0, 90.0);
            liftByEncoder(1700);
            rb.armServo1.setPosition(0.8);
            rb.armServo2.setPosition(0.8);
            sleep(1000);
            driveStraight(DRIVE_SPEED, -12.0, 90.0);
            rb.boxServo.setPosition(0.8);
            sleep(1500);
            liftByEncoder(2500);
            rb.armServo1.setPosition(0.4);
            rb.armServo2.setPosition(0.4);
            sleep(1000);
            liftByEncoder(0);
            sleep(800);
            strafeRightFixed(DRIVE_SPEED, -25.0, 90.0);

        } else if (level == 2) {

            driveStraight(DRIVE_SPEED, -21.5, 0.0);

            driveStraight(DRIVE_SPEED, 19.0, 0);

            turnToHeading(TURN_SPEED, 90.0);
            driveStraight(DRIVE_SPEED, -50.0, 90.0);
            strafeRightFixed(DRIVE_SPEED, -20.0, 90.0);
            liftByEncoder(1700);
            rb.armServo1.setPosition(0.8);
            rb.armServo2.setPosition(0.8);
            sleep(1000);
            driveStraight(DRIVE_SPEED, -12.0, 90.0);
            rb.boxServo.setPosition(0.8);
            sleep(1500);
            liftByEncoder(2500);
            rb.armServo1.setPosition(0.4);
            rb.armServo2.setPosition(0.4);
            sleep(1000);
            liftByEncoder(0);
            sleep(800);
            strafeRightFixed(DRIVE_SPEED, -20.0, 90.0);

        } else {
            turnToHeading(TURN_SPEED, -12.5);

            driveStraight(DRIVE_SPEED, -17.5, -12.5);

            driveStraight(DRIVE_SPEED, 15.0, -12.5);

            turnToHeading(TURN_SPEED, 0);

            turnToHeading(TURN_SPEED, 90.0);
            driveStraight(DRIVE_SPEED, -50.0, 90.0);
            strafeRightFixed(DRIVE_SPEED, -25.0, 90.0);
            liftByEncoder(1700);
            rb.armServo1.setPosition(0.8);
            rb.armServo2.setPosition(0.8);
            sleep(1000);
            driveStraight(DRIVE_SPEED, -12.0, 90.0);
            rb.boxServo.setPosition(0.8);
            sleep(1500);
            liftByEncoder(2500);
            rb.armServo1.setPosition(0.4);
            rb.armServo2.setPosition(0.4);
            sleep(1000);
            liftByEncoder(0);
            sleep(800);
            strafeRightFixed(DRIVE_SPEED, -15.0, 90.0);
        }



//        driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
//        turnToHeading( TURN_SPEED,  45.0);               // Turn  CCW  to  45 Degrees
//        holdHeading( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
//
//        driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
//        turnToHeading( TURN_SPEED,   0.0);               // Turn  CW  to 0 Degrees
//        holdHeading( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second
//
//        driveStraight(DRIVE_SPEED,-48.0, 0.0);    // Drive in Reverse 48" (should return to approx. staring position)
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            flTarget = fl.getCurrentPosition() + moveCounts;
            frTarget = fr.getCurrentPosition() + moveCounts;
            blTarget = bl.getCurrentPosition() + moveCounts;
            brTarget = br.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            fl.setTargetPosition(flTarget);
            fr.setTargetPosition(frTarget);
            bl.setTargetPosition(blTarget);
            br.setTargetPosition(brTarget);


            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed, 0);

                sendTelemetry(true);
            }

            moveRobot(0, 0, 0);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafeRightFixed(double maxDriveSpeed,
                                 double distance,
                                 double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            flTarget = fl.getCurrentPosition() + moveCounts;
            frTarget = fr.getCurrentPosition() - moveCounts;
            blTarget = bl.getCurrentPosition() - moveCounts;
            brTarget = br.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            fl.setTargetPosition(flTarget);
            fr.setTargetPosition(frTarget);
            bl.setTargetPosition(blTarget);
            br.setTargetPosition(brTarget);


            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed, 0);

                sendTelemetry(true);
            }

            moveRobot(0, 0, 0);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafeRight(double maxDriveSpeed, double distance, double heading) {

        if (opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            flTarget = fl.getCurrentPosition() + moveCounts;
            frTarget = fr.getCurrentPosition() - moveCounts;
            blTarget = bl.getCurrentPosition() - moveCounts;
            brTarget = br.getCurrentPosition() + moveCounts;

            fl.setTargetPosition(flTarget);
            fr.setTargetPosition(frTarget);
            bl.setTargetPosition(blTarget);
            br.setTargetPosition(brTarget);


            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(0,0, maxDriveSpeed);

            while (opModeIsActive() &&
                    (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {


                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1.0;
                moveRobot(0, turnSpeed, driveSpeed);
                sendTelemetry(true);
            }


            moveRobot(0, 0, 0);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void turnToHeading(double maxTurnSpeed, double heading) {


        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed, 0);
            sendTelemetry(false);
        }
        moveRobot(0, 0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn, double strafe) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        flSpeed  = drive - turn - strafe;
        frSpeed = drive + turn + strafe;
        blSpeed = drive - turn + strafe;
        brSpeed = drive + turn - strafe;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(flSpeed), Math.abs(frSpeed)), Math.max(Math.abs(blSpeed), Math.abs(brSpeed)));
        if (max > 1.0)
        {
            flSpeed /= max;
            frSpeed /= max;
            blSpeed /= max;
            brSpeed /= max;
        }

        fl.setPower(flSpeed);
        fr.setPower(frSpeed);
        bl.setPower(blSpeed);
        br.setPower(brSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
//            telemetry.addData("Motion", "Driving");
//            telemetry.addData("Target Pos L:R",  "%7d:%7d", flTarget, frTarget);
//            telemetry.addData("Actual Pos L:R",  "%7d:%7d", fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition());
        } else {
//            telemetry.addData("Motion", "Turning");
        }

//        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
//        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
//        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", flSpeed, frSpeed, blSpeed, brSpeed);
//        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);

    }
    void liftByEncoder(int encoder) {
        rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.liftmotor.setTargetPosition(encoder);

        rb.liftmotor.setPower(0.4);

        while ((rb.liftmotor.isBusy())) {
        }
        rb.liftmotor.setPower(0);
    }
}