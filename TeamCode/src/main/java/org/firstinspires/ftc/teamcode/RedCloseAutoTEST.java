package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "RedCloseAuto")
public class RedCloseAutoTEST extends LinearOpMode {
    // For April TAG navigation
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    double strafe;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webcam;
    OpenCvWebcam frontWebcam;

    MecanumRobot rb = new MecanumRobot();

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, this);

        rb.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rb.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rb.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        PixelRecognizer pipeline = new PixelRecognizer();
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

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
        double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise

        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for the driver to press Start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        waitForStart();
        //lift encoder numbers
        //bottom is 1200
        // middle is 3200

        runtime.reset();
//
        int level;
        int[] counts = {0,0,0};
        for(int i=0;i<50;i++) {
            if(pipeline.getShippingHubLevel() == 0) {
                i = 0;
                continue;
            }
            counts[pipeline.getShippingHubLevel() - 1] ++;
        }

        if(counts[0] > counts[1] && counts[0] > counts[2]) {
            level = 1; //left
        } else if(counts[1] > counts[0] && counts[1] > counts[2]) {
            level = 2; //middle
        } else {
            level = 3; //right
        }
        telemetry.addData("Location", level);
        telemetry.update();



        // Drive to the the pixel spot
        if(level == 1){ // left

        } else if(level == 2){ // middle

        } else { // right

        }

        //if statements are extra here but help with the blue side when the april tags are 4,5, and 6 so its easier
        if(level == 1){
            DESIRED_TAG_ID = 1;
        } else if(level == 2){ // middle
            DESIRED_TAG_ID = 2;
        } else { // right
            DESIRED_TAG_ID = 3;
        }

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData(">","Drive using joysticks to find valid target\n");
            }

            // If we have found the desired target, Drive to target Automatically .
            if (targetFound) {
                // Determine heading and range error so we can use them to control the robot automatically.
                double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError = desiredTag.ftcPose.bearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = 0;

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }

        //then lift and place



    }

    void liftByEncoder(double encoder, double power) {
        double currposition = rb.liftmotor.getCurrentPosition();
        while (Math.abs(currposition - encoder) > 10){
            rb.liftmotor.setPower(power);

            currposition -= (currposition - encoder)/(Math.abs(currposition - encoder));

            telemetry.addData("pos", currposition);
            telemetry.update();

        }
        rb.driveStop();
    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate power for each wheel
        double frontLeftPower  = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower   = x + y - yaw;
        double backRightPower  = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels
        rb.flMotor.setPower(frontLeftPower);
        rb.frMotor.setPower(frontRightPower);
        rb.blMotor.setPower(backLeftPower);
        rb.brMotor.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
}
