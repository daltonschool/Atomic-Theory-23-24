package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "BlueCloseAuto")
public class BlueCloseAuto extends LinearOpMode {

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
        telemetry.addData("Shipping Hub Location", level);
        telemetry.update();
//

//
//        telemetry.addData("Shipping Hub Level", level);
//        telemetry.update();

        // Drive to the the shipping hub


        // Deposit the box on the correct level

//       encoder auto CHANGE HERE
        rb.strafeRightByEncoder(-20, rb.frMotor, 0.6);
        sleep(1000);
        liftByEncoder(-800, -0.5);
        rb.armServo1.setPosition(0.8);
        rb.armServo2.setPosition(0.8);
        sleep(4000);
        rb.driveForwardByEncoder(-9, rb.frMotor, -0.3);
        rb.boxServo.setPosition(0.8);
        sleep(3500);
        rb.armServo1.setPosition(0.4);
        rb.armServo2.setPosition(0.4);
        sleep(3000);
        liftByEncoder(0, 0.5);
        sleep(800);




//
//        if(level == 1) { //left
//            rb.liftmotor.setTargetPosition(3000);
//            rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rb.liftmotor.setPower(0.75);
//            Thread.sleep(2000);
//        } else if (level == 2) { //middle
//            rb.strafeRightByEncoder(10, rb.blMotor, 1);
//            rb.liftmotor.setTargetPosition(3000);
//            rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rb.liftmotor.setPower(0.75);
//            Thread.sleep(2000);
//        } else { // right
//            rb.strafeRightByEncoder(20, rb.blMotor, 1);
//            rb.liftmotor.setTargetPosition(3000);
//            rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rb.liftmotor.setPower(0.75);
//            Thread.sleep(2000);
//        }
//
//
//
//
//
//        rb.boxServo.setPosition(0);
//        Thread.sleep(2000);
//        rb.boxServo.setPosition(0.6);
//        Thread.sleep(5000);
//
//        rb.liftmotor.setTargetPosition(1500);
//        rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rb.liftmotor.setPower(0.75);
//
//
//        Thread.sleep(2000);
//
//
//        rb.turnClockwiseByEncoder(-10, rb.blMotor, 0.5);
//        Thread.sleep(100);
        //rb.driveForwardByEncoder(10, rb.blMotor, 1);


//
//       turnClockWiseByTime(1000,-1);
//       driveForwardByTime(2000, 1);
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
}
