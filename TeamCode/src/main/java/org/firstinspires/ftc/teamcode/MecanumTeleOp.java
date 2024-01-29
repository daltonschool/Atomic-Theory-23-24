package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();

    double liftmotorStartingPosition;
    double servoBoxStartingPosition = 0;
    boolean YisPressed;
    boolean boxOpen;
    boolean BisPressed;
    boolean boxRaised;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        //liftmotorStartingPosition = rb.liftmotor.getCurrentPosition();
//        servoBoxStartingPosition = rb.boxServo.getPosition();
        rb.armServo1.setPosition(0.4);
        rb.armServo2.setPosition(0.4);
        rb.boxServo.setPosition(0.2);
        YisPressed = false;
        boxOpen = true;
        BisPressed = false;
        boxRaised = false;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();

//        rb.resetEncoder(rb.liftmotor);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");
        //rb.run(gamepad1);
        driveChassis();
        //slowdriveChassis();
//        moveDuck();
        liftNoEncoder();
        intake2();
        launcher();
        boxservo();
        boxarms();
        pullup();

        telemetry.update();
    }

    //moves the the spinning wheel
//    private void moveDuck() {
//        if(gamepad1.a){
//
//            rb.duckmotor.setPower(-0.8);
//        }
//        if(gamepad1.y){
//            rb.duckmotor.setPower(0.8);
//        }
//        else {
//            rb.duckmotor.setPower(0);
//        }
//    }

    private void lift() {
        if(gamepad2.b){
            rb.liftmotor.setPower(1);
        }
        else if (rb.liftmotor.getCurrentPosition() > 0){
            if(gamepad2.x){
                rb.liftmotor.setPower(-0.5);
            }
            else{
                rb.liftmotor.setPower(0);
            }
        }
        else {
            rb.liftmotor.setPower(0);
        }
        telemetry.addData("Starting at",
                rb.liftmotor.getCurrentPosition());
    }

    private void liftNoEncoder() {
        if (Math.abs(gamepad2.left_stick_y) > 0.5) {
            telemetry.addData("Starting at",
                    rb.liftmotor.getCurrentPosition());
            rb.boxServo.setPosition(0.2);
            boxOpen = true;
            rb.liftmotor.setPower(0.8 * gamepad2.left_stick_y);

        }
        else {
            rb.liftmotor.setPower(0);
        }

    }


    //moves the lift up
    private void intake() {
        if(gamepad2.right_bumper){
            rb.intakemotor.setPower(0.5);
        }
        else if(gamepad2.left_bumper){
            rb.intakemotor.setPower(-0.25);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    private void intake2() {

        if(gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
            rb.intakemotor.setPower((gamepad2.right_trigger + gamepad2.left_trigger)/2);
            rb.boxServo.setPosition(0.8);
            boxOpen = false;
        }

        else if(gamepad2.left_bumper || gamepad2.right_bumper){
            rb.intakemotor.setPower(-0.5);
        }
        else if(gamepad2.left_bumper && gamepad2.right_bumper){
            rb.intakemotor.setPower(-1);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    private void launcher() {
        if(gamepad1.dpad_up) {
            if(gamepad1.y) {
                telemetry.addData("Pressed", gamepad1.y);
                rb.launcher.setPosition(0.6);
                telemetry.addData("Launcher Position", rb.launcher.getPosition());
            }
        }
    }

    private void boxservo() {
        // arm servo
        if (!YisPressed) {
            if (gamepad2.y) {
                YisPressed = true;
                if (boxOpen) {
                    rb.boxServo.setPosition(0.8);
                    boxOpen = false;
                } else {
                    rb.boxServo.setPosition(0.2);
                    boxOpen = true;
                }
            }
        } else{
            if (!gamepad2.y) {
                YisPressed = false;
            }
        }
    }
    private void boxarms() {
        //box arms
        if (!BisPressed) {
            if (gamepad2.x) {
                BisPressed = true;
                if (boxRaised) {
                    rb.armServo1.setPosition(0.4);
                    rb.armServo2.setPosition(0.4);
                    boxRaised = false;
                } else {
                    rb.armServo1.setPosition(0.8);
                    rb.armServo2.setPosition(0.8);
                    boxRaised = true;
                }
            }
        } else{
            if (!gamepad2.x) {
                BisPressed = false;
            }
        }
    }

    private void pullup() {
        if (gamepad2.dpad_up) {
            if (Math.abs(gamepad2.right_stick_y) > 0.5) {
                rb.pMotor1.setPower(gamepad2.right_stick_y);
                rb.pMotor2.setPower(gamepad2.right_stick_y);
            }
            else {
                rb.pMotor1.setPower(0);
                rb.pMotor2.setPower(0);
            }
        }
        else{
            rb.pMotor1.setPower(0);
            rb.pMotor2.setPower(0);
        }
    }




    private void driveChassis() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.85;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x + rx) / denominator;
        double backLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if(gamepad1.right_trigger > 0.5) {
            rb.flMotor.setPower(frontLeftPower * 0.25);
            rb.blMotor.setPower(backLeftPower * 0.25);
            rb.frMotor.setPower(frontRightPower * 0.25);
            rb.brMotor.setPower(backRightPower * 0.25);
        }
        else {
            rb.flMotor.setPower(frontLeftPower);
            rb.blMotor.setPower(backLeftPower);
            rb.frMotor.setPower(frontRightPower);
            rb.brMotor.setPower(backRightPower);
        }
    }
}

