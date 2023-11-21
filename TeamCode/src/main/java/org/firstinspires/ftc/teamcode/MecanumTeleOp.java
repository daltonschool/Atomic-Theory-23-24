package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();

    double liftmotorStartingPosition;
    double servoBoxStartingPosition = 0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        //liftmotorStartingPosition = rb.liftmotor.getCurrentPosition();
//        servoBoxStartingPosition = rb.boxServo.getPosition();

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
        lift();
        intake2();
        launchservo();
        boxservo();

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
            rb.liftmotor.setPower(0.75);
        }
        else if (rb.liftmotor.getCurrentPosition() > 0){
            if(gamepad2.x){
                rb.liftmotor.setPower(-0.4);
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

        if(gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5){
            rb.intakemotor.setPower((gamepad2.right_trigger + gamepad2.left_trigger)/2);
        }

        else if(gamepad2.left_bumper || gamepad2.right_bumper){
            rb.intakemotor.setPower(-0.25);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    private void launchservo() {
        if(gamepad1.y){
            telemetry.addData("Pressed", gamepad2.y);
            rb.launchservo.setPosition(0.7);
        }
        if(gamepad1.a){
            telemetry.addData("Pressed", gamepad2.y);
            rb.launchservo.setPosition(0.0);
        }
    }

    private void boxservo() {
        if(gamepad2.dpad_up){
            rb.boxServo.setPosition(0);
        }
        else if(gamepad2.dpad_right){
            rb.boxServo.setPosition(0.6);
        }
        else if(gamepad2.dpad_down){
            rb.boxServo.setPosition(1);
        }

    }




    private void driveChassis() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.85;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
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

