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
        intake();
        //servo();

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
        if(gamepad1.b){
            rb.liftmotor.setPower(1);
        }
        else if(gamepad1.x){
            rb.liftmotor.setPower(-1);
        }
        else {
            rb.liftmotor.setPower(0);
        }
    }


    //moves the lift up
    private void intake() {
        if(gamepad1.right_bumper){
            rb.intakemotor.setPower(1);
        }
        else if(gamepad1.left_bumper){
            rb.intakemotor.setPower(-1);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    private void servo() {
        if(gamepad1.y){
            rb.launchservo.setPosition(0.5);
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

        rb.flMotor.setPower(frontLeftPower);
        rb.blMotor.setPower(backLeftPower);
        rb.frMotor.setPower(frontRightPower);
        rb.brMotor.setPower(backRightPower);
    }

    private void slowdriveChassis() {
        double scale = gamepad1.right_trigger > 0 ? 0.25 : 1;

        double y = -gamepad1.left_stick_y * 0.3 * scale;
        double x = gamepad1.left_stick_x * 0.3 * scale;
        double rx = gamepad1.right_stick_x * 0.35 * scale;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        rb.flMotor.setPower(frontLeftPower);
        rb.blMotor.setPower(backLeftPower);
        rb.frMotor.setPower(frontRightPower);
        rb.brMotor.setPower(backRightPower);
    }
}

