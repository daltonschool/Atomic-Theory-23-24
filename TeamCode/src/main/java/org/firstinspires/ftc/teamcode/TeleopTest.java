package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Chassis;

@TeleOp(name = "TeleOp Test", group = "TeleOp")
public class TeleopTest extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final Chassis chassis = new Chassis();

    @Override
    public void init() {
        chassis.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        chassis.run(gamepad1);
        telemetry.addData("FL: ", chassis.getFrontLeftPosition());
        telemetry.addData("FR: ", chassis.getFrontRightPosition());
        telemetry.addData("BL: ", chassis.getBackLeftPosition());
        telemetry.addData("BR: ", chassis.getBackRightPosition());
        telemetry.addData("Angle: ", chassis.getAngle());
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
