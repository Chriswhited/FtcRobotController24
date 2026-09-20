package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;

@TeleOp(name = "TrackingTest", group = "Robot")
public class TrackingTest extends OpMode {

    configwLimeLight conf = new configwLimeLight();

    double forward, strafe, rotate;

    @Override
    public void init() {
        conf.init(hardwareMap);

        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();

        conf.limelight.start();
    }

    @Override
    public void start() {
        conf.limelight.start();
        conf.limelight.pipelineSwitch(3);
    }

    @Override
    public void loop() {

        LLResult llResult = conf.limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            telemetry.addData("TX offset", llResult.getTx());
            telemetry.addData("TY offset", llResult.getTy());
            telemetry.addData("TA offset", llResult.getTa());
        } else {
            telemetry.addData("Limelight", "No target");
        }

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        
        if (gamepad1.a) {
            followBall();
        } else {
            drive(forward, strafe, rotate);
        }

        telemetry.update();
    }

    public void drive(double forward, double strafe, double rotate){
        double front_left_power = forward + strafe - rotate;
        double front_right_power = forward - strafe + rotate;
        double back_right_power = forward + strafe + rotate;
        double back_left_power = forward - strafe - rotate;

        conf.max_power = 1;
        conf.max_power = Math.max(conf.max_power, Math.abs(front_left_power));
        conf.max_power = Math.max(conf.max_power, Math.abs(front_right_power));
        conf.max_power = Math.max(conf.max_power, Math.abs(back_right_power));
        conf.max_power = Math.max(conf.max_power, Math.abs(back_left_power));

        if (gamepad1.left_trigger > 0.5) {
            conf.front_left_drive.setPower(front_left_power / (conf.max_power * 4));
            conf.back_left_drive.setPower(back_left_power / (conf.max_power * 4));
            conf.front_right_drive.setPower(front_right_power / (conf.max_power * 4));
            conf.back_right_drive.setPower(back_right_power / (conf.max_power * 4));
        }

        else if (gamepad1.right_trigger > 0.5) {
            conf.front_left_drive.setPower(front_left_power / (conf.max_power));
            conf.back_left_drive.setPower(back_left_power / (conf.max_power));
            conf.front_right_drive.setPower(front_right_power / (conf.max_power));
            conf.back_right_drive.setPower(back_right_power / (conf.max_power));
        }

        else {
            conf.front_left_drive.setPower(front_left_power / conf.max_power * 1.5);
            conf.back_left_drive.setPower(back_left_power / conf.max_power * 1.5);
            conf.front_right_drive.setPower(front_right_power / conf.max_power * 1.5);
            conf.back_right_drive.setPower(back_right_power / conf.max_power * 1.5);
        }


    }

    public void followBall() {

        LLResult llResult = conf.limelight.getLatestResult();

        if (llResult == null || !llResult.isValid()) {
            conf.moveRobot(0, 0, 0);
            return;
        }

        double tx = llResult.getTx();

        double x = 0.3;
        double y = 0.3;
        double h = tx * -0.02;

        h = Math.max(-0.3, Math.min(0.3, h));

        conf.moveRobot(x, y, h);

        telemetry.addData("TX", tx);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("H", h);
    }
}