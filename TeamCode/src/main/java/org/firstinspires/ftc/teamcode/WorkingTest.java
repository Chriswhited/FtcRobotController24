package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;
import org.firstinspires.ftc.teamcode.mechanisms.testconfig;

import java.util.List;

@TeleOp(name = "WorkingTest", group = "Robot")
public class WorkingTest extends OpMode {

    configwLimeLight conf = new configwLimeLight();

    double forward, strafe, rotate;




    @Override
    public void init() {
        conf.init(hardwareMap);
    }

    public void start(){

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        if(gamepad1.y){ //Opponent Shooting

            AutoOdometryDrive(110,-51,85, 1, gamepad1);

        }

        else if(gamepad1.x){ //Middle shooting
            AutoOdometryDrive(66.5,-8.9,47, 1, gamepad1);

        }
        else if(gamepad1.a){ //Open gate
            AutoOdometryDrive(49.3, 43.15, -120, conf.xMaxSpeed, gamepad1);
        }

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

    public void AutoOdometryDrive(double targetX, double targetY, double targetH, double speed, Gamepad gamepad) {//, String button
        double integralSumX = 0;
        double lastErrorX = 0;
        double integralSumY = 0;
        double lastErrorY = 0;
        conf.xMaxSpeed = speed;
        conf.yMaxSpeed = speed;
        ElapsedTime timer = new ElapsedTime();

        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();

        //pos = myPosition();
        double xError = targetX - pose2D.getX(DistanceUnit.INCH);
        double yError = targetY - pose2D.getY(DistanceUnit.INCH);
        double hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

        while (Math.abs(xError) > 1 || Math.abs(yError) > 1 || Math.abs(hError) > .5 && !gamepad.b) { //&& !gamepad.button

            conf.pinpoint.update();
            pose2D = conf.pinpoint.getPosition();
            xError = targetX - pose2D.getX(DistanceUnit.INCH);
            yError = targetY - pose2D.getY(DistanceUnit.INCH);
            hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

            double derivativeX = (xError - lastErrorX) / timer.seconds();
            integralSumX = integralSumX + (xError * timer.seconds());
            double derivativeY = (yError - lastErrorY) / timer.seconds();
            integralSumY = integralSumY + (yError * timer.seconds());

            double x = Range.clip((conf.xProp * xError) + (conf.xInt * integralSumX) + (conf.xDer * derivativeX), -conf.xMaxSpeed, conf.xMaxSpeed);
            double y = Range.clip((conf.yProp * yError) + (conf.yInt * integralSumY) + (conf.yDer * derivativeY), -conf.yMaxSpeed, conf.yMaxSpeed);
            double h = Range.clip((hError * conf.hProp) + (conf.hDer * conf.derivativeH), -conf.hMaxSpeed, conf.hMaxSpeed);


            conf.moveRobot(x, y, h);

            lastErrorX = xError;
            lastErrorY = yError;
            timer.reset();

        }
        conf.moveRobot(0, 0, 0);
    }

}