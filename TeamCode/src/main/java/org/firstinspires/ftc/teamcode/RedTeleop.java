package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;

@TeleOp(name = "RedTeleop", group = "Robot")
public class RedTeleop extends OpMode {
    configwLimeLight conf = new configwLimeLight();

    @Override
    public void init() {
        conf.init(hardwareMap);

        telemetry.addLine("Push your robot around to see it track");
        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        telemetry.addData("Right Hue", conf.hsvValuesRight[0]);
        telemetry.addData("Left Hue", conf.hsvValuesLeft[0]);
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        conf.redLED.off();
        conf.greenLED.off();

    }

    @Override
    public void loop()  {
        if(gamepad1.right_bumper){ //Endgame Parking
            conf.odometryDrive(-21.3,48,0, 1);
        }
        else if(gamepad1.dpad_right){ //Far Shooting
            conf.launch_motor_1.setPower(0.7);
            conf.odometryDrive(2.5,2.2,-22, 1);
        }
        else if(gamepad1.dpad_up){ //Opponents goal shooting
            conf.launch_motor_1.setPower(.63);
            conf.odometryDrive(103,48.8,-83.5, 1);
        }
        else if(gamepad1.dpad_left){ //Middle shooting
            conf.launch_motor_1.setPower(.56);
            conf.odometryDrive(66.5,8.9,-45.4, 1);
        }
        else if(gamepad1.dpad_down){ //Close shooting
            conf.launch_motor_1.setPower(.53);
            conf.odometryDrive(87.45,-6.59,-45.4, conf.xMaxSpeed);
        }
        else {
            double front_left_power = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double front_right_power = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double back_right_power = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double back_left_power = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;

            conf.pinpoint.update();
            Pose2D pose2D = conf.pinpoint.getPosition();
            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            conf.max_power = 1;
            conf.max_power = Math.max(conf.max_power, Math.abs(front_left_power));
            conf.max_power = Math.max(conf.max_power, Math.abs(front_right_power));
            conf.max_power = Math.max(conf.max_power, Math.abs(back_right_power));
            conf.max_power = Math.max(conf.max_power, Math.abs(back_left_power));

            //brandt button
            if (gamepad1.left_trigger > 0.5) {
                conf.front_left_drive.setPower(front_left_power / (conf.max_power * 2));
                conf.back_left_drive.setPower(back_left_power / (conf.max_power * 2));
                conf.front_right_drive.setPower(front_right_power / (conf.max_power * 2));
                conf.back_right_drive.setPower(back_right_power / (conf.max_power * 2));
            } else {
                conf.front_left_drive.setPower(front_left_power / conf.max_power);
                conf.back_left_drive.setPower(back_left_power / conf.max_power);
                conf.front_right_drive.setPower(front_right_power / conf.max_power);
                conf.back_right_drive.setPower(back_right_power / conf.max_power);
            }
        }

        //Get Color Sensor Values
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);

        //Reverse kolby cage if full
        if(conf.hsvValuesRight[0] > 140 && conf.hsvValuesLeft[0] > 140 && conf.hsvValuesCenter[0] > 140){

            if (!conf.ColorReadVar){
                conf.ColorReadVar = true;
                conf.colorReadTimer.reset();
            }
            Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
            Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
            Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);
            if (conf.hsvValuesRight[0] > 140 && conf.hsvValuesLeft[0] > 140 && conf.hsvValuesCenter[0] > 140 && conf.colorReadTimer.seconds() > 0.25){
                conf.ColorReadVar = false;
                conf.redLED.off();
                conf.greenLED.off();
                conf.intake_motor.setPower(0);
            }
        }
        //start kolby kage
        else if(gamepad1.right_trigger > .5){
            conf.greenLED.on();
            conf.redLED.off();
            conf.intake_motor.setPower(1);
        }

        //reverse kage
        if(gamepad1.b && conf.intake_var2 && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(-1);
            conf.redLED.on();
            conf.greenLED.off();
            conf.intake_var2 = false;
            conf.intake_timer.reset();
        }
        else if(gamepad1.b && !conf.intake_var2 && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(0);
            conf.redLED.off();
            conf.greenLED.off();
            conf.intake_var2 = true;
            conf.intake_timer.reset();
        }

        //Flywheel launcher
        if (gamepad2.a) {
            conf.launch_motor_1.setPower(.54);
            telemetry.addData("Flywheel on", conf.launch_motor_1.getPower()*100);
        } else if (gamepad2.b) {
            conf.launch_motor_1.setPower(.55);
            telemetry.addData("Flywheel on", conf.launch_motor_1.getPower()*100);
        } else if (gamepad2.x) {
            conf.launch_motor_1.setPower(.60);
            telemetry.addData("Flywheel on", conf.launch_motor_1.getPower()*100);
        } else if (gamepad2.y) {
            conf.launch_motor_1.setPower(.70);
            telemetry.addData("Flywheel on", conf.launch_motor_1.getPower()*100);
        }
        else if (gamepad2.back) {
            telemetry.addLine("Flywheel off");
            conf.launch_motor_1.setPower(0);
        }

        //franklin flipper right
        if(gamepad2.right_trigger > 0.5) {
            conf.franklin_flipper_right.setPosition(0.11);

        }
        else{
            conf.franklin_flipper_right.setPosition(0.44);
        }

        //franklin flipper left
        if(gamepad2.left_trigger > 0.5) {
            conf.franklin_flipper_left.setPosition(1);
        }
        else{
            conf.franklin_flipper_left.setPosition(0.64
            );
        }

    }
}
