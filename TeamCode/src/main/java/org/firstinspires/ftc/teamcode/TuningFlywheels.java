package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;
import org.firstinspires.ftc.teamcode.mechanisms.testconfig;

@TeleOp(name = "TuningFlywheels", group = "Robot")
public class TuningFlywheels extends OpMode{
    configwLimeLight conf = new configwLimeLight();
    boolean press = false;
    double speedVal1 = 0;
    double speedVal2 = 0;

    @Override
    public void init() {
        conf.init(hardwareMap);
        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);
        Color.RGBToHSV(conf.colorIntake.red() * 8, conf.colorIntake.green() * 8, conf.colorIntake.blue() * 8, conf.hsvValuesIntake);
        telemetry.addData("Right Hue", conf.hsvValuesRight[0]);
        telemetry.addData("Left Hue", conf.hsvValuesLeft[0]);
        telemetry.addData("Center Hue", conf.hsvValuesCenter[0]);
        telemetry.addData("Intake Hue", conf.hsvValuesIntake[0]);
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        conf.redLED.off();
        conf.greenLED.off();
        conf.limelight.start();
        conf.limelight.pipelineSwitch(3);

    }

    @Override
    public void loop() {
        if(speedVal1 < 0){
            speedVal1 = 1280;
        }
        if(speedVal2 < 0){
            speedVal2 = 1280;
        }
        telemetry.addData("V1", conf.launch_motor_1.getVelocity());
        telemetry.addData("V2", conf.launch_motor_2.getVelocity());
        telemetry.addData("P1", conf.launch_motor_1.getPower());
        telemetry.addData("P2", conf.launch_motor_2.getPower());
        telemetry.addData("Goal1", speedVal1);
        telemetry.addData("Goal2", speedVal2);
        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();


        conf.dashboardTelemetry.addData("V1", conf.launch_motor_1.getVelocity());
        conf.dashboardTelemetry.addData("V2", conf.launch_motor_2.getVelocity());
        conf.dashboardTelemetry.addData("P1", conf.launch_motor_1.getPower());
        conf.dashboardTelemetry.addData("P2", conf.launch_motor_2.getPower());
        conf.dashboardTelemetry.addData("Battery", conf.batteryVoltage.getVoltage());


        conf.dashboardTelemetry.update();
        telemetry.update();

        //Intake
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);
        Color.RGBToHSV(conf.colorIntake.red() * 8, conf.colorIntake.green() * 8, conf.colorIntake.blue() * 8, conf.hsvValuesIntake);

        if (conf.hsvValuesRight[0] > 140 && conf.hsvValuesLeft[0] > 140 && conf.hsvValuesCenter[0] > 145 && conf.hsvValuesIntake[0] > 145) {
            if(conf.intake_timer.milliseconds() > 250) {
                conf.intake_motor.setPower(-1);
            }
        }
        else if (conf.hsvValuesRight[0] > 140 && conf.hsvValuesLeft[0] > 140 && conf.hsvValuesCenter[0] > 145 && conf.colorReadTimer.seconds() > 0.25) {
            conf.intake_motor.setPower(0);
            conf.intake_timer.reset();
        }
        else{
            conf.intake_motor.setPower(1);
            conf.intake_timer.reset();
        }

        //franklin flipper right
        if(gamepad1.right_trigger > 0.5) {
            conf.franklin_flipper_right.setPosition(0.11);
        }
        else{
            conf.franklin_flipper_right.setPosition(0.44);
        }

        //franklin flipper left
        if(gamepad1.left_trigger > 0.5) {
            conf.franklin_flipper_left.setPosition(1);
        }
        else{
            conf.franklin_flipper_left.setPosition(0.64);
        }

        //flywheel speed controls
        if(gamepad1.dpad_right){
            speedVal2 += 20;
        }
        else if(gamepad1.dpad_left){
            speedVal2 -= 20;
        }
        else if(gamepad1.dpad_up){
            speedVal1 += 20;
        }
        else if(gamepad1.dpad_down){
            speedVal1 -= 20;
        }
        else if(gamepad1.left_bumper){
            conf.launch_motor_1.setVelocity(0);
            conf.launch_motor_2.setVelocity(0);
        }
        else if(gamepad1.right_bumper){
            conf.launch_motor_1.setVelocity(speedVal1);
            conf.launch_motor_2.setVelocity(speedVal2);
            //conf.launch_motor_1.setPower(1);
            //conf.launch_motor_2.setPower(0);
        }

        //Presents
        if(gamepad1.b){ //Far Shooting
            conf.setFlywheelPower(1740);//1680
            conf.ledColors(1740);
            conf.odometryDrive(5,21.5,-34, 1); // 2.5,-2.2,22
        }
        else if(gamepad1.y){
            conf.setFlywheelPower(1480);
            conf.ledColors(1480);
            conf.odometryDrive(110,51,-85, 1);
        }
        else if(gamepad1.x){ //Middle shooting
            conf.setFlywheelPower(1420);
            conf.ledColors(1380);
            conf.odometryDrive(66.5,8.9,-47, 1);
        }
        else if(gamepad1.a){ //Open gate
            conf.odometryDrive(49.3, -43.15, 120, conf.xMaxSpeed);
        }

        //Drive
        else {
            conf.status = 0;
            double front_left_power = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double front_right_power = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double back_right_power = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double back_left_power = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;


            conf.max_power = 1;
            conf.max_power = Math.max(conf.max_power, Math.abs(front_left_power));
            conf.max_power = Math.max(conf.max_power, Math.abs(front_right_power));
            conf.max_power = Math.max(conf.max_power, Math.abs(back_right_power));
            conf.max_power = Math.max(conf.max_power, Math.abs(back_left_power));

            conf.front_left_drive.setPower(front_left_power / conf.max_power * 1.5);
            conf.back_left_drive.setPower(back_left_power / conf.max_power * 1.5);
            conf.front_right_drive.setPower(front_right_power / conf.max_power * 1.5);
            conf.back_right_drive.setPower(back_right_power / conf.max_power * 1.5);

        }


    }

}
