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

@TeleOp(name = "BlueTeleop", group = "Robot")
public class BlueTeleop extends OpMode{
    configwLimeLight conf = new configwLimeLight();

    @Override
    public void init() {
        conf.init(hardwareMap);
        telemetry.addLine("Push your robot around to see it track");
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
    public void loop()  {
        conf.dashboardTelemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        conf.dashboardTelemetry.addData("PID", conf.launch_motor_1.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        PIDFCoefficients PIDF = new PIDFCoefficients(500,0,0,0);
        conf.launch_motor_1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        telemetry.addData("Velocity",conf.launch_motor_1.getVelocity());
        conf.dashboardTelemetry.update();
        //telemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());

        if(!conf.flywheelStart){
            conf.flywheelStart = true;
            conf.setFlywheelPower(1380);
        }
        conf.ledColors(conf.velocity1);
        if(gamepad1.left_bumper) { //Auto Align

            conf.AutoAlign();
            conf.status = conf.status + 1;

        }
        else if(gamepad1.right_bumper){ //Endgame Parking
            conf.odometryDrive(23,-48,90, 1);
        }
        else if(gamepad1.b){ //Far Shooting
            conf.setFlywheelPower(1720);//1680
            conf.ledColors(1720);
            conf.odometryDrive(5,-21.5,28.5, 1); // 2.5,-2.2,22
        }
        else if(gamepad1.y){
            conf.setFlywheelPower(1480);
            conf.ledColors(1480);
            conf.odometryDrive(110,-51,83, 1);
        }
        else if(gamepad1.x){ //Middle shooting
            conf.setFlywheelPower(1380);
            conf.ledColors(1380);
            conf.odometryDrive(66.5,-8.9,45.4, 1);
        }
        else if(gamepad1.a){ //Close shooting
            conf.setFlywheelPower(1260);
            conf.ledColors(1260);
            conf.odometryDrive(87.45,6.59,45.4, conf.xMaxSpeed);
        }
        //else if(gamepad1.left_bumper){
        //conf.AutoAlign();
        //}
        else {
            conf.status = 0;
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

        //Get Color Sensor Values
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);
        Color.RGBToHSV(conf.colorIntake.red() * 8, conf.colorIntake.green() * 8, conf.colorIntake.blue() * 8, conf.hsvValuesIntake);

        //reverse kage
        if(gamepad2.right_bumper){
            conf.intake_motor.setPower(-1);
            conf.intake_timer.reset();
        }
        else if (conf.hsvValuesRight[0] > 140 && conf.hsvValuesLeft[0] > 140 && conf.hsvValuesCenter[0] > 145 && conf.hsvValuesIntake[0] > 145) {
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
        /*
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
        if(gamepad2.right_bumper && conf.intake_var2 && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(-1);
            conf.redLED.on();
            conf.greenLED.off();
            conf.intake_var2 = false;
            conf.intake_timer.reset();
        }
        else if(gamepad2.right_bumper && !conf.intake_var2 && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(0);
            conf.redLED.off();
            conf.greenLED.off();
            conf.intake_var2 = true;
            conf.intake_timer.reset();
        }

         */

        //Flywheel launcher
        if (gamepad2.a) {
            conf.setFlywheelPower(1280);
            conf.ledColors(1280);
            //telemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        } else if (gamepad2.b) {
            conf.setFlywheelPower(1300);
            conf.ledColors(1300);
            //telemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        } else if (gamepad2.x) {
            conf.setFlywheelPower(1420);
            conf.ledColors(1420);
            //telemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        } else if (gamepad2.y) {
            conf.setFlywheelPower(1660);
            conf.ledColors(1660);
            //telemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        }
        else if (gamepad2.back) {
            telemetry.addLine("Flywheel off");
            conf.setFlywheelPower(0);
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
    public void odometryDrive(double targetX, double targetY, double targetH, double speed){
/*
        if(!PIDreset){
            PIDreset = true;
            integralSumX = 0;
            lastErrorX = 0;
            integralSumY = 0;
            lastErrorY = 0;
            xMaxSpeed = speed;
            yMaxSpeed = speed;

            PIDtimer.reset();

            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            xError = targetX - pose2D.getX(DistanceUnit.INCH);
            yError = targetY - pose2D.getY(DistanceUnit.INCH);
            hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);
        }

        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        xError = targetX - pose2D.getX(DistanceUnit.INCH);
        yError = targetY - pose2D.getY(DistanceUnit.INCH);
        hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

        derivativeX = (xError - lastErrorX) / PIDtimer.seconds();
        integralSumX = integralSumX + (xError  * PIDtimer.seconds());
        derivativeY = (yError - lastErrorY) / PIDtimer.seconds();
        integralSumY = integralSumY + (yError  * PIDtimer.seconds());

        double x = Range.clip((xProp * xError) + (xInt * integralSumX) + (xDer * derivativeX),-xMaxSpeed,xMaxSpeed);
        double y = Range.clip((yProp * yError) + (yInt * integralSumY) + (yDer * derivativeY),-yMaxSpeed,yMaxSpeed);
        double h = Range.clip((hError * hProp) + (hDer * derivativeH), -hMaxSpeed, hMaxSpeed);


        moveRobot(x, y, h);

        lastErrorX = xError;
        lastErrorY = yError;
        PIDtimer.reset();

        if(Math.abs(xError) < .0001 && Math.abs(yError) < .0001 && Math.abs(hError) < .5) {
            PIDreset = false;
            moveRobot(0, 0, 0);
        }

 */

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

        while((gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)){

            conf.pinpoint.update();
            pose2D = conf.pinpoint.getPosition();
            xError = targetX - pose2D.getX(DistanceUnit.INCH);
            yError = targetY - pose2D.getY(DistanceUnit.INCH);
            hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

            double derivativeX = (xError - lastErrorX) / timer.seconds();
            integralSumX = integralSumX + (xError  * timer.seconds());
            double derivativeY = (yError - lastErrorY) / timer.seconds();
            integralSumY = integralSumY + (yError  * timer.seconds());

            double x = Range.clip((conf.xProp * xError) + (conf.xInt * integralSumX) + (conf.xDer * derivativeX),-conf.xMaxSpeed,conf.xMaxSpeed);
            double y = Range.clip((conf.yProp * yError) + (conf.yInt * integralSumY) + (conf.yDer * derivativeY),-conf.yMaxSpeed,conf.yMaxSpeed);
            double h = Range.clip((hError * conf.hProp) + (conf.hDer * conf.derivativeH), -conf.hMaxSpeed, conf.hMaxSpeed);


            conf.moveRobot(x, y, h);

            lastErrorX = xError;
            lastErrorY = yError;
            timer.reset();

            conf.dashboardTelemetry.addData("X position", pose2D.getX(DistanceUnit.INCH));
            conf.dashboardTelemetry.addData("Y position", pose2D.getY(DistanceUnit.INCH));
            conf.dashboardTelemetry.addData("X Error", xError);
            conf.dashboardTelemetry.addData("Y Error", yError);
            conf.dashboardTelemetry.addData("H Error", hError);
            conf.dashboardTelemetry.update();

        }
        conf.moveRobot(0, 0, 0);
    }
}
