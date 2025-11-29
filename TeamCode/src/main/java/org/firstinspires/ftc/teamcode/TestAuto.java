package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.config;

@Autonomous(name = "TestAuto", group = "Robot")
public class TestAuto extends OpMode {
    config conf = new config();

    @Override
    public void init() {

        conf.init(hardwareMap);

    }
    public void init_loop() {
        telemetry.addLine("Push your robot around to see it track");
        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        telemetry.addData("Hue Right", conf.hsvValuesRight[0]);
        telemetry.addData("Hue Left", conf.hsvValuesLeft[0]);
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        conf.franklin_flipper_left.setPosition(.64);
        conf.franklin_flipper_right.setPosition(.44);
    }


    @Override
    public void start(){
        conf.launch_motor_1.setPower(0.9);//.083
        conf.AutoOdometryDrive(2.5,2.2,-22, conf.xMaxSpeed);
        conf.sleep(4000); //spinup flywheel 5200
        conf.launch_motor_1.setPower(0.7);
        conf.sleep(1000);
        conf.launch();
        conf.AutoOdometryDrive(25,-17,90, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.AutoOdometryDrive(25,-46,90, 0.4);
        //intake_motor.setPower(0);
        conf.AutoOdometryDrive(2.5,2.2,-22, conf.xMaxSpeed);
        conf.intake_motor.setPower(0);
        conf.launch();
        conf.AutoOdometryDrive(50,-9,90, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.AutoOdometryDrive(50,-41,90, conf.xMaxSpeed);
        conf.AutoOdometryDrive(2.5,2.2,-22, conf.xMaxSpeed);
        conf.intake_motor.setPower(0);
        conf.launch();
        conf.sleep(550);
        conf.franklin_flipper_right.setPosition(0.11);
        conf.franklin_flipper_left.setPosition(1);
        conf.sleep(550);
        conf.franklin_flipper_left.setPosition(0.64);
        conf.franklin_flipper_right.setPosition(0.44);
        //AutoOdometryDrive(14,-3,-42, xMaxSpeed); //NEAR PARK
        conf.AutoOdometryDrive(72, -15, 90, conf.xMaxSpeed); //FAR PARK
        //flywheel off


    }
    public void loop(){

    }
}