package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.config;

@Autonomous(name = "WorkingTest", group = "Robot")

public class WorkingTest extends OpMode {
    config conf = new config();
    double xMaxSpeed = 0.8;

    @Override
    public void init() {
        conf.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addLine("working");

        conf.launch_motor_1.setPower(0.9);//.083
        conf.odometryDrive(2.5,2.2,-22, xMaxSpeed);
        conf.sleep(4); //spinup flywheel 5200
        conf.launch_motor_1.setPower(0.7);
        conf.sleep(1);
        conf.launch();
        conf.odometryDrive(25,-17,90, xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.odometryDrive(25,-46,90, 0.4);
        //intake_motor.setPower(0);
        conf.odometryDrive(2.5,2.2,-22, xMaxSpeed);
        conf.intake_motor.setPower(0);
        conf.launch();
        conf.odometryDrive(50,-17,90, xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.odometryDrive(50,-45,90, xMaxSpeed);
        conf.odometryDrive(2.5,2.2,-22, xMaxSpeed);
        conf.intake_motor.setPower(0);
        conf.launch();
        conf.sleep(.55);
        conf.franklin_flipper_right.setPosition(0.11);
        conf.franklin_flipper_left.setPosition(1);
        conf.sleep(.55);
        conf.franklin_flipper_left.setPosition(0.64);
        conf.franklin_flipper_right.setPosition(0.44);
        //odometryDrive(14,-3,-42, xMaxSpeed); //NEAR PARK
        conf.odometryDrive(72, -15, 90, xMaxSpeed); //FAR PARK
        //flywheel off
    }
}
