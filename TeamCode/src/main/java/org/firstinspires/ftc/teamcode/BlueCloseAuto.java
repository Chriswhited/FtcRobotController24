package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.config;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;

@Autonomous(name = "BlueCloseAuto", group = "Robot")
public class BlueCloseAuto extends OpMode {
    configwLimeLight conf = new configwLimeLight();

    @Override
    public void init() {

        conf.init(hardwareMap);
        conf.configurePinpoint();

    }
    public void init_loop() {
        conf.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 119.15, 30.68, AngleUnit.DEGREES, 51));
        telemetry.addLine("Push your robot around to see it track");
        conf.pinpoint.update();
        //conf.limelight.start();
        //conf.ReadTag();
        Pose2D pose2D = conf.pinpoint.getPosition();
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        telemetry.addData("Right Hue", conf.hsvValuesRight[0]);
        telemetry.addData("Left Hue", conf.hsvValuesLeft[0]);
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        //telemetry.addData("id", conf.id);
        telemetry.update();
        conf.franklin_flipper_left.setPosition(.64);
        conf.franklin_flipper_right.setPosition(.44);
        conf.redLED.off();
        conf.greenLED.off();
    }


    @Override
    public void start(){
        conf.limelight.start();
        conf.launch_motor_1.setPower(.54);
        conf.AutoOdometryDrive(87.45,6.59,-25.7, conf.xMaxSpeed);
        conf.ReadTag();
        conf.sleep(2000);
        telemetry.addData("id", conf.id);
        telemetry.update();
        conf.limelight.stop();
        conf.AutoOdometryDrive(87.45,6.59,45.4, conf.xMaxSpeed);
        conf.ColorLaunch(conf.id); //Launch PreLoad

        conf.AutoOdometryDrive(71.73,10.13,-90, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);

        //Get Color Sensor Values
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);

        //Reverse kolby cage if full
        if(conf.hsvValuesRight[0] > 140 || conf.hsvValuesLeft[0] > 140 || conf.hsvValuesCenter[0] > 140){
            conf.AutoOdometryDrive(71.73,35.32,-90, 0.4);
            conf.intake_motor.setPower(-1);
            conf.sleep(250);
            conf.intake_motor.setPower(0);
        }
        else{
            conf.AutoOdometryDrive(71.73,35.32,-90, 0.4);
            conf.intake_motor.setPower(1);
        }

        //Launch 2nd Cycle
        conf.AutoOdometryDrive(87.45,6.59,45.4, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.ColorLaunch(conf.id);
        conf.intake_motor.setPower(0);

        //Intake 3rd Cycle
        conf.AutoOdometryDrive(48.39,10.13,-90, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);

        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);

        //Reverse kolby cage if full
        if(conf.hsvValuesRight[0] > 140 || conf.hsvValuesLeft[0] > 140 || conf.hsvValuesCenter[0] > 140){
            conf.AutoOdometryDrive(48.39,40.98,-90, conf.xMaxSpeed);
            conf.intake_motor.setPower(-1);
            conf.sleep(250);
            conf.intake_motor.setPower(0);
        }
        else{
            conf.AutoOdometryDrive(48.39,40.98,-90, conf.xMaxSpeed);
            conf.intake_motor.setPower(1);
        }

        //Launch 3rd Cycle
        conf.AutoOdometryDrive(87.45,6.59,45.4, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.ColorLaunch(conf.id);
        conf.intake_motor.setPower(0);

        conf.AutoOdometryDrive(68.14,10.18,45.4, conf.xMaxSpeed);
        conf.launch_motor_1.setPower(0);

    }
    public void loop(){

    }
}