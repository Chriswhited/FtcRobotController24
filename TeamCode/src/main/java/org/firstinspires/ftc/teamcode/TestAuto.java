package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.config;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;

@Autonomous(name = "TestAuto", group = "Robot")
public class TestAuto extends OpMode {
    configwLimeLight conf = new configwLimeLight();

    @Override
    public void init() {

        conf.init(hardwareMap);
        conf.configurePinpoint();
        conf.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    }
    public void init_loop() {
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
        conf.setFlywheelPower(1720);
        conf.sleep(200);
        conf.ReadTag();
        telemetry.addData("id", conf.id);
        telemetry.update();
        conf.limelight.stop();
        conf.AutoOdometryDrive(2.5,2.2,-22, conf.xMaxSpeed);
        while(conf.launch_motor_1.getVelocity() < 1660 && conf.launch_motor_2.getVelocity() < 1660)
        {
            conf.dashboardTelemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
            conf.dashboardTelemetry.update();
        }
        conf.ColorLaunch(conf.id); //Launch PreLoad

        //Intake 2nd Cycle
        conf.AutoOdometryDrive(25,-17,90, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);

        //Get Color Sensor Values
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);

        //Reverse kolby cage if full
        if(conf.hsvValuesRight[0] > 140 || conf.hsvValuesLeft[0] > 140 || conf.hsvValuesCenter[0] > 140){
            conf.AutoOdometryDrive(25,-41,90, 0.4);
            conf.intake_motor.setPower(-1);
            conf.sleep(250);
            conf.intake_motor.setPower(0);
        }
        else{
            conf.AutoOdometryDrive(25,-41,90, 0.4);
            conf.intake_motor.setPower(1);
        }

        //Launch 2nd Cycle
        conf.AutoOdometryDrive(2.5,2.2,-22, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.ColorLaunch(conf.id);
        conf.intake_motor.setPower(0);

        //Intake 3rd Cycle
        conf.AutoOdometryDrive(50,-9,90, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);

        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        Color.RGBToHSV(conf.colorCenter.red() * 8, conf.colorCenter.green() * 8, conf.colorCenter.blue() * 8, conf.hsvValuesCenter);

        //Reverse kolby cage if full
        if(conf.hsvValuesRight[0] > 140 || conf.hsvValuesLeft[0] > 140 || conf.hsvValuesCenter[0] > 140){
            conf.AutoOdometryDrive(50,-41,90, conf.xMaxSpeed);
            conf.intake_motor.setPower(-1);
            conf.sleep(250);
            conf.intake_motor.setPower(0);
        }
        else{
            conf.AutoOdometryDrive(50,-41,90, conf.xMaxSpeed);
            conf.intake_motor.setPower(1);
        }

        //Launch 3rd Cycle
        conf.AutoOdometryDrive(2.5,2.2,-22, conf.xMaxSpeed);
        conf.intake_motor.setPower(1);
        conf.ColorLaunch(conf.id);
        conf.intake_motor.setPower(0);

        //Park
        //AutoOdometryDrive(14,-3,-42, xMaxSpeed); //NEAR PARK
        conf.AutoOdometryDrive(72, -15, 90, conf.xMaxSpeed); //FAR PARK
        //flywheel off




    }
    public void loop(){

    }
}