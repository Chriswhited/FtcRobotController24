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

@TeleOp(name = "LaumchTest", group = "Robot")
public class LaumchTest extends OpMode {
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
    public void loop() {
        //conf.dashboardTelemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        //conf.dashboardTelemetry.addData("PID", conf.launch_motor_1.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        //PIDFCoefficients PIDF = new PIDFCoefficients(500, 0, 0, 0);
        //conf.launch_motor_1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        //telemetry.addData("Velocity 1", conf.launch_motor_1.getVelocity());
        //telemetry.addData("Velocity 2", conf.launch_motor_2.getVelocity());
        telemetry.addData("V 1", conf.launch_motor_1.getVelocity());
        telemetry.addData("V 2", conf.launch_motor_2.getVelocity());
        conf.dashboardTelemetry.update();

        if (gamepad1.b) { //Far Shooting
            conf.setFlywheelPower(1740);
            //conf.launch_motor_1.setPower(1);
            //conf.launch_motor_2.setPower(1);
            conf.ledColors(1740);
        } else if (gamepad1.y) {
            conf.setFlywheelPower(1480);
            conf.ledColors(1480);
        }
    }
}
