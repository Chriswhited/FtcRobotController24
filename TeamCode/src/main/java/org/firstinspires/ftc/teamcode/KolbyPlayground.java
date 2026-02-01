package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "KolbyPlayground", group = "Robot")
public class KolbyPlayground extends OpMode {
    configwLimeLight conf = new configwLimeLight();

    @Override
    public void init() {
        conf.init(hardwareMap);

        telemetry.addLine("Push your robot around to see it track");
        conf.pinpoint.update();

        Pose2D pose2D = conf.pinpoint.getPosition();
        Color.RGBToHSV(conf.colorRight.red() * 8, conf.colorRight.green() * 8, conf.colorRight.blue() * 8, conf.hsvValuesRight);
        Color.RGBToHSV(conf.colorLeft.red() * 8, conf.colorLeft.green() * 8, conf.colorLeft.blue() * 8, conf.hsvValuesLeft);
        telemetry.update();
        conf.redLED.off();
        conf.greenLED.off();
        conf.limelight.start();
        conf.limelight.pipelineSwitch(3);


    }

    @Override
    public void loop() {;
        PIDFCoefficients PIDF = new PIDFCoefficients(500, 0, 0, 0);
        conf.launch_motor_1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        conf.dashboardTelemetry.update();
        //telemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());

        distanceGoal();
        if (gamepad1.left_bumper) { //Auto Align
            conf.AutoAlign();
        } else {
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
            } else if (gamepad1.right_trigger > 0.5) {
                conf.front_left_drive.setPower(front_left_power / (conf.max_power));
                conf.back_left_drive.setPower(back_left_power / (conf.max_power));
                conf.front_right_drive.setPower(front_right_power / (conf.max_power));
                conf.back_right_drive.setPower(back_right_power / (conf.max_power));
            } else {
                conf.front_left_drive.setPower(front_left_power / conf.max_power * 1.5);
                conf.back_left_drive.setPower(back_left_power / conf.max_power * 1.5);
                conf.front_right_drive.setPower(front_right_power / conf.max_power * 1.5);
                conf.back_right_drive.setPower(back_right_power / conf.max_power * 1.5);
            }
        }
    }
    public void distanceGoal(){
        LLResult llresult = conf.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            conf.idd = fiducial.getFiducialId(); //The ID number of the fiducial
        }
        telemetry.addData("Target X", llresult.getTx());
        telemetry.addData("Target Area", llresult.getTa());

    }
}


