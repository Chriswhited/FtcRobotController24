package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.config;

@TeleOp(name = "NewTeleop", group = "Robot")
public class NewTeleop extends OpMode {
    config conf = new config();

    @Override
    public void init() {
        conf.init(hardwareMap);

    }

    @Override
    public void loop()  {
        if(gamepad1.right_bumper){ //Endgame Parking
            conf.odometryDrive(19,-15.75,0, 1);
        }
        else if(gamepad1.dpad_left){ //Far Shooting
            conf.launch_motor_1.setPower(0.7);
            conf.odometryDrive(2.5,2.2,-22, 1);
        }
        else if(gamepad1.dpad_up){ //Opponents goal shooting
            conf.launch_motor_1.setPower(.6);
            conf.odometryDrive(107,53,-84.5, 1);
        }
        else if(gamepad1.dpad_right){ //Middle shooting
            conf.launch_motor_1.setPower(.6);
            conf.odometryDrive(86.5,14.5,-49.5, 1);
        }
        else if(gamepad1.dpad_down){ //Close shooting
            conf.launch_motor_1.setPower(.50);
            conf.odometryDrive(95,-12.3,-55, 1);
        }
        else{
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

        //start kolby kage
        if(gamepad1.a && conf.intake_var && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(1);
            conf.intake_var = false;
            conf.intake_timer.reset();
        }
        else if(gamepad1.a && !conf.intake_var && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(0);
            conf.intake_var = true;
            conf.intake_timer.reset();
        }
        //reverse kolby kage
        if(gamepad1.b && conf.intake_var2 && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(-1);
            conf.intake_var2 = false;
            conf.intake_timer.reset();
        }
        else if(gamepad1.b && !conf.intake_var2 && conf.intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(0);
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
