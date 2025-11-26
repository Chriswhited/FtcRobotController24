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
    double xMaxSpeed = 0.8;
    boolean intake_var;
    boolean intake_var2;
    double max_power = 1.0;
    ElapsedTime intake_timer = new ElapsedTime();

    @Override
    public void init() {
        conf.init(hardwareMap);

    }

    @Override
    public void loop()  {
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

        max_power = 1;
        max_power = Math.max(max_power, Math.abs(front_left_power));
        max_power = Math.max(max_power, Math.abs(front_right_power));
        max_power = Math.max(max_power, Math.abs(back_right_power));
        max_power = Math.max(max_power, Math.abs(back_left_power));

        //brandt button
        if(gamepad1.left_trigger > 0.5){
            conf.front_left_drive.setPower(front_left_power/(max_power*2));
            conf.back_left_drive.setPower(back_left_power/(max_power*2));
            conf.front_right_drive.setPower(front_right_power/(max_power*2));
            conf.back_right_drive.setPower(back_right_power/(max_power*2));
        }
        else {
            conf.front_left_drive.setPower(front_left_power / max_power);
            conf.back_left_drive.setPower(back_left_power / max_power);
            conf.front_right_drive.setPower(front_right_power / max_power);
            conf.back_right_drive.setPower(back_right_power / max_power);
        }



/*
        front_left_drive.setPower(front_left_power / max_power);
        back_left_drive.setPower(back_left_power / max_power);
        front_right_drive.setPower(front_right_power / max_power);
        back_right_drive.setPower(back_right_power / max_power);


 */
        //start kolby kage
        if(gamepad1.a && intake_var && intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(1);
            intake_var = false;
            intake_timer.reset();
        }
        else if(gamepad1.a && !intake_var && intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(0);
            intake_var = true;
            intake_timer.reset();
        }
        //reverse kolby kage
        if(gamepad1.b && intake_var2 && intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(-1);
            intake_var2 = false;
            intake_timer.reset();
        }
        else if(gamepad1.b && !intake_var2 && intake_timer.seconds() > 0.5){
            conf.intake_motor.setPower(0);
            intake_var2 = true;
            intake_timer.reset();
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
    public void configurePinpoint(){
        conf.pinpoint.setOffsets(0.945, 6.5, DistanceUnit.INCH); //Set robot offset
        //pinpoint.setOffsets(-6.5, -0.945, DistanceUnit.INCH); //Set robot offset
        conf.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); //Sets type of pod
        conf.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, //Set direction for pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        conf.pinpoint.resetPosAndIMU();
    }
    public void oldOdometryDrive(double targetX, double targetY, double targetH, double speed){
        double integralSumX = 0;
        double lastErrorX = 0;
        double integralSumY = 0;
        double lastErrorY = 0;
        xMaxSpeed = speed;
        conf.yMaxSpeed = speed;
        ElapsedTime timer = new ElapsedTime();

        conf.pinpoint.update();
        Pose2D pose2D = conf.pinpoint.getPosition();

        //pos = myPosition();
        double xError = targetX - pose2D.getX(DistanceUnit.INCH);
        double yError = targetY - pose2D.getY(DistanceUnit.INCH);
        double hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

        while(opModeIsActive() && Math.abs(xError) > 1.5 || Math.abs(yError) > 1.5 || Math.abs(hError) > 4){

            conf.pinpoint.update();
            pose2D = conf.pinpoint.getPosition();
            xError = targetX - pose2D.getX(DistanceUnit.INCH);
            yError = targetY - pose2D.getY(DistanceUnit.INCH);
            hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

            double derivativeX = (xError - lastErrorX) / timer.seconds();
            integralSumX = integralSumX + (xError  * timer.seconds());
            double derivativeY = (yError - lastErrorY) / timer.seconds();
            integralSumY = integralSumY + (yError  * timer.seconds());

            telemetry.addData("derX", derivativeX);
            telemetry.addData("derY", derivativeY);
            telemetry.addData("IntX", integralSumX);
            telemetry.addData("IntX", integralSumY);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.update();

            double x = Range.clip((conf.xProp * xError) + (conf.xInt * integralSumX) + (conf.xDer * derivativeX),-xMaxSpeed,xMaxSpeed);
            double y = Range.clip((conf.yProp * yError) + (conf.yInt * integralSumY) + (conf.yDer * derivativeY),-conf.yMaxSpeed,conf.yMaxSpeed);
            double h = Range.clip(hError * conf.hProp, -conf.hMaxSpeed, conf.hMaxSpeed);


            conf.moveRobot(x, y, h);

            lastErrorX = xError;
            lastErrorY = yError;
            timer.reset();
        }
        conf.moveRobot(0, 0, 0);
    }
}
