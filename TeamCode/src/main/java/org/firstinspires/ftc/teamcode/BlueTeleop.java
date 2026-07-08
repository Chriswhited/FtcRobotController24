package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;
import org.firstinspires.ftc.teamcode.mechanisms.testconfig;

import java.util.List;

@TeleOp(name = "BlueTeleop", group = "Robot")
public class BlueTeleop extends OpMode {

    configwLimeLight conf = new configwLimeLight();

    double kP = 0.0150;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.5;
    double kD = 0.00001;
    double curTime = 0;
    double lastTime = 0;
    int idgoal = 0;

    double forward, strafe, rotate;




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
        conf.flag.setPosition(0);

    }

    public void start(){
        resetRuntime();
        curTime = getRuntime();
        conf.limelight.start();
        conf.limelight.pipelineSwitch(3);
    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        conf.dashboardTelemetry.addData("Flywheel on", conf.launch_motor_1.getVelocity());
        conf.dashboardTelemetry.addData("PID", conf.launch_motor_1.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        telemetry.addData("Velocity 1", conf.launch_motor_1.getVelocity());
        telemetry.addData("Velocity 2", conf.launch_motor_2.getVelocity());
        conf.dashboardTelemetry.update();

        boolean wantDrive = true;

        if (!conf.flywheelStart) { //Initially start flywheel
            conf.flywheelStart = true;
            conf.setFlywheelPower(1380);
            conf.flagTimer.startTime();
        }

        if(gamepad1.dpad_up){
            conf.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, -76.7, AngleUnit.DEGREES, 0));
        }

        conf.getDistance();
        if(conf.distance > 0 && gamepad1.left_bumper){
            conf.setFlywheelPower(conf.flywheelSpeed(conf.distance));
        }

        else if(gamepad1.right_bumper){ //Endgame Parking
            conf.odometryDrive(23,-48,90, 1);
            wantDrive = false;
        }

        else if(gamepad1.b){ //Far Shooting
            conf.setFlywheelPower(1500);//1680
            //conf.ledColors(1500);
            conf.odometryDrive(5.5,-22.5,35, 1); // 2.5,-2.2,22
            wantDrive = false;
        }

        else if(gamepad1.y){ //Opponent Shooting
            conf.launch_motor_1.setVelocity(1300);
            conf.launch_motor_2.setVelocity(1320);
            //conf.ledColors(1320);
            conf.odometryDrive(110,-51,85, 1);
            wantDrive = false;
        }

        else if(gamepad1.x){ //Middle shooting
            conf.setFlywheelPower(1200);
            //conf.ledColors(1200);
            conf.odometryDrive(66.5,-8.9,47, 1);
            wantDrive = false;
        }
        else if(gamepad1.a){ //Open gate
            conf.odometryDrive(49.3, 43.15, -120, conf.xMaxSpeed);
            wantDrive = false;
        }

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
        if(gamepad1.back && gamepad2.back && conf.flagTimer.seconds() > 115){
            conf.flag.setPosition(0.4);
        }


        LLResult llresult = conf.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            idgoal = fiducial.getFiducialId(); //The ID number of the fiducial
        }

        double tx = llresult.getTx();

        if(gamepad1.dpad_right){
            kP += 0.0001;
        }
        else if(gamepad1.dpad_left){
            kP -= 0.0001;
        }

        if(gamepad1.left_bumper){
            if(idgoal == 20){
                error = goalX - tx;

                if (Math.abs(error) < angleTolerance){
                    rotate = 0;
                }
                else{
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.5, 0.5);

                    Pose2D pose2D = conf.pinpoint.getPosition();
                    conf.xL = pose2D.getX(DistanceUnit.INCH);
                    conf.yL = pose2D.getY(DistanceUnit.INCH);
                    conf.hL = pose2D.getHeading(AngleUnit.DEGREES);

                    //x = 2 y = 16
                    //x = 1 y = -12
                    //x = 19, max 20
                    //
                    if(conf.yL <= 16 && conf.yL >= 12 && conf.xL <= 20 && conf.xL >= 0){
                        rotate = rotate - 14;
                    }

                    lastError = error;
                    lastTime = curTime;
                }
            }
            else{
                lastTime = getRuntime();
                lastError = 0;

            }
        }
        else{
            lastError = 0;
            lastTime = getRuntime();

        }

        if (wantDrive) {
            drive(forward, strafe, rotate);
        }

        if(idgoal == 20){
            if(gamepad1.left_bumper){
                telemetry.addLine("AutoAlign");
                telemetry.addData("ID",idgoal);
                telemetry.addData("P",kP);
                telemetry.addData("D",kD);
                conf.getDistance();
                telemetry.addData("Distance", conf.distance);
                telemetry.update();
            }
            else{
                telemetry.addLine("Manual");
                telemetry.update();
            }
        }
        else{
            telemetry.addData("ID Value", idgoal);
            telemetry.update();
        }
    }

    public void drive(double forward, double strafe, double rotate){
        double front_left_power = forward + strafe - rotate;
        double front_right_power = forward - strafe + rotate;
        double back_right_power = forward + strafe + rotate;
        double back_left_power = forward - strafe - rotate;

        conf.max_power = 1;
        conf.max_power = Math.max(conf.max_power, Math.abs(front_left_power));
        conf.max_power = Math.max(conf.max_power, Math.abs(front_right_power));
        conf.max_power = Math.max(conf.max_power, Math.abs(back_right_power));
        conf.max_power = Math.max(conf.max_power, Math.abs(back_left_power));

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
}