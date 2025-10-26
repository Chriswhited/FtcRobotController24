package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Worlds")
@Disabled

public class Worlds extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor left_front_drive;
    private DcMotor right_drive;
    private DcMotor right_front_drive;
    private DcMotor arm;
    private DcMotor other_arm;
    private Servo bumper_servo;
    private Servo claw;
    private Servo other_claw;
    SparkFunOTOS Odometer;
    SparkFunOTOS.Pose2D pos;

    //Setting Variables for PIDs

    /*
    double xProp = 0.06;
    double xInt = 0.0;
    double xDer = 0.001;
    double yProp = 0.12;
    double yInt = 0.0;
    double yDer = 0.001;
    double hProp = 0.03;

     */
    double xProp = 0.08;
    double xInt = 0.0;
    double xDer = 0.001;
    double yProp = 0.12;
    double yInt = 0.0;
    double yDer = 0.001;
    double hProp = 0.03;

    double xMaxSpeed = 1;
    double yMaxSpeed = 1;
    double hMaxSpeed = 0.6;

    double wallgrab = -500;
    double armhangheight = -2970;
    double armpullheight = -2460;
    double bumperUp = 0.03;
    double bumperDown = 0.36;
    double spClawOpen = 0.45;
    double spClawClosed = 0.1;

    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        left_front_drive = hardwareMap.get(DcMotor.class,"left_front_drive");
        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_front_drive = hardwareMap.get(DcMotor.class,"right_front_drive");
        Odometer = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        bumper_servo = hardwareMap.get(Servo.class,"bumper servo");
        other_claw = hardwareMap.get(Servo.class, "other claw servo");
        claw = hardwareMap.get(Servo.class, "claw servo");
        other_arm = hardwareMap.get(DcMotor.class, "sub arm motor");
        arm = hardwareMap.get(DcMotor.class, "arm motor");

        //Settings Wheel Directions
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        //Set motor behavior with no power
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset Encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        other_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Configuring Odometer
        Odometer.setLinearUnit(DistanceUnit.INCH);
        Odometer.setAngularUnit(AngleUnit.DEGREES);

        //SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.8125, -2.375, 0);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(2.375, -.8125, 0);
        Odometer.setOffset(offset);

        Odometer.setLinearScalar(1);
        Odometer.setAngularScalar(1);

        Odometer.calibrateImu();
        Odometer.resetTracking();

        //Set robot starting position
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(-53.5, 0, 0);
        Odometer.setPosition(currentPosition);

        while (opModeInInit()) {
            pos = myPosition();
            telemetry.addData("Status","Initialized");
            telemetry.addLine();
            telemetry.addData("X position", pos.x);
            telemetry.addLine();
            telemetry.addData("Y position", pos.y);
            telemetry.addLine();
            telemetry.addData("Current Heading", pos.h);
            telemetry.update();
            bumper_servo.setPosition(0.03);
            claw.setPosition(spClawClosed);
            other_claw.setPosition(0.4);
        }

        waitForStart();
        //Hang first specimen
        OTMArmDrive(armhangheight, 1);
        other_claw.setPosition(0.06);
        OTMOtherArmDrive(-375, 1);
        bumper_servo.setPosition(bumperDown);
        sleep(500);
        odometryDrive(-52.5, -10, 90);
        odometryDrive(-52.5,-22, 90);
        ArmDrive(armpullheight, 1);
        sleep(100);
        claw.setPosition(spClawOpen);

        // Pushing samples
        OTMArmDrive(wallgrab, 1);
        bumper_servo.setPosition(bumperUp);
        odometryDrive(-27.937,-21.9, 90);

        //1st
        //odometryDrive(-25.77,-46.6, 90);
        odometryDrive(-17.4,-54, 90);//-53
        odometryDrive(-17.5,-15, 90);
        //odometryDrive(-16.4,-46.6, 90);

        //2nd
        odometryDrive(-9.4,-54, 85); //-9.4
        odometryDrive(-5.8,-15, 90);

        //3rd
        odometryDrive(-4.8,-47, 90);
        //odometryDrive(-0.45,-50, 90);

        left_front_drive.setPower(-.65);
        right_front_drive.setPower(.65);
        left_drive.setPower(.4);
        right_drive.setPower(-.4);
        sleep(500);
        left_front_drive.setPower(0);
        right_front_drive.setPower(0);
        left_drive.setPower(0);
        right_drive.setPower(0);
        sleep(100);

        pos = myPosition();



        //telemetry.addData("xPostion", pos.x);
        //telemetry.addData("yPostion", pos.y);
        // pos = myPosition();
        //telemetry.addData("AfterXPostion", pos.x);
        //telemetry.addData("AfterYPostion", pos.y);
        //telemetry.update();
        //sleep(5000);

        //grab specimen
        odometryDrive((pos.x), -21, 85);
        odometryDrive(-15.1,-5.9, 90);

        //Hang second specimen
        odometryDrive(-15.1, -1.0, 90);
        claw.setPosition(spClawClosed);
        sleep(150);
        bumper_servo.setPosition(bumperDown);
        OTMArmDrive(armhangheight, 1);
        odometryDrive(-68.5, -15, 90);
        odometryDrive(-68.5, -23, 90);
        ArmDrive(armpullheight, 1);
        //sleep(200);
        claw.setPosition(spClawOpen);

        //Hang third specimen
        OTMArmDrive(wallgrab, 1);
        odometryDrive(-25.1, -10, 90);
        odometryDrive(-25.1, -1.0, 90);
        claw.setPosition(spClawClosed);
        sleep(150);
        OTMArmDrive(armhangheight, 1);
        odometryDrive(-65, -15, 90);
        odometryDrive(-65, -23, 90);
        ArmDrive(armpullheight, 1);
        //sleep(200);
        claw.setPosition(spClawOpen);

        //Hang fourth specimen
        OTMArmDrive(wallgrab, 1);
        odometryDrive(-25.1, -10, 90);
        odometryDrive(-25.1, -1.0, 90);
        claw.setPosition(spClawClosed);
        sleep(150);
        OTMArmDrive(armhangheight, 1);
        odometryDrive(-61.5, -15, 90);
        odometryDrive(-61.5, -23, 90);
        ArmDrive(armpullheight, 1);
        //sleep(200);
        claw.setPosition(spClawOpen);

        //Hang fifth specimen
        OTMArmDrive(wallgrab, 1);
        odometryDrive(-25.1, -10, 90);
        odometryDrive(-25.1, -1.0, 90);
        claw.setPosition(spClawClosed);
        sleep(150);
        OTMArmDrive(armhangheight, 1);
        odometryDrive(-58, -15, 90);
        odometryDrive(-58, -23, 90);
        ArmDrive(armpullheight, 1);
        //sleep(350);
        claw.setPosition(spClawOpen);
        OTMArmDrive(wallgrab, 1);
        OTMOtherArmDrive(0, 1);
        odometryDrive(-5,-5,90);


        /*
        bumper_servo.setPosition(0.03);
        claw.setPosition(0.0);
        other_claw.setPosition(0.0);
        odometryDrive(10, 10, 90);
        ArmDrive(1000, 1);
        OTMArmDrive(1000, 1);
        OTMOtherArmDrive(1000, 1);
         */


    }
    SparkFunOTOS.Pose2D myPosition() {
        pos = Odometer.getPosition();
        SparkFunOTOS.Pose2D CurrentPosition = new SparkFunOTOS.Pose2D(pos.x, -pos.y, -pos.h);
        return(CurrentPosition);
    }
    void odometryDrive(double targetX, double targetY, double targetH){
        double integralSumX = 0;
        double lastErrorX = 0;
        double integralSumY = 0;
        double lastErrorY = 0;
        ElapsedTime timer = new ElapsedTime();

        pos = myPosition();
        double xError = targetX - pos.x;
        double yError = targetY - pos.y;
        double hError = targetH - pos.h;

        while(opModeIsActive() && Math.abs(xError) > 1.5 || Math.abs(yError) > 1.5 || Math.abs(hError) > 4){

            pos = myPosition();
            xError = targetX - pos.x;
            yError = targetY - pos.y;
            hError = targetH - pos.h;

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

            double x = (xProp * xError) + (xInt * integralSumX) + (xDer * derivativeX);
            double y = (yProp * yError) + (yInt * integralSumY) + (yDer * derivativeY);
            double h = Range.clip(hError * hProp, -hMaxSpeed, hMaxSpeed);


            moveRobot(x, y, h);

            lastErrorX = xError;
            lastErrorY = yError;
            timer.reset();
        }
        moveRobot(0, 0, 0);
    }

    //calculate and normalize wheel powers, then send powers to wheels
    void moveRobot(double x, double y, double h){
        //Convert to radians
        pos = myPosition();
        double radian = pos.h * Math.PI / 180;

        //Account for robot rotation
        double x_rotated = x * Math.cos(-radian) - y * Math.sin(-radian);
        double y_rotated = x * Math.sin(-radian) + y * Math.cos(-radian);

        double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(h), 1);

        double leftFrontPower = (x_rotated + y_rotated + h) / denominator;
        double leftBackPower = (x_rotated - y_rotated + h) / denominator;
        double rightFrontPower = (x_rotated - y_rotated - h) / denominator;
        double rightBackPower = (x_rotated + y_rotated - h) / denominator;
/*
        telemetry.addData("LF: ", leftFrontPower);
        telemetry.addData("LB: ", leftBackPower);
        telemetry.addData("RF: ", rightFrontPower);
        telemetry.addData("RB: ", rightBackPower);
        telemetry.update();

 */

        // Send powers to the wheels.
        left_front_drive.setPower(leftFrontPower);
        right_front_drive.setPower(rightFrontPower);
        left_drive.setPower(leftBackPower);
        right_drive.setPower(rightBackPower);
        sleep(10);
    }
    public void ArmDrive(double position, double speed) {
        if (opModeIsActive()) {

            int Target = (int)(position);
            arm.setTargetPosition(Target);
            arm.setPower(speed);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Wait for motors to finish driving
            while (arm.isBusy()){
                telemetry.addData("Status", "Running");
                telemetry.addData("Position:", "%d ", arm.getCurrentPosition());
                telemetry.update();
            }

            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setPower(0);
        }
    }



    public void OTMArmDrive(double position, double speed) {
        if (opModeIsActive()) {

            int Target = (int)(position);
            arm.setTargetPosition(Target);
            arm.setPower(speed);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }

    public void OTMOtherArmDrive(double position, double speed) {
        if (opModeIsActive()) {

            int Target = (int)(position);
            other_arm.setTargetPosition(Target);
            other_arm.setPower(speed);
            other_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
}
