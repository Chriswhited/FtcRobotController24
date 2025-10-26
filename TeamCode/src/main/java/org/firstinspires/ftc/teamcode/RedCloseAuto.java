package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "RedCloseAuto", group = "Robot")
public class RedCloseAuto extends LinearOpMode {
    DcMotor back_left_drive;
    DcMotor front_left_drive;
    DcMotor back_right_drive;
    DcMotor front_right_drive;
    DcMotor launch_motor_1;
    DcMotor intake_motor;
    // ColorSensor color1;
    Servo franklin_flipper_right;
    Servo franklin_flipper_left;
    GoBildaPinpointDriver pinpoint;
    IMU imu;
    float hsvValues[] = {0F, 0F, 0F};
    Pose2D pos;
    double xProp = 0.04;
    double xInt = 0.0;
    double xDer = 0.0;
    double yProp = 0.04;
    double yInt = 0.0;
    double yDer = 0.0;
    double hProp = 0.03;
    double xMaxSpeed = 0.8;
    double yMaxSpeed = 0.8;
    double hMaxSpeed = 0.7;
    @Override
    public void runOpMode(){

        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        launch_motor_1 = hardwareMap.get(DcMotor.class, "launch_motor_1");
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        //color1 = hardwareMap.get(ColorSensor.class, "color1");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        franklin_flipper_left = hardwareMap.get(Servo.class, "franklin_flipper_left");
        franklin_flipper_right = hardwareMap.get(Servo.class, "franklin_flipper_right");

        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        launch_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        launch_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);

        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbdirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbdirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        while (opModeInInit()) {
            telemetry.addLine("Push your robot around to see it track");
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        waitForStart();

        /*
        odometryDrive(20,0,0);
        odometryDrive(20,-20,0);
        odometryDrive(0,-20,0);
        odometryDrive(0,0,0);
        odometryDrive(20,20,0);
        odometryDrive(20,20,90);
        odometryDrive(0,20,0);
         */
        //flywheel on

        launch_motor_1.setPower(1);
        odometryDrive(-44,-1,0, xMaxSpeed);

        sleep(4000); //spinup flywheel

        launch_motor_1.setPower(0.4);
        sleep(1000);
        franklin_flipper_right.setPosition(.11);
        sleep(1500);
        franklin_flipper_left.setPosition(1);
        sleep(500);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);
        sleep(250);
        franklin_flipper_right.setPosition(.11);
        sleep(1000);
        franklin_flipper_left.setPosition(1);
        sleep(500);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);
        sleep(750);
        //odometryDrive(6,-1,-22, xMaxSpeed);
        //odometryDrive(6,1,-22, xMaxSpeed);
        franklin_flipper_left.setPosition(1);
        franklin_flipper_right.setPosition(.11);
        sleep(500);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);

        odometryDrive(-43,-22,140, xMaxSpeed);
        intake_motor.setPower(1);
        odometryDrive(-24,-33,140, 0.5);
        intake_motor.setPower(0);
        odometryDrive(-44,-1,0, xMaxSpeed);

        franklin_flipper_right.setPosition(.11);
        sleep(1500);
        franklin_flipper_left.setPosition(1);
        sleep(500);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);
        sleep(250);
        franklin_flipper_right.setPosition(.11);
        sleep(1000);
        franklin_flipper_left.setPosition(1);
        sleep(500);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);
        sleep(750);
        //odometryDrive(6,-1,-22, xMaxSpeed);
        //odometryDrive(6,1,-22, xMaxSpeed);
        franklin_flipper_left.setPosition(1);
        franklin_flipper_right.setPosition(.11);
        sleep(500);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);

        odometryDrive(-57,-37,140, xMaxSpeed);
        sleep(200);//intake on
        odometryDrive(-34,-58,140, xMaxSpeed);
        sleep(200);//intake off
        odometryDrive(-44,-1,0, xMaxSpeed);
        sleep(200);//shoot
        odometryDrive(-51,-23,0, xMaxSpeed);
        //flywheel off


    }
    public void configurePinpoint(){
        pinpoint.setOffsets(0.945, 6.5, DistanceUnit.INCH); //Set robot offset
        //pinpoint.setOffsets(-6.5, -0.945, DistanceUnit.INCH); //Set robot offset
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); //Sets type of pod
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, //Set direction for pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    void odometryDrive(double targetX, double targetY, double targetH, double speed){
        double integralSumX = 0;
        double lastErrorX = 0;
        double integralSumY = 0;
        double lastErrorY = 0;
        xMaxSpeed = speed;
        yMaxSpeed = speed;
        ElapsedTime timer = new ElapsedTime();

        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        //pos = myPosition();
        double xError = targetX - pose2D.getX(DistanceUnit.INCH);
        double yError = targetY - pose2D.getY(DistanceUnit.INCH);
        double hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

        while(opModeIsActive() && Math.abs(xError) > 1.5 || Math.abs(yError) > 1.5 || Math.abs(hError) > 4){

            pinpoint.update();
            pose2D = pinpoint.getPosition();
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

            double x = Range.clip((xProp * xError) + (xInt * integralSumX) + (xDer * derivativeX),-xMaxSpeed,xMaxSpeed);
            double y = Range.clip((yProp * yError) + (yInt * integralSumY) + (yDer * derivativeY),-yMaxSpeed,yMaxSpeed);
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
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        double radian = pose2D.getHeading(AngleUnit.DEGREES) * Math.PI / 180;

        //Account for robot rotation
        double x_rotated = x * Math.cos(-radian) - y * Math.sin(-radian);
        double y_rotated = x * Math.sin(-radian) + y * Math.cos(-radian);

        double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(h), 1);

        double leftFrontPower = (x_rotated - y_rotated + h) / denominator;
        double leftBackPower = (x_rotated + y_rotated + h) / denominator;
        double rightFrontPower = (x_rotated + y_rotated - h) / denominator;
        double rightBackPower = (x_rotated - y_rotated - h) / denominator;


        // Send powers to the wheels.
        front_left_drive.setPower(leftFrontPower);
        front_right_drive.setPower(rightFrontPower);
        back_left_drive.setPower(leftBackPower);
        back_right_drive.setPower(rightBackPower);
        sleep(10);
    }
}