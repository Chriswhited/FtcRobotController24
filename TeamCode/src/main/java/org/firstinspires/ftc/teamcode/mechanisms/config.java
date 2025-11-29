package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class config {
    public DcMotor back_left_drive;
    public DcMotor front_left_drive;
    public  DcMotor back_right_drive;
    public DcMotor front_right_drive;
    public DcMotor launch_motor_1;
    public DcMotor intake_motor;
    // private ColorSensor color1;
    public Servo franklin_flipper_right;
    public Servo franklin_flipper_left;
    public GoBildaPinpointDriver pinpoint;
    public ColorSensor colorRight;
    public ColorSensor colorLeft;
    Pose2D pos;
    double xProp = 0.04;
    double xInt = 0.0;
    double xDer = 0.0;
    double yProp = 0.04;
    double yInt = 0.0;
    double yDer = 0.0;
    double hProp = 0.03;
    public double xMaxSpeed = 0.8;
    double yMaxSpeed = 0.8;
    double hMaxSpeed = 0.7;
    double xError = 0;
    double yError = 0;
    double hError = 0;
    double integralSumX = 0;
    double lastErrorX = 0;
    double integralSumY = 0;
    double lastErrorY = 0;
    double derivativeY = 0;
    double derivativeX = 0;
    double tag = 0;
    public boolean intake_var;
    public boolean intake_var2;
    public double max_power = 1.0;
    public float hsvValuesLeft[] = {0F,0F,0F};
    public float hsvValuesRight[] = {0F,0F,0F};

    boolean PIDreset = false;
    ElapsedTime sleeptime = new ElapsedTime();
    ElapsedTime PIDtimer = new ElapsedTime();
    public ElapsedTime intake_timer = new ElapsedTime();


    public void init(HardwareMap hwMap)  {
        back_left_drive = hwMap.get(DcMotor.class, "back_left_drive");
        front_left_drive = hwMap.get(DcMotor.class, "front_left_drive");
        back_right_drive = hwMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hwMap.get(DcMotor.class, "front_right_drive");
        launch_motor_1 = hwMap.get(DcMotor.class, "launch_motor_1");
        intake_motor = hwMap.get(DcMotor.class, "intake_motor");
        colorRight = hwMap.get(ColorSensor.class, "colorRight");
        colorLeft = hwMap.get(ColorSensor.class, "colorLeft");
        pinpoint = hwMap.get(GoBildaPinpointDriver .class, "pinpoint");
        franklin_flipper_right = hwMap.get(Servo .class, "franklin_flipper_right");
        franklin_flipper_left = hwMap.get(Servo.class, "franklin_flipper_left");

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

    }
    public void moveRobot(double x, double y, double h){
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
    public void launch(){
        franklin_flipper_right.setPosition(.11); //shoot cycle 1
        sleep(200);
        franklin_flipper_right.setPosition(.44);
        sleep(550);
        franklin_flipper_left.setPosition(1);
        sleep(200);
        franklin_flipper_left.setPosition(.64);
        sleep(500);
        franklin_flipper_right.setPosition(.11);
        franklin_flipper_left.setPosition(1);
        sleep(300);
        franklin_flipper_left.setPosition(.64);
        franklin_flipper_right.setPosition(.44);
    }
    public void configurePinpoint(){
        pinpoint.setOffsets(0.945, 6.5, DistanceUnit.INCH); //Set robot offset
        //pinpoint.setOffsets(-6.5, -0.945, DistanceUnit.INCH); //Set robot offset
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); //Sets type of pod
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, //Set direction for pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void odometryDrive(double targetX, double targetY, double targetH, double speed){

        if(!PIDreset){
            PIDreset = true;
            integralSumX = 0;
            lastErrorX = 0;
            integralSumY = 0;
            lastErrorY = 0;
            xMaxSpeed = speed;
            yMaxSpeed = speed;

            //ElapsedTime PIDtimer = new ElapsedTime();

            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            xError = targetX - pose2D.getX(DistanceUnit.INCH);
            yError = targetY - pose2D.getY(DistanceUnit.INCH);
            hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);
        }

        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        xError = targetX - pose2D.getX(DistanceUnit.INCH);
        yError = targetY - pose2D.getY(DistanceUnit.INCH);
        hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

        derivativeX = (xError - lastErrorX) / PIDtimer.seconds();
        integralSumX = integralSumX + (xError  * PIDtimer.seconds());
        derivativeY = (yError - lastErrorY) / PIDtimer.seconds();
        integralSumY = integralSumY + (yError  * PIDtimer.seconds());

        double x = Range.clip((xProp * xError) + (xInt * integralSumX) + (xDer * derivativeX),-xMaxSpeed,xMaxSpeed);
        double y = Range.clip((yProp * yError) + (yInt * integralSumY) + (yDer * derivativeY),-yMaxSpeed,yMaxSpeed);
        double h = Range.clip(hError * hProp, -hMaxSpeed, hMaxSpeed);


        moveRobot(x, y, h);

        lastErrorX = xError;
        lastErrorY = yError;
        PIDtimer.reset();

        if(Math.abs(xError) < .25 && Math.abs(yError) < .25 && Math.abs(hError) < .5) {
            PIDreset = false;
            moveRobot(0, 0, 0);
        }
    }
    public void sleep(double time){
        sleeptime.reset();
        while(sleeptime.milliseconds() <= time){

        }
    }
    public void AutoOdometryDrive(double targetX, double targetY, double targetH, double speed){
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

        while(Math.abs(xError) > 1.5 || Math.abs(yError) > 1.5 || Math.abs(hError) > 4){

            pinpoint.update();
            pose2D = pinpoint.getPosition();
            xError = targetX - pose2D.getX(DistanceUnit.INCH);
            yError = targetY - pose2D.getY(DistanceUnit.INCH);
            hError = targetH - pose2D.getHeading(AngleUnit.DEGREES);

            double derivativeX = (xError - lastErrorX) / timer.seconds();
            integralSumX = integralSumX + (xError  * timer.seconds());
            double derivativeY = (yError - lastErrorY) / timer.seconds();
            integralSumY = integralSumY + (yError  * timer.seconds());

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
    public void ColorLaunch(double tag){
        if (tag == 21) {
            franklin_flipper_right.setPosition(.11); //shoots green
            sleep(200);
            franklin_flipper_right.setPosition(.44);
            sleep(550);
            franklin_flipper_left.setPosition(1); //shoots purple
            sleep(200);
            franklin_flipper_left.setPosition(.64);
            sleep(700);
            franklin_flipper_left.setPosition(1); //shoots purple
            sleep(200);
            franklin_flipper_left.setPosition(.64);
            sleep(300);
            if(hsvValuesRight[0] > 145 || hsvValuesLeft[0] > 145){
                franklin_flipper_left.setPosition(1);
                franklin_flipper_right.setPosition(.11);
                sleep(200);
                franklin_flipper_left.setPosition(.64);
                franklin_flipper_right.setPosition(.44);
            }
        }
        else if (tag == 22){
            franklin_flipper_left.setPosition(1); //shoots purple
            sleep(200);
            franklin_flipper_left.setPosition(.64);
            sleep(550);
            franklin_flipper_right.setPosition(.11); //shoots green
            sleep(200);
            franklin_flipper_right.setPosition(.44);
            sleep(550);
            franklin_flipper_left.setPosition(1); //shoots purple
            sleep(200);
            franklin_flipper_left.setPosition(.64);
            sleep(300);
            if(hsvValuesRight[0] > 145 || hsvValuesLeft[0] > 145){
                franklin_flipper_left.setPosition(1);
                franklin_flipper_right.setPosition(.11);
                sleep(200);
                franklin_flipper_left.setPosition(.64);
                franklin_flipper_right.setPosition(.44);
            }
        }
        else{
            franklin_flipper_left.setPosition(1); //shoots purple
            sleep(200);
            franklin_flipper_left.setPosition(.64);
            sleep(700);
            franklin_flipper_left.setPosition(1); //shoots purple
            sleep(200);
            franklin_flipper_left.setPosition(.64);
            sleep(550);
            franklin_flipper_right.setPosition(.11); //shoots green
            sleep(200);
            franklin_flipper_right.setPosition(.44);
            sleep(300);
            if (hsvValuesRight[0] > 145 || hsvValuesLeft[0] > 145) {
                franklin_flipper_left.setPosition(1);
                franklin_flipper_right.setPosition(.11);
                sleep(200);
                franklin_flipper_left.setPosition(.64);
                franklin_flipper_right.setPosition(.44);
            }
        }

    }

}

