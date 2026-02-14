package org.firstinspires.ftc.teamcode.mechanisms;


//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.List;


public class configwLimeLight {
    public DcMotor back_left_drive;
    public DcMotor front_left_drive;
    public DcMotor back_right_drive;
    public DcMotor front_right_drive;
    public DcMotorEx launch_motor_1;
    public DcMotorEx launch_motor_2;
    public DcMotor intake_motor;
    public Servo franklin_flipper_right;
    public Servo franklin_flipper_left;
    public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight;
    public ColorSensor colorRight;
    public ColorSensor colorLeft;
    public ColorSensor colorCenter;
    public ColorSensor colorIntake;
    PrismAnimations.Solid solid = new PrismAnimations.Solid(org.firstinspires.ftc.teamcode.Prism.Color.BLUE);

    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
    Pose2D pos;
    double xProp = 0.05; //0.04
    double xInt = 0; //0.0
    double xDer = 0.001; //0.0
    double yProp = 0.06; //0.04
    double yInt = 0; //0.0
    double yDer = 0.0015; //0.0
    double hProp = 0.03;
    double hDer = 0.006;
    public double xMaxSpeed = 1;
    double yMaxSpeed = 1;
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
    double derivativeH = 0;
    public boolean ColorReadVar = false;
    public double tag = 0;
    public double id = 0;
    public double idd = 0;
    public double idg = 0;
    public double updown = 200;
    public boolean intake_var;
    public boolean flywheelStart = false;
    public boolean intake_var2;
    public double max_power = 1.0;
    public float hsvValuesLeft[] = {0F,0F,0F};
    public float hsvValuesRight[] = {0F,0F,0F};
    public float hsvValuesCenter[] = {0F,0F,0F};
    public float hsvValuesIntake[] = {0F,0F,0F};
    boolean PIDreset = false;
    ElapsedTime sleeptime = new ElapsedTime();
    ElapsedTime PIDtimer = new ElapsedTime();
    public ElapsedTime intake_timer = new ElapsedTime();

    public ElapsedTime colorReadTimer = new ElapsedTime();

    public LED redLED;
    public LED greenLED;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public double status = 0;
    double currentX = 0;
    double currentY = 0;
    public GoBildaPrismDriver prism;
    public double velocity1 = 0;
    public double wantedVelocity;




    public void init(HardwareMap hwMap)  {
        back_left_drive = hwMap.get(DcMotor.class, "back_left_drive");
        front_left_drive = hwMap.get(DcMotor.class, "front_left_drive");
        back_right_drive = hwMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hwMap.get(DcMotor.class, "front_right_drive");
        launch_motor_1 = hwMap.get(DcMotorEx.class, "launch_motor_1");
        launch_motor_2 = hwMap.get(DcMotorEx.class, "launch_motor_2");
        intake_motor = hwMap.get(DcMotor.class, "intake_motor");
        colorRight = hwMap.get(ColorSensor.class, "colorRight");
        colorLeft = hwMap.get(ColorSensor.class, "colorLeft");
        colorCenter = hwMap.get(ColorSensor.class, "colorCenter");
        colorIntake = hwMap.get(ColorSensor.class, "colorIntake");
        pinpoint = hwMap.get(GoBildaPinpointDriver .class, "pinpoint");
        franklin_flipper_right = hwMap.get(Servo .class, "franklin_flipper_right");
        franklin_flipper_left = hwMap.get(Servo.class, "franklin_flipper_left");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        redLED = hwMap.get(LED.class, "redLED");
        greenLED = hwMap.get(LED.class, "greenLED");
        prism = hwMap.get(GoBildaPrismDriver.class,"prism");

        launch_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        launch_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);

        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launch_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launch_motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients PIDF = new PIDFCoefficients(500,0,0,0);
        launch_motor_1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        launch_motor_2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);

        solid.setBrightness(100);
        solid.setStartIndex(6);
        solid.setStopIndex(17);

        rainbowSnakes.setNumberOfSnakes(2);
        rainbowSnakes.setSnakeLength(3);
        rainbowSnakes.setSpacingBetween(6);
        rainbowSnakes.setSpeed(0.5f);

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
    public void setFlywheelPower(double velocity){
        launch_motor_1.setVelocity(velocity);
        launch_motor_2.setVelocity(velocity);
        velocity1 = velocity;
    }

    public void ledColors(double velocity1){
        if((velocity1-90) <= launch_motor_1.getVelocity() && (velocity1+30) > launch_motor_1.getVelocity()){
            Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
            Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorCenter.red() * 8, colorCenter.green() * 8, colorCenter.blue() * 8, hsvValuesCenter);

            if(hsvValuesRight[0] > 140 && hsvValuesLeft[0] > 140 && hsvValuesCenter[0] > 140){
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
            }
            else{
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
            }
        }
        else{
            Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
            Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorCenter.red() * 8, colorCenter.green() * 8, colorCenter.blue() * 8, hsvValuesCenter);

            if(hsvValuesRight[0] > 140 && hsvValuesLeft[0] > 140 && hsvValuesCenter[0] > 140){
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
            }
            else{
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
            }
        }
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

            PIDtimer.reset();

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
        double h = Range.clip((hError * hProp) + (hDer * derivativeH), -hMaxSpeed, hMaxSpeed);


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
            dashboardTelemetry.addData("Flywheel on", launch_motor_1.getVelocity());
            dashboardTelemetry.update();
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

        while(Math.abs(xError) > 1 || Math.abs(yError) > 1 || Math.abs(hError) > .5){

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
            double h = Range.clip((hError * hProp) + (hDer * derivativeH), -hMaxSpeed, hMaxSpeed);


            moveRobot(x, y, h);

            lastErrorX = xError;
            lastErrorY = yError;
            timer.reset();

            dashboardTelemetry.addData("X position", pose2D.getX(DistanceUnit.INCH));
            dashboardTelemetry.addData("Y position", pose2D.getY(DistanceUnit.INCH));
            dashboardTelemetry.addData("X Error", xError);
            dashboardTelemetry.addData("Y Error", yError);
            dashboardTelemetry.addData("H Error", hError);
            dashboardTelemetry.update();
        }
        moveRobot(0, 0, 0);
    }
    public void ReadTag(){
        LLResult llresult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
    }


    public void AutoAlign() {

        //Getting Current Pinpoint
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        if(status == 0) {
            currentX = pos.getX(DistanceUnit.INCH);
            currentY = pos.getY(DistanceUnit.INCH);
        }
        double currentH = pos.getHeading(AngleUnit.DEGREES);
        pinpoint.update();


        //Read tag
        LLResult llresult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            idg = fiducial.getFiducialId(); //The ID number of the fiducial
        }
        if (idg == 20 || idg == 24) {
            double tx = llresult.getTx(); //target x
            dashboardTelemetry.addData("Target X", tx);
            dashboardTelemetry.update();
            double alignerror = 1.5;   //How much error

            //Stop when we are centered on april tag
            if (Math.abs(tx) < alignerror) {
                odometryDrive(currentX, currentY, currentH, xMaxSpeed);
            }
            //Keep driving till we are in the error margin
            else{
                odometryDrive(currentX, currentY, (currentH - tx), xMaxSpeed);
            }
        }
    }

    public void ColorLaunch(double tag) {

        //GPP
        if (tag == 21) {
            //Reading in Hue Values
            Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
            Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorCenter.red() * 8, colorCenter.green() * 8, colorCenter.blue() * 8, hsvValuesCenter);

            //Checking if Green Artifact is on the right side
            if (hsvValuesRight[0] > 140 && hsvValuesRight[0] < 180) {
                franklin_flipper_right.setPosition(.11); //shoots green
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(550);
                franklin_flipper_left.setPosition(1); //shoots purple
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(550);
                franklin_flipper_right.setPosition(.11); //shoots purple
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(500);
                Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                while (hsvValuesRight[0] > 140 || hsvValuesLeft[0] > 140) {
                    franklin_flipper_left.setPosition(1);
                    franklin_flipper_right.setPosition(.11);
                    sleep(updown);
                    franklin_flipper_left.setPosition(.64);
                    franklin_flipper_right.setPosition(.44);
                    sleep(600);
                    Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                    Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                }
            }
            //If Green Artifact is not on the right side then it must be on left
            else {
                franklin_flipper_left.setPosition(1); //shoots green
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(550);
                franklin_flipper_right.setPosition(.11); //shoots purple
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(550);
                franklin_flipper_left.setPosition(1); //shoots purple
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(500);
                Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);

                //Checks if any artifacts were not launched
                while (hsvValuesRight[0] > 140 || hsvValuesLeft[0] > 140) {
                    franklin_flipper_left.setPosition(1);
                    franklin_flipper_right.setPosition(.11);
                    sleep(updown);
                    franklin_flipper_left.setPosition(.64);
                    franklin_flipper_right.setPosition(.44);
                    sleep(600);
                    Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                    Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                }
            }

        }
        //PGP
        else if (tag == 22) {
            Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
            Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);

            //Checking if Green Artifact is on the right side
            if (hsvValuesRight[0] > 140 && hsvValuesRight[0] < 180) {
                franklin_flipper_left.setPosition(1); //shoots purple
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(550);
                franklin_flipper_right.setPosition(.11); //shoots green
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(550);
                franklin_flipper_left.setPosition(1); //shoots purple
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(500);
                Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);

                //Checks if any artifacts were not launched
                while (hsvValuesRight[0] > 140 || hsvValuesLeft[0] > 140) {
                    franklin_flipper_left.setPosition(1);
                    franklin_flipper_right.setPosition(.11);
                    sleep(updown);
                    franklin_flipper_left.setPosition(.64);
                    franklin_flipper_right.setPosition(.44);
                    sleep(600);
                    Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                    Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                }
            }
            //If Green Artifact is not on the right side then it must be on left
            else {
                franklin_flipper_right.setPosition(.11); //shoots purple
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(550);
                franklin_flipper_left.setPosition(1); //shoots green
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(550);
                franklin_flipper_right.setPosition(.11); //shoots purple
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(500);
                Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                while (hsvValuesRight[0] > 140 || hsvValuesLeft[0] > 140) {
                    franklin_flipper_left.setPosition(1);
                    franklin_flipper_right.setPosition(.11);
                    sleep(updown);
                    franklin_flipper_left.setPosition(.64);
                    franklin_flipper_right.setPosition(.44);
                    sleep(600);
                    Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                    Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                }
            }

        }

        //PPG
        else {
            Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
            Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);

            //Checking if Green Artifact is on the right side
            if (hsvValuesRight[0] > 140 && hsvValuesRight[0] < 180) {
                franklin_flipper_left.setPosition(1); //shoots purple
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(1000);
                franklin_flipper_left.setPosition(1); //shoots purple
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(550);
                franklin_flipper_right.setPosition(.11); //shoots green
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(500);
                Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                while (hsvValuesRight[0] > 140 || hsvValuesLeft[0] > 140) {
                    franklin_flipper_left.setPosition(1);
                    franklin_flipper_right.setPosition(.11);
                    sleep(updown);
                    franklin_flipper_left.setPosition(.64);
                    franklin_flipper_right.setPosition(.44);
                    sleep(600);
                    Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                    Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                }
            }
            //If Green Artifact is not on the right side then it must be on left
            else {
                franklin_flipper_right.setPosition(.11); //shoots purple
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(1000);
                franklin_flipper_right.setPosition(.11); //shoots purple
                sleep(updown);
                franklin_flipper_right.setPosition(.44);
                sleep(550);
                franklin_flipper_left.setPosition(1); //shoots green
                sleep(updown);
                franklin_flipper_left.setPosition(.64);
                sleep(500);
                Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                while (hsvValuesRight[0] > 140 || hsvValuesLeft[0] > 140) {
                    franklin_flipper_left.setPosition(1);
                    franklin_flipper_right.setPosition(.11);
                    sleep(updown);
                    franklin_flipper_left.setPosition(.64);
                    franklin_flipper_right.setPosition(.44);
                    sleep(600);
                    Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
                    Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
                }
            }

        }
    }
}

