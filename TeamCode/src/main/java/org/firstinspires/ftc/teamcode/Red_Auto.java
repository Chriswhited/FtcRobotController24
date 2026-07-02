package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Red_Auto", group = "Robot")
public class Red_Auto extends OpMode {
    public double fieldColor = 1; //If Blue set to -1, if red set to 1
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor springMotor;
    DcMotor intakeMotor;
    DcMotorEx transferMotor;
    CRServo sweeper;
    Servo launchAngle;
    Servo transferServo;
    RevColorSensorV3 colorLauncher;
    RevColorSensorV3 colorRight;
    RevColorSensorV3 colorLeft;
    IMU imu;
    SparkFunOTOS opticalSensor;
    int attempts = 0;
    int rotations = 0;
    ElapsedTime sleeptime = new ElapsedTime();
    public double xProp = 0.04; //0.04
    public double xInt = 0; //0.0
    public double xDer = 0; //0.0
    public double yProp = 0.04; //0.04
    public double yInt = 0; //0.0
    public double yDer = 0; //0.0
    public double hProp = 0.03;
    public double hDer = 0.006;
    public double derivativeH = 0;
    public double hMaxSpeed = 0.7;
    public double xMaxSpeed = 1;
    public double yMaxSpeed = 1;
    public double CurrentX = 0;
    public double CurrentY = 0;
    public double CurrentH = 0;


    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drive_motor_3");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drive_motor_4");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drive_motor_1");
        backRightDrive = hardwareMap.get(DcMotor.class, "drive_motor_2");
        springMotor = hardwareMap.get(DcMotor.class, "motor_5");
        opticalSensor = hardwareMap.get(SparkFunOTOS.class, "optical_sensor");
        intakeMotor = hardwareMap.get(DcMotor.class,"motor_6");
        transferMotor = hardwareMap.get(DcMotorEx.class,"motor_7");
        colorLauncher = hardwareMap.get(RevColorSensorV3.class,"color_launcher");
        colorRight = hardwareMap.get(RevColorSensorV3.class, "intake_sensor_right");
        colorLeft = hardwareMap.get(RevColorSensorV3.class, "intake_sensor_left");
        sweeper = hardwareMap.get(CRServo.class,"sweeper");
        launchAngle = hardwareMap.get(Servo.class,"launch_servo");
        transferServo = hardwareMap.get(Servo.class,"transfer_servo");

        // We set the left motors in reverse which is needed for mecanum drive
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        springMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to run using encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        configureOptical(); // Configure Optical Odometry Sensor

        springMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        springMotor.setTargetPosition(0);
        springMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        springMotor.setPower(1);
    }
    public void init_loop() {
        //Set flywheel position
        transferServo.setPosition(.56);
        launchAngle.setPosition(.65);

        // Reset the tracking if the user requests it
        if (gamepad1.y) {
            opticalSensor.resetTracking();
        }

        // Re-calibrate the IMU if the user requests it
        if (gamepad1.x) {
            opticalSensor.calibrateImu();
        }

        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
        telemetry.addLine();

        // Log the position to the telemetry
        GetCurrentPosition();
        telemetry.addData("X coordinate", CurrentX);
        telemetry.addData("Y coordinate", CurrentY);
        telemetry.addData("Heading angle", CurrentH);

        // Update the telemetry on the driver station
        telemetry.update();

    }


    @Override
    public void start(){

        intakeMotor.setPower(1);
        transferMotor.setVelocity(2400);
        //sweeper.setPower(-1);

        //move to launch location
        AutoOdometryDrive(79,18,45,.6);

        while (transferMotor.getVelocity()<2360){

        }
        //Launch artifacts
        Launch();
        Verify();
        Launch();
        Verify();
        Launch3();

        //Move to first spike mark
        AutoOdometryDrive(77,33,90,.6);
        //Intake first 2 artifacts
        AutoOdometryDrive(77,44,90,.2);

        //Transfer 1st artifact to launcher
        transferServo.setPosition(.4);
        sleep(250);
        transferServo.setPosition(.56);
        sleep(750);
        Verify();
        //Intake third artifact
        AutoOdometryDrive(77,52,90,.2);

        //Move to Launch position
        AutoOdometryDrive(79,18,45,.6);
        sweeper.setPower(0);
        //Launch artifacts
        Launch();
        Verify();
        Launch();
        Verify();
        Launch3();

        AutoOdometryDrive(55,33,90,.6);
        //Intake first 2 artifacts
        AutoOdometryDrive(55,44,90,.2);

        //Transfer 1st artifact to launcher
        transferServo.setPosition(.4);
        sleep(250);
        transferServo.setPosition(.56);
        sleep(750);
        Verify();
        //Intake third artifact
        AutoOdometryDrive(55,52,90,.2);

        //Move to Launch position
        AutoOdometryDrive(79,18,45,.6);
        sweeper.setPower(0);
        //Launch artifacts
        Launch();
        Verify();
        Launch();
        Verify();
        Launch3();

        AutoOdometryDrive(67,29.5,45,.6);
    }


    public void loop(){

    }

    public void AutoOdometryDrive(double targetX, double targetY, double targetH, double speed){
        double integralSumX = 0;
        double lastErrorX = 0;
        double integralSumY = 0;
        double lastErrorY = 0;
        xMaxSpeed = speed;
        yMaxSpeed = speed;
        ElapsedTime timer = new ElapsedTime();

        //Get initial error
        GetCurrentPosition();
        double xError = (targetX) - CurrentX;
        double yError = (targetY * fieldColor) - CurrentY;
        double hError = (targetH * fieldColor) - CurrentH;

        while(Math.abs(xError) > 1 || Math.abs(yError) > 1 || Math.abs(hError) > 1){

            GetCurrentPosition();
            xError = (targetX) - CurrentX;
            yError = (targetY * fieldColor) - CurrentY;
            hError = (targetH * fieldColor) - CurrentH;

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

            telemetry.addData("X coordinate", CurrentX);
            telemetry.addData("Y coordinate", CurrentY * fieldColor);
            telemetry.addData("Heading angle", CurrentH * fieldColor);
            telemetry.update();

        }
        moveRobot(0, 0, 0);
    }

    //calculate and normalize wheel powers, then send powers to wheels
    void moveRobot(double x, double y, double h){
        //Convert to radians
        SparkFunOTOS.Pose2D pos = opticalSensor.getPosition();
        double radian = pos.h * Math.PI / 180;

        //Account for robot rotation
        double x_rotated = x * Math.cos(radian) - y * Math.sin(radian);
        double y_rotated = x * Math.sin(radian) + y * Math.cos(radian);

        double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(h), 1);

        double leftFrontPower = (x_rotated + y_rotated + h) / denominator;
        double leftBackPower = (x_rotated - y_rotated + h) / denominator;
        double rightFrontPower = (x_rotated - y_rotated - h) / denominator;
        double rightBackPower = (x_rotated + y_rotated - h) / denominator;


        // Send powers to the wheels.
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
        sleep(10);
    }
    public void sleep(double time){
        sleeptime.reset();
        while(sleeptime.milliseconds() <= time){
        }
    }

    public void GetCurrentPosition(){
        SparkFunOTOS.Pose2D pos = opticalSensor.getPosition();
        CurrentX = pos.y;
        CurrentY = pos.x;
        CurrentH = -pos.h;
    }

    public void Launch(){
        rotations = rotations + 1;
        springMotor.setTargetPosition(rotations * 2786);
        sleep(300);
        transferServo.setPosition(.4);
        sleep(250);
        transferServo.setPosition(.56);
        while(springMotor.isBusy()){

        }
    }
    public void Launch3(){
        rotations = rotations + 1;
        springMotor.setTargetPosition(rotations * 2786);
        while(springMotor.isBusy()){

        }
    }
    public void Verify(){
        if (colorLauncher.getDistance(DistanceUnit.INCH) < 2 || (colorLauncher.getDistance(DistanceUnit.INCH) > 2.3 && colorLeft.getDistance(DistanceUnit.INCH) > 2.75 && colorRight.getDistance(DistanceUnit.INCH) > 1.5)) {
            sweeper.setPower(-1);
        } else if (colorLeft.getDistance(DistanceUnit.INCH) < 2.3) {
            sweeper.setPower(0);
            while (colorLauncher.getDistance(DistanceUnit.INCH) > 2.3 && attempts < 3) {
                transferServo.setPosition(.4);
                sleep(250);
                transferServo.setPosition(.56);
                sleep(1000);
                attempts = attempts + 1;
            }
            attempts = 0;
            sweeper.setPower(-1);
        } else {
            sweeper.setPower(-1);
            sleep(750);
            while (colorLauncher.getDistance(DistanceUnit.INCH) < 2.3 && attempts < 3) {
                transferServo.setPosition(.4);
                sleep(250);
                transferServo.setPosition(.56);
                sleep(1000);
                attempts = attempts + 1;
            }
            attempts = 0;
        }

    }
    private void configureOptical() {
        telemetry.addLine("Configuring Optical Sensor...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        opticalSensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        opticalSensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(.35, -5.75, 0);
        opticalSensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        opticalSensor.setLinearScalar(1.0);
        opticalSensor.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        opticalSensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        opticalSensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D((fieldColor * 55.7), 111.4, (fieldColor * -51));

        //Zero Starting Position
        //SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);

        opticalSensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        opticalSensor.getVersionInfo(hwVersion, fwVersion);
    }
}