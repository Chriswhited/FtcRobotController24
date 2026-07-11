package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Red Teleop", group = "Robot")

public class Red_Teleop extends OpMode {
    public double fieldColor = 1; //If blue set to -1, if red set to 1
    public double offset = 0; //If blue set to 4, if red set to 0
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
    IMU imu;
    SparkFunOTOS opticalSensor;
    RevColorSensorV3 colorLauncher;
    RevColorSensorV3 colorRight;
    DistanceSensor leftDistance;
    //RevColorSensorV3 colorLeft;

    public float hsvValuesRight[] = {0F,0F,0F};
    public float hsvValuesLeft[] = {0F,0F,0F};
    public float hsvValuesLauncher[] = {0F,0F,0F};
    public static int rotations = 0;
    int counter = 0;
    ElapsedTime sleeptime = new ElapsedTime();
    ElapsedTime manualTransitionTime = new ElapsedTime();
    ElapsedTime autoTransitionTime = new ElapsedTime();
    ElapsedTime launchTransitionTime = new ElapsedTime();
    boolean autoTransitionVariable = false;
    boolean transferring = false;
    boolean start = true;
    double servoPosition = 1;
    int transferVelocity = 2100;
    double transferPosition = 0.42;

    //Odometry Dive Variables
    boolean PIDreset = false;
    ElapsedTime PIDtimer = new ElapsedTime();
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
    double integralSumX = 0;
    double lastErrorX = 0;
    double integralSumY = 0;
    double lastErrorY = 0;
    double xError = 0;
    double yError = 0;
    double hError = 0;
    double derivativeY = 0;
    double derivativeX = 0;
    public double CurrentX = 0;
    public double CurrentY = 0;
    public double CurrentH = 0;
    double maxSpeed = 1.0;  // make this slower to drive slower


    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drive_motor_3");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drive_motor_4");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drive_motor_1");
        backRightDrive = hardwareMap.get(DcMotor.class, "drive_motor_2");
        springMotor = hardwareMap.get(DcMotor.class, "motor_5");
        intakeMotor = hardwareMap.get(DcMotor.class,"motor_6");
        transferMotor = hardwareMap.get(DcMotorEx.class,"motor_7");
        opticalSensor = hardwareMap.get(SparkFunOTOS.class, "optical_sensor");
        colorLauncher = hardwareMap.get(RevColorSensorV3.class,"color_launcher");
        colorRight = hardwareMap.get(RevColorSensorV3.class, "intake_sensor_right");
        leftDistance = hardwareMap.get(DistanceSensor.class,"left_distance");
        //colorLeft = hardwareMap.get(RevColorSensorV3.class, "intake_sensor_left");
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

        imu = hardwareMap.get(IMU.class, "imu");
        // Set the orientation of the control hub for IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //configureOptical(); // Configure Optical Odometry Sensor

        //Set up launch paddle motor
        //springMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// Only use when not using an Auto before
        //springMotor.setTargetPosition(0); // Only use when not using an Auto before
        //rotations = 0; // Only use when not using an Auto before
        springMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void init_loop() {
        // Get sensor data
        GetCurrentPosition();

        // Log the sensors to the telemetry
        telemetry.addData("X coordinate", CurrentX);
        telemetry.addData("Y coordinate", CurrentY * fieldColor);
        telemetry.addData("Heading angle", CurrentH * fieldColor);
        telemetry.addLine();
        telemetry.addData("Launch Distance", colorLauncher.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance", colorRight.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance:", String.format("%.01f in", leftDistance.getDistance(DistanceUnit.INCH)));
        telemetry.addLine();
        telemetry.addData("Rotations", rotations);


        telemetry.update();
    }


    @Override
    public void loop() {

        //Get Sensor Data
        GetCurrentPosition();

        //Telemetry
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addData("X coordinate", CurrentX);
        telemetry.addData("Y coordinate", CurrentY * fieldColor);
        telemetry.addData("Heading angle", CurrentH * fieldColor);
        //telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("Launch Distance", colorLauncher.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", leftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Auto Transfer Veriable", autoTransitionVariable);
        telemetry.addData("Auto Transfer Timer", autoTransitionTime.milliseconds());
        //telemetry.addData("Velocity", transferMotor.getVelocity());
        telemetry.addData("Transferring", transferring);
        //telemetry.addData("Auto Timer", autoTransitionTime);
        //telemetry.addData("Manual", manualTransitionTime);
        telemetry.addData("Transfer Velocity", transferVelocity);
        telemetry.addData("Transfer Position", transferPosition);
        telemetry.addLine();
        telemetry.addData("Rotations", rotations);
        telemetry.update();


        //Set motor and servo powers/positions
        if(start){
            springMotor.setPower(1);
            transferServo.setPosition(.56);
            start = false;
        }
        springMotor.setTargetPosition(rotations * 2786);
        transferMotor.setVelocity(transferVelocity);


        //Set Intake motor
        if (leftDistance.getDistance(DistanceUnit.INCH) < 2.5 && colorRight.getDistance(DistanceUnit.INCH) < 1.5) {
            intakeMotor.setPower(0);
        } else {
            intakeMotor.setPower(.35);
        }


        //Determine if transfering
        if(colorLauncher.getDistance(DistanceUnit.INCH) < 2.0 || (autoTransitionTime.milliseconds() > 1000 && launchTransitionTime.milliseconds() > 1000 && manualTransitionTime.milliseconds() > 1000)){
            transferring = false;
        }


        //Set sweeper direction
        if (gamepad1.back) {
            transferMotor.setVelocity(-2000);
            sweeper.setPower(1);
            intakeMotor.setPower(-.35);
            transferring = false;
        } else if (leftDistance.getDistance(DistanceUnit.INCH) < 2.5 && colorRight.getDistance(DistanceUnit.INCH) < 1.5) {
            sweeper.setPower(0);
        } else if(transferring){
            sweeper.setPower(0);
        } else if (gamepad1.left_trigger > .5) {
            sweeper.setPower(1);
        } else {
            sweeper.setPower(-1);
        }


        //Set Driving conditions and drive
        // If you press the left bumper, you get a drive from the point of view of the robot
        if(gamepad1.y){ // Auto Center Field Position Button
            launchAngle.setPosition(.48);
            AutoOdometryDrive(73.5-offset,7,46+offset,1);

        } else if (gamepad1.x) { // Auto inside large triangle Position Button
            launchAngle.setPosition(.48);
            AutoOdometryDrive(102.5-offset,4,65+offset,1);

        } else if (gamepad1.b) { // Auto far outside large triangle Position Button
            launchAngle.setPosition(.48);
            AutoOdometryDrive(94.5-offset,-27,67+offset,1);

        } else if (gamepad1.a) { //Auto Human Player location
            AutoOdometryDrive(14-offset,-49,46+offset,1);

        } else if (gamepad1.right_trigger > .5) { // Auto Gate Open Position
            AutoOdometryDrive(64-offset,58.5,-90,1);

        }else if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


        //Launch Controls
        if(gamepad1.rightBumperWasPressed()){
            rotations = rotations + 1;
            launchTransitionTime.reset();
        }
        if (gamepad1.rightBumperWasReleased()){
            gamepad1.reset();
        }

        //Transition artifacts to the launcher
        //Note that auto load after launch is initated in the launch controls section
        /*
        //Transition manually
        if(gamepad1.x){
            manualTransitionTime.reset();
            transferServo.setPosition(.42);
            transferring = true;
        }
         */

        //Auto transition time
        if(leftDistance.getDistance(DistanceUnit.INCH) < 2 && colorLauncher.getDistance(DistanceUnit.INCH) > 2.3 ){
            counter = counter + 1;
            if(counter > 8){
                autoTransitionTime.reset();
                autoTransitionVariable = true;
                transferring = true;
            }
        }

        //Actual Transition Actions
        //Set transfer motor position for flush
        if (gamepad1.back) {
            transferServo.setPosition(.42);

            //Auto transfer after a launch
        } else if (launchTransitionTime.milliseconds() > 300 && launchTransitionTime.milliseconds() < 550){
            transferServo.setPosition(transferPosition);
            transferring = true;

            //Manual transfer
        /*} else if (manualTransitionTime.milliseconds() < 550) {
            transferServo.setPosition(transferPosition);
            transferring = true;
         */

            //Auto load if there is no artifact in launcher
        } else if (autoTransitionTime.milliseconds() < 250) {
            transferServo.setPosition(transferPosition);
            transferring = true;

            //Raise transfer motor
        } else {
            transferServo.setPosition(.56);
        }


        /*
        //Set Launch Angle
        if(gamepad1.dpad_up){
            transferVelocity = transferVelocity + 20;
        }
        if(gamepad1.dpad_down){
            transferVelocity = transferVelocity - 20;
        }
        if(gamepad1.dpad_right){
            transferPosition = transferPosition + .01;
        }
        if(gamepad1.dpad_left){
            transferPosition =transferPosition -.01;
        }
        if(gamepad1.b){
            transferMotor.setVelocity(transferVelocity);
            transferServo.setPosition(transferPosition);
        }

         */


    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        if (gamepad1.right_trigger > 0.5){
            maxSpeed = 0.3;
        } else {
            maxSpeed = 1.0;
        }

        double maxPower = 1.0;

        // Prevent motor clipped and keep proportional
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void AutoOdometryDrive(double targetX, double targetY, double targetH, double speed){
        if (!PIDreset){
            PIDreset = true;
            integralSumX = 0;
            lastErrorX = 0;
            integralSumY = 0;
            lastErrorY = 0;
            xMaxSpeed = speed;
            yMaxSpeed = speed;
            PIDtimer.reset();

            //Get initial error
            GetCurrentPosition();
            double xError = (targetX) - CurrentX;
            double yError = (targetY * fieldColor) - CurrentY;
            double hError = (targetH * fieldColor) - CurrentH;
        }

        GetCurrentPosition();
        xError = (targetX) - CurrentX;
        yError = (targetY * fieldColor) - CurrentY;
        hError = (targetH * fieldColor) - CurrentH;

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

        telemetry.addData("X coordinate", CurrentX);
        telemetry.addData("Y coordinate", CurrentY * fieldColor);
        telemetry.addData("Heading angle", CurrentH * fieldColor);
        telemetry.update();

        if (Math.abs(xError) < .25 && Math.abs(yError) < .25 && Math.abs(hError) < .5) {
            PIDreset = false;
            moveRobot(0, 0, 0);
        }
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
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
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
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        opticalSensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        opticalSensor.getVersionInfo(hwVersion, fwVersion);

    }
}
