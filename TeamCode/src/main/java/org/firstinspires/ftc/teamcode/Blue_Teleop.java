/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Blue Teleop", group = "Robot")

public class Blue_Teleop extends OpMode {
    public double fieldColor = -1; //If Blue set to -1, if red set to 1
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor springMotor;
    DcMotor intakeMotor;
    DcMotor transferMotor;
    CRServo sweeper;
    Servo launchAngle;
    Servo transferServo;
    IMU imu;
    SparkFunOTOS opticalSensor;
    RevColorSensorV3 colorLauncher;
    RevColorSensorV3 colorRight;
    RevColorSensorV3 colorLeft;
    //ColorSensor colorRight;
    //ColorSensor colorLeft;
    public float hsvValuesRight[] = {0F,0F,0F};
    public float hsvValuesLeft[] = {0F,0F,0F};
    public float hsvValuesLauncher[] = {0F,0F,0F};
    int rotations = 0;
    ElapsedTime sleeptime = new ElapsedTime();
    ElapsedTime manualTransitionTime = new ElapsedTime();
    ElapsedTime autoTransitionTime = new ElapsedTime();

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
    double servoPosition = 1;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drive_motor_3");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drive_motor_4");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drive_motor_1");
        backRightDrive = hardwareMap.get(DcMotor.class, "drive_motor_2");
        springMotor = hardwareMap.get(DcMotor.class, "motor_5");
        intakeMotor = hardwareMap.get(DcMotor.class,"motor_6");
        transferMotor = hardwareMap.get(DcMotor.class,"motor_7");
        opticalSensor = hardwareMap.get(SparkFunOTOS.class, "optical_sensor");
        colorLauncher = hardwareMap.get(RevColorSensorV3.class,"color_launcher");
        //colorRight = hardwareMap.get(ColorSensor.class, "intake_sensor_right");
        //colorLeft = hardwareMap.get(ColorSensor.class, "intake_sensor_left");
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
        springMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        springMotor.setTargetPosition(0);
        springMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        springMotor.setPower(1);

    }

    @Override
    public void init_loop() {
        // Get sensor data
        GetCurrentPosition();
        Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
        Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
        Color.RGBToHSV(colorLauncher.red() * 8, colorLauncher.green() * 8, colorLauncher.blue() * 8, hsvValuesLauncher);


        // Reset the tracking if the user requests it
        if (gamepad1.y) {
            opticalSensor.resetTracking();
        }

        // Re-calibrate the IMU if the user requests it
        if (gamepad1.x) {
            opticalSensor.calibrateImu();
        }

        //Set flywheel position
        transferServo.setPosition(.3);

        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
        telemetry.addLine();

        // Log the sensors to the telemetry
        telemetry.addData("X coordinate", CurrentX);
        telemetry.addData("Y coordinate", CurrentY * fieldColor);
        telemetry.addData("Heading angle", CurrentH * fieldColor);
        telemetry.addLine();
        telemetry.addData("Launch Hue", hsvValuesLauncher[0]);
        telemetry.addData("Launch Distance", colorLauncher.getDistance(DistanceUnit.INCH));
        telemetry.addLine();
        telemetry.addData("Right Hue", hsvValuesRight[0]);
        telemetry.addData("Right Distance", colorRight.getDistance(DistanceUnit.INCH));
        telemetry.addLine();
        telemetry.addData("Left Hue", hsvValuesLeft[0]);
        telemetry.addData("Left Distance", colorLeft.getDistance(DistanceUnit.INCH));
        telemetry.addLine();
        telemetry.addData("Servo Position", servoPosition);

        telemetry.update();
    }

    @Override
    public void loop() {

        //Get Sensor Data
        GetCurrentPosition();
        Color.RGBToHSV(colorRight.red() * 8, colorRight.green() * 8, colorRight.blue() * 8, hsvValuesRight);
        //Color.RGBToHSV(colorLeft.red() * 8, colorLeft.green() * 8, colorLeft.blue() * 8, hsvValuesLeft);
        //Color.RGBToHSV(colorLauncher.red() * 8, colorLauncher.green() * 8, colorLauncher.blue() * 8, hsvValuesLauncher);

        //Telemetry
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addData("X coordinate", CurrentX);
        telemetry.addData("Y coordinate", CurrentY * fieldColor);
        telemetry.addData("Heading angle", CurrentH * fieldColor);
        telemetry.addData("Servo Position", servoPosition);
        //telemetry.addData("Launch Hue", hsvValuesLauncher[0]);
        telemetry.addData("Right Hue", hsvValuesRight[0]);
        //telemetry.addData("Left Hue", hsvValuesLeft[0]);
        telemetry.addData("Launch Distance", colorLauncher.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Right Distance", colorRight.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", colorLeft.getDistance(DistanceUnit.INCH));
        telemetry.update();

        //Set motor and survo powers/positions
        springMotor.setTargetPosition(rotations * 2786);
        transferMotor.setPower(1);
        intakeMotor.setPower(1);

        /*
        if(hsvValuesRight[0] < 35 && hsvValuesLeft[0] < 35){
        //if(colorRight.getDistance(DistanceUnit.INCH) < 35 && colorLeft.getDistance(DistanceUnit.INCH) < 35){
            intakeMotor.setPower(0);
        } else {
            intakeMotor.setPower(1);
        }

         */

        if(colorRight.getDistance(DistanceUnit.INCH) > 1.5 || colorLeft.getDistance(DistanceUnit.INCH) > 2.5){
        //if(colorRight.getDistance(DistanceUnit.INCH) > 40 || colorLeft.getDistance(DistanceUnit.INCH) > 40){
            sweeper.setPower(-1);
        } else {
            sweeper.setPower(0);
        }


        //Set Driving conditions and drive
        // If you press the left bumper, you get a drive from the point of view of the robot
        if(gamepad1.y){ // Auto Position Button
            AutoOdometryDrive(69.5,7,50,1);
        } else if (gamepad1.a) {
            AutoOdometryDrive(14,-53,50,1);
        } else if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


        //Launch Controls
        if(gamepad1.rightBumperWasPressed()){
            rotations = rotations + 1;
            autoTransitionTime.reset();

        }
        if (gamepad1.rightBumperWasReleased()){
            gamepad1.reset();
        }

        //Transition artifacts to the launcher 
        if(autoTransitionTime.milliseconds() > 300 && autoTransitionTime.milliseconds() < 550){
            transferServo.setPosition(.16);
        } else if (manualTransitionTime.milliseconds() > 300 && manualTransitionTime .milliseconds() < 550) {
            transferServo.setPosition(.16);
        } else {
            transferServo.setPosition(.3);
        }

        //Transition artifacts to the launcher manually
        if(gamepad1.x){
            manualTransitionTime.reset();
        }


        //Set Launch Angle
        if(gamepad1.dpad_up){
            servoPosition = servoPosition + .01;
        }
        if(gamepad1.dpad_down){
            servoPosition = servoPosition - .01;
        }
        if(gamepad1.x){
            launchAngle.setPosition(servoPosition);
        }

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
