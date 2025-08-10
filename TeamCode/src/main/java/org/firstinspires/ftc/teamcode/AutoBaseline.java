package org.firstinspires.ftc.teamcode;

//Based on RobotAutoDriveByGyro_Linear sample program

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AutoBaseline")

public class AutoBaseline extends LinearOpMode {
    private IMU imu;
    private DcMotor left_drive;
    private DcMotor left_front_drive;
    private DcMotor right_drive;
    private DcMotor right_front_drive;
    private DcMotor arm;
    private DcMotor other_arm;
    private Servo claw;
    private Servo other_claw;
    private Servo bumper_servo;

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private double  frontSpeed     = 0;
    private double  backSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    private int     frontLeftTarget    = 0;
    private int     backLeftTarget    = 0;
    private int     frontRightTarget   = 0;
    private int     backRightTarget   = 0;
    double wallgrab = -500;
    double armhangheight = -2970;
    double armpullheight = -2489;

    //Set variables
    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.



    @Override
    public void runOpMode() {
        //Initialize Variables
        imu = hardwareMap.get(IMU.class, "imu");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        left_front_drive = hardwareMap.get(DcMotor.class,"left_front_drive");
        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_front_drive = hardwareMap.get(DcMotor.class,"right_front_drive");
        arm = hardwareMap.get(DcMotor.class, "arm motor");
        claw = hardwareMap.get(Servo.class, "claw servo");
        other_arm = hardwareMap.get(DcMotor.class, "sub arm motor");
        other_claw = hardwareMap.get(Servo.class, "other claw servo");
        bumper_servo = hardwareMap.get(Servo.class,"bumper servo");

        //Set wheel direction
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        //Define hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        //Initialize IMU
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //Reset the encoders and set the motors to BRAKE mode
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        other_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        other_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set telemetry display before match starts
        while (opModeInInit()) {
            telemetry.addData("Status","Scrimmage");
            telemetry.addData("Current Heading","%.2f Deg.",getHeading());
            telemetry.update();
            claw.setPosition(0.0);
            other_claw.setPosition(0.3);
            bumper_servo.setPosition(0.03);
        }

        //Set motors to use encoder and reset IMU
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        waitForStart();


        //hang specimen
        //sleep(4000);
        //OTMOtherArmDrive(-350, 0.1);
        //claw.setPosition(0.05);
        //other_claw.setPosition(0.0);
        //OTMArmDrive(-2935, 1);
        Strafe(DRIVE_SPEED, 3.0, 0.0, -1);
        driveStraight(DRIVE_SPEED, -0.1, 0.0);
        turnToHeading(0.5, -90);
        //Strafe(DRIVE_SPEED, 3.0, -90.0, -1);
        driveStraight(DRIVE_SPEED, -19.0, -90.0);
        //ArmDrive(-2500, 1);
        //sleep(250);
        //claw.setPosition(0.4);

        //Push 1st sample
        Strafe(DRIVE_SPEED, 11.0, -90.0, 1);
        driveStraight(DRIVE_SPEED, -27.0, -90.0);
        Strafe(DRIVE_SPEED, 4.0, -90.0, 1);
        driveStraight(DRIVE_SPEED, -0.1, -90.0);
        turnToHeading(0.5, -100);
        driveStraight(0.6, 46.0, -100.0);

        //Push 2nd sample
        driveStraight(0.6, -49.0, -100.0);
        turnToHeading(0.5, -90.0);
        driveStraight(DRIVE_SPEED, -0.1, -90.0);
        Strafe(DRIVE_SPEED, 7.0, -90.0, 1);
        driveStraight(0.6, 46.0, -90.0);

        //Push 3rd sample
        driveStraight(0.6, -46.0, -90.0);
        Strafe(DRIVE_SPEED, 6.0, -90.0, 1);
        driveStraight(0.6, 38.0, -90.0);

        //Go touch bar
        driveStraight(1, -40.0, -90.0);
        turnToHeading(0.5, 179.0);
        OTMArmDrive(-2875, 1);
        driveStraight(DRIVE_SPEED, -36.0, 179.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = left_drive.getCurrentPosition() + moveCounts;
            leftTarget = left_front_drive.getCurrentPosition() + moveCounts;
            rightTarget = right_drive.getCurrentPosition() + moveCounts;
            rightTarget = right_front_drive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            left_drive.setTargetPosition(leftTarget);
            left_front_drive.setTargetPosition(leftTarget);
            right_front_drive.setTargetPosition(rightTarget);
            right_drive.setTargetPosition(rightTarget);

            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (left_drive.isBusy() && left_front_drive.isBusy() && right_drive.isBusy() && right_front_drive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void Strafe(double maxDriveSpeed, double distance, double heading, int direction) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = left_drive.getCurrentPosition() + (moveCounts * 2 * direction);
            backLeftTarget = left_front_drive.getCurrentPosition() + (moveCounts * -2 * direction);
            frontRightTarget = right_drive.getCurrentPosition() + (moveCounts * -2 * direction);
            backRightTarget = right_front_drive.getCurrentPosition() + (moveCounts * 2 * direction);

            // Set Target FIRST, then turn on RUN_TO_POSITION

            left_drive.setTargetPosition(backLeftTarget);
            left_front_drive.setTargetPosition(frontLeftTarget);
            right_front_drive.setTargetPosition(frontRightTarget);
            right_drive.setTargetPosition(backRightTarget);

            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left_front_drive.setPower(maxDriveSpeed);
            left_drive.setPower(maxDriveSpeed);
            right_front_drive.setPower(maxDriveSpeed);
            right_drive.setPower(maxDriveSpeed);



            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            //maxDriveSpeed = Math.abs(maxDriveSpeed);
            //moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (left_drive.isBusy() && left_front_drive.isBusy() && right_drive.isBusy() && right_front_drive.isBusy())) {


                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                //if (distance < 0)
                //    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                driveSpeed = maxDriveSpeed;     // save this value as a class member so it can be used by telemetry.

                if (direction>0) {
                    frontSpeed = driveSpeed - turnSpeed;
                    backSpeed = driveSpeed + turnSpeed;
                }
                if (direction<0) {
                    frontSpeed = driveSpeed + turnSpeed;
                    backSpeed = driveSpeed - turnSpeed;
                }

                // Scale speeds down if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    frontSpeed /= max;
                    backSpeed /= max;
                }

                left_front_drive.setPower(frontSpeed);
                left_drive.setPower(backSpeed);
                right_front_drive.setPower(frontSpeed);
                right_drive.setPower(backSpeed);



                // Display drive status for the driver.
                sendTelemetry(true);
            }



            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Reset encoders to make all four motor values the same for the driveStraight function
            left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
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
    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        left_front_drive.setPower(leftSpeed);
        left_drive.setPower(leftSpeed);
        right_front_drive.setPower(rightSpeed);
        right_drive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", left_front_drive.getCurrentPosition(), right_front_drive.getCurrentPosition());
        }

        else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Arm Position", "%d", arm.getCurrentPosition());
        telemetry.update();


    }
    /**
     * Read heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}
