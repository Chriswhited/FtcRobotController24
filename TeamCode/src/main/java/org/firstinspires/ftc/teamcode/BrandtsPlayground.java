package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Disabled
@TeleOp(name = "BrandtsPlayground", group = "Robot")
public class BrandtsPlayground extends OpMode {
    DcMotor back_left_drive;
    DcMotor front_left_drive;
    DcMotor back_right_drive;
    DcMotor front_right_drive;
    DcMotor launch_motor_1;
    DcMotor intake_motor;
    Servo franklin_flipper_right;
    Servo franklin_flipper_left;
    GoBildaPinpointDriver pinpoint;
    IMU imu;

    private Limelight3A limelight;

    boolean intake_var;
    boolean intake_var2;
    double max_power = 1.0;
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
    ElapsedTime intake_timer = new ElapsedTime();
    ElapsedTime auto_timer = new ElapsedTime();
    float aimP = 0;
    float aimmin = 0;

    @Override
    public void init(){

        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        launch_motor_1 = hardwareMap.get(DcMotor.class, "launch_motor_1");
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        franklin_flipper_right = hardwareMap.get(Servo.class, "franklin_flipper_right");
        franklin_flipper_left = hardwareMap.get(Servo.class, "franklin_flipper_left");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        //configurePinpoint();
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        launch_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
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

    }
    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop()  {

        LLResult llresult = limelight.getLatestResult();
        Pose3D botpose = llresult.getBotpose();
        List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
        double tx = llresult.getTx();

        telemetry.addData("tx", llresult.getTx());
        telemetry.addData("ty", llresult.getTy());
        telemetry.addData("ta", llresult.getTa());

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            telemetry.addData("id", id);
        }
        double steeringadjust = 0;
        if (gamepad1.dpad_down){
            double aimherror = tx;
            if (Math.abs(aimherror) > 1.0){
                if (aimherror < 0){
                    steeringadjust = aimP * aimherror + aimmin;
                } else {
                    steeringadjust = aimP * aimherror - aimmin;
                }
            }
        }
        double rotation = gamepad1.right_stick_x + steeringadjust;
        double front_left_power = -gamepad1.left_stick_y + gamepad1.left_stick_x - rotation;
        double front_right_power = -gamepad1.left_stick_y - gamepad1.left_stick_x + rotation;
        double back_right_power  = -gamepad1.left_stick_y + gamepad1.left_stick_x + rotation;
        double back_left_power   = -gamepad1.left_stick_y - gamepad1.left_stick_x - rotation;

/*
        double front_left_power = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double front_right_power = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double back_right_power = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double back_left_power = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
 */
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
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
            front_left_drive.setPower(front_left_power/(max_power*2));
            back_left_drive.setPower(back_left_power/(max_power*2));
            front_right_drive.setPower(front_right_power/(max_power*2));
            back_right_drive.setPower(back_right_power/(max_power*2));
        }
        else {
            front_left_drive.setPower(front_left_power / max_power);
            back_left_drive.setPower(back_left_power / max_power);
            front_right_drive.setPower(front_right_power / max_power);
            back_right_drive.setPower(back_right_power / max_power);
        }



/*
        front_left_drive.setPower(front_left_power / max_power);
        back_left_drive.setPower(back_left_power / max_power);
        front_right_drive.setPower(front_right_power / max_power);
        back_right_drive.setPower(back_right_power / max_power);


 */
        //start kolby kage
        if(gamepad1.a && intake_var && intake_timer.seconds() > 0.5){
            intake_motor.setPower(1);
            intake_var = false;
            intake_timer.reset();
        }
        else if(gamepad1.a && !intake_var && intake_timer.seconds() > 0.5){
            intake_motor.setPower(0);
            intake_var = true;
            intake_timer.reset();
        }
        //reverse kolby kage
        if(gamepad1.b && intake_var2 && intake_timer.seconds() > 0.5){
            intake_motor.setPower(-1);
            intake_var2 = false;
            intake_timer.reset();
        }
        else if(gamepad1.b && !intake_var2 && intake_timer.seconds() > 0.5){
            intake_motor.setPower(0);
            intake_var2 = true;
            intake_timer.reset();
        }
        //Far shoot pos
        if (gamepad1.y) {
            auto_timer.reset();
            launch_motor_1.setPower(0.7);
            odometryDrive(2.5,2.2,-22, xMaxSpeed);
        }
        //Flywheel launcher
        if (gamepad2.a) {
            launch_motor_1.setPower(.54);
            telemetry.addData("Flywheel on", launch_motor_1.getPower()*100);
        } else if (gamepad2.b) {
            launch_motor_1.setPower(.55);
            telemetry.addData("Flywheel on", launch_motor_1.getPower()*100);
        } else if (gamepad2.x) {
            launch_motor_1.setPower(.60);
            telemetry.addData("Flywheel on", launch_motor_1.getPower()*100);
        } else if (gamepad2.y) {
            launch_motor_1.setPower(.70);
            telemetry.addData("Flywheel on", launch_motor_1.getPower()*100);
        }
        else if (gamepad2.back) {
            telemetry.addLine("Flywheel off");
            launch_motor_1.setPower(0);
        }

        //franklin flipper right
        if(gamepad2.right_trigger > 0.5) {
            franklin_flipper_right.setPosition(0.11);
             
        }
        else{
            franklin_flipper_right.setPosition(0.44);
        }

        //franklin flipper left
        if(gamepad2.left_trigger > 0.5) {
            franklin_flipper_left.setPosition(1);
        }
        else{
            franklin_flipper_left.setPosition(0.64
            );
        }

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

        //while(gamepad1.y && Math.abs(xError) > 1.5 || Math.abs(yError) > 1.5 || Math.abs(hError) > 4){
            //auto_timer.seconds() < .001
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
        telemetry.addData("Time", auto_timer);
        telemetry.addData("YButton", gamepad1.y);
        telemetry.update();

        double x = Range.clip((xProp * xError) + (xInt * integralSumX) + (xDer * derivativeX),-xMaxSpeed,xMaxSpeed);
        double y = Range.clip((yProp * yError) + (yInt * integralSumY) + (yDer * derivativeY),-yMaxSpeed,yMaxSpeed);
        double h = Range.clip(hError * hProp, -hMaxSpeed, hMaxSpeed);


        moveRobot(x, y, h);

        lastErrorX = xError;
        lastErrorY = yError;
        timer.reset();
            /*
            if (gamepad1.left_stick_x > 0.5 || gamepad1.left_stick_y > 0.5){
                break;
            }

             */

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
        while(auto_timer.seconds() < .01){
            
        }
    }
}
