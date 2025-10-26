package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop", group = "Robot")
public class teleop extends OpMode {
    DcMotor back_left_drive;
    DcMotor front_left_drive;
    DcMotor back_right_drive;
    DcMotor front_right_drive;
    DcMotor launch_motor_1;
    DcMotor intake_motor;
    Servo franklin_flipper_right;
    Servo franklin_flipper_left;

    IMU imu;

    boolean intake_var;
    boolean intake_var2;
    ElapsedTime intake_timer = new ElapsedTime();

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

        launch_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        launch_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);


        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void loop()  {
        double front_left_power = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double front_right_power = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double back_right_power = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double back_left_power = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

        double max_power = 1.0;

        max_power = Math.max(max_power, Math.abs(front_left_power));
        max_power = Math.max(max_power, Math.abs(front_right_power));
        max_power = Math.max(max_power, Math.abs(back_right_power));
        max_power = Math.max(max_power, Math.abs(back_left_power));

        front_left_drive.setPower(front_left_power/max_power);
        back_left_drive.setPower(back_left_power/max_power);
        front_right_drive.setPower(front_right_power/max_power);
        back_right_drive.setPower(back_right_power/max_power);

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
        if(gamepad1.b && intake_var2 && intake_timer.seconds() > 0.5){
            intake_motor.setPower(-1);
            intake_var = false;
            intake_timer.reset();
        }
        else if(gamepad1.b && !intake_var2 && intake_timer.seconds() > 0.5){
            intake_motor.setPower(0);
            intake_var = true;
            intake_timer.reset();
        }
        //Flywheel launcher
        if (gamepad2.a) {
            telemetry.addData("Flywheel on", gamepad2.a);
            launch_motor_1.setPower(.60);
        } else if (gamepad2.b) {
            telemetry.addData("Flywheel on", gamepad2.b);
            launch_motor_1.setPower(.80);
        } else if (gamepad2.x) {
            telemetry.addData("Flywheel on", gamepad2.x);
            launch_motor_1.setPower(.90);
        } else if (gamepad2.y) {
            telemetry.addData("Flywheel on", gamepad2.y);
            launch_motor_1.setPower(1);
        }
        else if (gamepad2.back) {
            telemetry.addData("Flywheel off", gamepad2.right_bumper);
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
            franklin_flipper_left.setPosition(0.67);
        }

    }
}
