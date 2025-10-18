package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "WorkingTest", group = "Robot")
public class WorkingTest extends OpMode {
    DcMotor back_left_drive;
    DcMotor front_left_drive;
    DcMotor back_right_drive;
    DcMotor front_right_drive;
    //DcMotor launch_motor_1;
    DcMotor intake_motor;
    //NormalizedColorSensor color1;
    ColorSensor color1;

    IMU imu;

    boolean intake_var;
    float hsvValues[] = {0F,0F,0F};
    ElapsedTime intake_timer = new ElapsedTime();

    @Override
    public void init(){

        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        //launch_motor_1 = hardwareMap.get(DcMotor.class, "launch_motor_1");
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        //color1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        color1 = hardwareMap.get(ColorSensor.class, "color1");

        //launch_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //launch_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        //launch_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);


        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launch_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        Color.RGBToHSV(color1.red() * 8, color1.green() * 8, color1.blue() * 8, hsvValues);
        telemetry.addData("Red Value", (color1.red()));
        telemetry.addData("Green Value", (color1.green()));
        telemetry.addData("Blue Value", (color1.blue()));
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
    }
}
