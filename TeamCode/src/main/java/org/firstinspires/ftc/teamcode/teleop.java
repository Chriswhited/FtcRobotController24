package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Teleop", group = "Robot")
public class teleop extends OpMode {
    DcMotor left_drive;
    DcMotor left_front_drive;
    DcMotor right_drive;
    DcMotor right_front_drive;

    IMU imu;

    @Override
    public void init(){

        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");

        left_front_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        left_front_drive.setPower(front_left_power/max_power);
        left_drive.setPower(back_left_power/max_power);
        right_front_drive.setPower(front_right_power/max_power);
        right_drive.setPower(back_right_power/max_power);

    }


}
