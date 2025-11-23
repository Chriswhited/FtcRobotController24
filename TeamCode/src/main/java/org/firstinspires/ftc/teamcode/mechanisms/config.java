package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class config {

    private DcMotor back_left_drive;
    private DcMotor front_left_drive;
    private  DcMotor back_right_drive;
    private DcMotor front_right_drive;
    private DcMotor launch_motor_1;
    private DcMotor intake_motor;
    // private ColorSensor color1;
    private Servo franklin_flipper_right;
    private Servo franklin_flipper_left;
    private GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hwMap)  {
        back_left_drive = hwMap.get(DcMotor.class, "back_left_drive");
        front_left_drive = hwMap.get(DcMotor.class, "front_left_drive");
        back_right_drive = hwMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hwMap.get(DcMotor.class, "front_right_drive");
        launch_motor_1 = hwMap.get(DcMotor.class, "launch_motor_1");
        intake_motor = hwMap.get(DcMotor.class, "intake_motor");
//color1 = hwMap.get(ColorSensor.class, "color1");
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
}

