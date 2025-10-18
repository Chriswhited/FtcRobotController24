package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp
public class LaumchTest extends OpMode {

    DcMotor launch_motor_1;
    //ColorSensor color_sensor;

    @Override
    public void init() {

        launch_motor_1 = hardwareMap.get(DcMotorEx.class, "launch_motor_1");
        //color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        //color_sensor.enableLed(true);


        launch_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        //launch_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        /*
        //Color Sensor

        float hsvValues[] = {0F,0F,0F};
        //final float values[] = hsvValues;
        Color.RGBToHSV(color_sensor.red() * 8, color_sensor.green() * 8, color_sensor.blue() * 8, hsvValues);



        telemetry.addData("Red: ", color_sensor.red());
        telemetry.addData("Green: ", color_sensor.green());
        telemetry.addData("Blue: ", color_sensor.blue());
        telemetry.addData("Hue",  hsvValues[0]);

         */
/*
        Catapult
        telemetry.addData("button_a",gamepad1.a);
        telemetry.addData("launch position", launch_motor_1.getCurrentPosition());

        if (gamepad1.a)
        {
            telemetry.addData("pressed", gamepad1.a);
            launch_motor_1.setTargetPosition(135);
            launch_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            launch_motor_1.setPower(1);
            while (launch_motor_1.isBusy()){
                telemetry.addData("Status", "Running");
                telemetry.update();

            }

            launch_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launch_motor_1.setPower(0);
        }

 */

        //Flywheel launcher
        if (gamepad1.a) {
            telemetry.addData("Flywheel on", gamepad1.a);
            launch_motor_1.setPower(.60);
        } else if (gamepad1.b) {
            telemetry.addData("Flywheel off", gamepad1.b);
            launch_motor_1.setPower(.80);
        } else if (gamepad1.x) {
            telemetry.addData("Flywheel off", gamepad1.x);
            launch_motor_1.setPower(.90);
        } else if (gamepad1.y) {
            telemetry.addData("Flywheel off", gamepad1.y);
            launch_motor_1.setPower(1);
        }
        else if (gamepad1.right_bumper) {
            telemetry.addData("Flywheel off", gamepad1.right_bumper);
            launch_motor_1.setPower(0);
        }


    }
}