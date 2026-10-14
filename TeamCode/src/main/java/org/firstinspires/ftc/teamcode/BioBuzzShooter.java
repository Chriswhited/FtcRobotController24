package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;
import org.firstinspires.ftc.teamcode.mechanisms.testconfig;

@TeleOp(name = "BioBuzzShooter", group = "Robot")
public class BioBuzzShooter extends OpMode{
    public DcMotorEx launch_motor_1;
    public DcMotorEx launch_motor_2;
    public double speedVar;
    @Override
    public void init() {


        launch_motor_1 = hardwareMap.get(DcMotorEx.class, "launch_motor_1");
        launch_motor_2 = hardwareMap.get(DcMotorEx.class, "launch_motor_2");
        speedVar = 0;
    }

    @Override
    public void loop() {

        if(speedVar < 0){
            speedVar = 0.05;
        }
        if(gamepad1.a){
            speedVar = speedVar + .05;
        }
        if(gamepad1.b){
            speedVar = speedVar - .05;
        }
        if(gamepad1.x){
            launch_motor_1.setPower(-speedVar);
            launch_motor_2.setPower(speedVar);
        }
        if(gamepad1.y){
            launch_motor_1.setPower(0);
            launch_motor_2.setPower(0);
        }
        
        telemetry.addData("V1", launch_motor_1.getVelocity());
        telemetry.addData("V2", launch_motor_2.getVelocity());
        telemetry.addData("P1", launch_motor_1.getPower());
        telemetry.addData("P2", launch_motor_2.getPower());;



    }

}
