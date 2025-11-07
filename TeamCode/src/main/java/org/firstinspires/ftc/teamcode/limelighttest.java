package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class limelighttest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(2);
    imu = hardwareMap.get(IMU.class, "imu");
    RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        Pose3D botpose = llresult.getBotpose_MT2();
        telemetry.addData("tx", llresult.getTx());
        telemetry.addData("ty", llresult.getTy());
        telemetry.addData("ta", llresult.getTa());
        List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            telemetry.addData("id", id);
        }
        }
    }
