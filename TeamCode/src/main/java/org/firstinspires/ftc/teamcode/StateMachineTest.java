package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.configwLimeLight;

@TeleOp(name = "StateMachineTest", group = "Robot")
public class StateMachineTest {
    configwLimeLight conf = new configwLimeLight();
    String currentState;


    public void init() {
        currentState = "IDLE";
    }
    

    public void loop() {

        switch(currentState){
            case "IDLE":
            case "DRIVE":
            case "FAR":
            case "CLOSE":

        }
    }

    enum State {
        IDLE,
        DRIVE,

        FAR,
        CLOSE
    }
}
