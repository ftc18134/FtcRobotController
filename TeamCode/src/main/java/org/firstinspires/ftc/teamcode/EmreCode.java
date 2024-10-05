package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MyCode", group = "Robot")
//@Disabled
public class EmreCode extends LinearOpMode {

    // * define OpMode members *
    private Servo wrist = null;


    // *define them variables*
    final double WRIST_SET_POSITION = 0;

    @Override
    public void runOpMode () {
        // *initialize servos *
        wrist  = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(WRIST_SET_POSITION);


    }



}
