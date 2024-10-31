package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EmreCode", group = "Robot")
//@Disabled
public class EmreCode extends LinearOpMode {

    // * define OpMode members *
    private ElapsedTime runtime = new ElapsedTime();
    private Servo wrist = null;


    // *define them variables*

    @Override
    public void runOpMode () {
        // *initialize servos *
        wrist  = hardwareMap.get(Servo.class, "wrist");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a);
            wrist.setPosition(0.8333);
            //or
            //wrist.setPosition(0);
        }

    }
    // ########################################################################################
    // !!!!                      IMPORTANT DIRECTIONS AND NOTES                           !!!!!
    // ########################################################################################
    /* notes: the code compiles just fine, but the driver hub is giving me errors, and such
    IMPORTANT: THIS WILL LIKELY STILL BE A DIFF FILE, IF SO PLEASE CREATE A NEW CLASS THEN COPY
    AND PASTE IT INTO THAT CLASS.
    once the OpMode is active press 'A' on game pad 1 and it should set the position,
    if you don't think the wrist servo was rotated all the way counter-clockwise try changing the
    set position value as such: wrist.setPosition(); it is listed above but commented with '//'
    remove the '//' and add it in front of wrist.setPosition(0.8333); If it still doesn't work,
    please contact me.
     */

}
