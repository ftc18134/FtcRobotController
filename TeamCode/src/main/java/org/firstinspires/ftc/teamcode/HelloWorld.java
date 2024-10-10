package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class HelloWorld extends OpMode {
    DcMotor frontLeftWheel = null;
    DcMotor frontRightWheel = null;
    DcMotor backLeftWheel = null;
    DcMotor backRightWheel = null;
    DcMotor arm = null;

    //port 0 is back left
    //port 1 is front right
    //port 2 is back right
    //port 3 is Front left

    @Override
    public void init() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "Front Left Wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "Front Right Wheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "Back Right Wheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "Back Left Wheel");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addData("Motoring", "Running");
            frontLeftWheel.setPower(-0.5);
            frontRightWheel.setPower(0.5);
            backRightWheel.setPower(-0.5);
            backLeftWheel.setPower(0.5);
        } else {
            frontLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backRightWheel.setPower(0);
            backLeftWheel.setPower(0);
        }
        if (gamepad1.b) {
            telemetry.addData("Motoring", "Running");
            frontLeftWheel.setPower(0.5);
            frontRightWheel.setPower(-0.5);
            backRightWheel.setPower(0.5);
            backLeftWheel.setPower(-0.5);
        } else {
            frontLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backRightWheel.setPower(0);
            backLeftWheel.setPower(0);
        }
        if (gamepad1.dpad_right) {
            frontLeftWheel.setPower(-0.5);
            frontRightWheel.setPower(-0.5);
            backRightWheel.setPower(-0.5);
            backLeftWheel.setPower(-0.5);
        }
    }
}
