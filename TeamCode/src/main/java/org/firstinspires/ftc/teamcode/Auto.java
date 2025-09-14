package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//Port 0 had arm motor
//port 1 is extension
@Autonomous(name="Robot: Auto", group="Robot")
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  armMotor    = null;
    public CRServo intake       = null;
    public DcMotor extensionArm = null;


    public ElapsedTime runtime      = new ElapsedTime();
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightBackDrive   = null;

    static final double     FORWARD_SPEED = 0.6;
    final double INTAKE_DEPOSIT = 0.5;
    final double INTAKE_OFF = 0.0;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Front Left Wheel");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Back Left Wheel");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Front Right Wheel");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Back Right Wheel");
        intake = hardwareMap.get(CRServo.class, "Intake");
        armMotor   = hardwareMap.get(DcMotor.class, "Arm");
        extensionArm = hardwareMap.get(DcMotor.class,"Extension");



        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Auto - Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Drive for 2 Seconds
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            leftFrontDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        runtime.reset();
        // Move the Arm motor
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive() && (runtime.seconds() < 4.5)) {
            armMotor.setPower(0.4);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        armMotor.setPower(0);
        runtime.reset();
        // Extend the arm
        extensionArm.setDirection(DcMotor.Direction.REVERSE);
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            extensionArm.setPower(0.4);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();


        }
        extensionArm.setPower(0);
        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            intake.setPower(INTAKE_DEPOSIT);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        intake.setPower(INTAKE_OFF);
        runtime.reset();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //System.exit(0);
    }
}
