/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp(name="Robot Hardware Main")
//@Disabled
public class RobotHardwareMain extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor armMotor = null; //the arm motor
    public DcMotor extensionArm = null; //the arm extension motor
    public CRServo intake = null; //the active intake servo

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* the threshold that must be passed for a trigger press to register */
    final double TRIGGER_THRESHOLD = 0.75;


    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        /* Define and Initialize Motors and Servos */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Front Left Wheel");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Back Left Wheel");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Front Right Wheel");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Back Right Wheel");
        armMotor = hardwareMap.get(DcMotor.class, "Arm"); //the arm motor
        extensionArm = hardwareMap.get(DcMotor.class, "Extension"); // the arm extension motor
        intake = hardwareMap.get(CRServo.class, "Intake");//the intake


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();
        intake.setPower(INTAKE_OFF);

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            double max;

            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;


            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.right_bumper) {
                leftFrontDrive.setPower(leftFrontPower/2);
                rightFrontDrive.setPower(rightFrontPower/2);
                leftBackDrive.setPower(leftBackPower/2);
                rightBackDrive.setPower(rightBackPower/2);
            } else {
                leftFrontDrive.setPower(leftFrontPower*1.25);
                rightFrontDrive.setPower(rightFrontPower*1.25);
                leftBackDrive.setPower(leftBackPower*1.25);
                rightBackDrive.setPower(rightBackPower*1.25);
            }

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extensionArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            /*--------------------------------!!!IMPORTANT!!!---------------------------------------
                                  !!!game pad mapping for game pad 2!!!
            a = set intake to collect
            x = turn off intake
            b = set intake to deposit

            RB = arm move backwards
            LB = arm move forwards
            RT = moves extension arm outward
            LT = moves extension arm inward
             */

            if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);

            } else if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad2.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                //Moves extension arm forwards
                extensionArm.setDirection(DcMotor.Direction.REVERSE);
                extensionArm.setPower(0.7);
            } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
                //Moves extension arm backwards
                extensionArm.setDirection(DcMotor.Direction.FORWARD);
                extensionArm.setPower(0.7);
            } else if (gamepad2.right_stick_button) {
                extensionArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                extensionArm.setPower(0);
            }

            else if (gamepad2.left_bumper) {
                armMotor.setDirection(DcMotor.Direction.FORWARD);
                armMotor.setPower(0.8);
            }

            else if (gamepad2.right_bumper) {
                armMotor.setPower(0.8);
                armMotor.setDirection(DcMotor.Direction.REVERSE);

            } else if (gamepad2.left_stick_button) {
                armMotor.setPower(0);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}