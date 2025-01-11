/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the tele op period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="Robot Code", group="Code")

public class RobotCode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor extensionArm = null;// the extension arm
    public DcMotor armMotor = null;
    public Servo intake = null;

    final double TRIGGER_THRESHOLD = 0.75;

    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    final double EXTENSION_DOWN_POWER = 0.4;
    final double EXTENSION_UP_POWER = 0.3;

    final double ARM_UP_POWER = 0.3;
    final double ARM_DOWN_POWER = 0.4;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Front Left Wheel");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Back Left Wheel");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Front Right Wheel");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Back Right Wheel");
        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        extensionArm = hardwareMap.get(DcMotor.class, "Extension");
        intake = hardwareMap.get(Servo.class, "Claw");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower / 2);
                rightFrontDrive.setPower(rightFrontPower / 2);
                leftBackDrive.setPower(leftBackPower / 2);
                rightBackDrive.setPower(rightBackPower / 2);


                if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                    //Moves extension arm forwards
                    extensionArm.setDirection(DcMotor.Direction.FORWARD);
                    extensionArm.setPower(EXTENSION_UP_POWER);

                } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
                    //Moves extension arm backwards
                    extensionArm.setDirection(DcMotor.Direction.REVERSE);
                    extensionArm.setPower(EXTENSION_DOWN_POWER);

                } else if (gamepad2.left_bumper) {
                    //stops extension arm from moving
                    extensionArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    extensionArm.setPower(0);

                } else if (gamepad2.dpad_up) {
                    //Moves arm forwards
                    armMotor.setDirection(DcMotor.Direction.FORWARD);
                    armMotor.setPower(ARM_UP_POWER);

                } else if (gamepad2.dpad_down) {
                    //Moves arm backwards
                    armMotor.setPower(ARM_DOWN_POWER);
                    armMotor.setDirection(DcMotor.Direction.REVERSE);

                } else if (gamepad2.dpad_right) {
                    //stops arm from moving
                    armMotor.setPower(0);
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                } else if (gamepad2.a) {
                    //close claw
                    intake.setPosition(INTAKE_COLLECT);

                } else if (gamepad2.x) {

                    intake.setPosition(INTAKE_OFF);

                } else if (gamepad2.b) {
                    //open claw
                    intake.setPosition(INTAKE_DEPOSIT);
                }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();

                while (gamepad1.right_bumper) {
                    leftFrontDrive.setPower(leftFrontPower * 2);
                    rightFrontDrive.setPower(rightFrontPower * 2);
                    leftBackDrive.setPower(leftBackPower * 2);
                    rightBackDrive.setPower(rightBackPower * 2);
                }

                while (gamepad2.right_bumper) {
                    extensionArm.setPower(EXTENSION_UP_POWER / 2);
                    extensionArm.setPower(EXTENSION_DOWN_POWER / 2);
                    armMotor.setPower(ARM_DOWN_POWER / 2);
                    armMotor.setPower(ARM_UP_POWER / 2);
                }

            }
        }
    }
}