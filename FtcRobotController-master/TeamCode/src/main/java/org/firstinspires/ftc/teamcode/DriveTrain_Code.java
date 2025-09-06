/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
/*
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a Robot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: DriveTrain with keypad", group = "Concept")
//@Disabled
public class DriveTrain_Code extends LinearOpMode {

    static final double INCREMENT   = 0.25;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   100;     // period of each cycle
    static final double MAX_FWD     =  3.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -3.0;     // Maximum REV power applied to motor

    // Define class members
    DcMotor top_left;
    double  top_left_power   = 0;
    //boolean rampUp  = true;

    DcMotor bottom_left;

    double bottom_left_power = 0;

    //boolean rampUp2= true;

    DcMotor top_right;

    double top_right_power = 0;

    DcMotor bottom_right;

    double bottom_right_power = 0;


    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        top_left = hardwareMap.get(DcMotor.class, "top_left");
        bottom_left = hardwareMap.get(DcMotor.class, "bottom_left");
        top_right = hardwareMap.get(DcMotor.class, "top_right");
        bottom_right = hardwareMap.get(DcMotor.class, "bottom_right");
        top_left.setDirection(DcMotor.Direction.REVERSE);
        top_right.setDirection(DcMotor.Direction.FORWARD);
        bottom_left.setDirection(DcMotor.Direction.FORWARD);
        bottom_right.setDirection(DcMotor.Direction.REVERSE);
        top_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        top_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();


        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            // Ramp the motors, according to the rampUp variable.
            //move forward
            if (gamepad1.left_stick_y == -1.0) {
                // Keep stepping up until we hit the max value.
                telemetry.addData(">", "right bumper pressed" );
                telemetry.update();
                top_left_power = INCREMENT ;
                top_right_power = INCREMENT;
                bottom_left_power = -INCREMENT;
                bottom_right_power = -INCREMENT;

            }
            else if (gamepad1.left_stick_y == 1.0) {
                // Keep stepping down until we hit the min value.
                //move backward
                top_left_power = -INCREMENT ;
                top_right_power = -INCREMENT;
                bottom_left_power = INCREMENT;
                bottom_right_power = INCREMENT;
                telemetry.addData(">", "left bumper pressed" );
                telemetry.update();


            }

            if (gamepad1.left_stick_x == -1.0) {
                //strafe left
                telemetry.addLine("Left stick activated");
                telemetry.update();
                top_left_power = -INCREMENT;
                top_right_power = INCREMENT;
                bottom_left_power = -INCREMENT;
                bottom_right_power = INCREMENT;

            }
            else if (gamepad1.left_stick_x == 1.0) {
                //strafe right
                top_left_power = INCREMENT;
                top_right_power = -INCREMENT;
                bottom_left_power = INCREMENT;
                bottom_right_power = -INCREMENT;


            }

            if (gamepad1.right_stick_x == -1.0) {
                //turn right
                top_left_power = -INCREMENT;
                top_right_power = INCREMENT;
                bottom_left_power = INCREMENT;
                bottom_right_power = -INCREMENT;
            }

            else if (gamepad1.right_stick_x == 1.0) {
                // turn left
                top_left_power = INCREMENT;
                top_right_power = -INCREMENT;
                bottom_left_power = -INCREMENT;
                bottom_right_power = INCREMENT;
            }

            if (gamepad1.left_stick_y == 0) {
                //disable all motor power
                top_left_power = 0;
                top_right_power = 0;
                bottom_left_power = 0;
                bottom_right_power = 0;
            }

            // Display the current value
            //telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            top_left.setPower(top_left_power);
            top_right.setPower(top_right_power);
            bottom_left.setPower(bottom_left_power);
            bottom_right.setPower(bottom_right_power);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        top_left.setPower(0);
        top_right.setPower(0);
        bottom_left.setPower(0);
        bottom_right.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();


    }
}
