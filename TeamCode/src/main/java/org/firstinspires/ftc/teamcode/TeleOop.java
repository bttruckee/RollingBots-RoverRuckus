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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMod
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class TeleOop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Rover rover = null;
    private boolean markerLocked;
    private boolean isLocked;
    private boolean endGame;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Not Initialized");

        // Initialize the rover as an object

        rover = new Rover();
        rover.init(hardwareMap);
        markerLocked = true;
        isLocked = true;
        endGame = false;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("init", "loop");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("start", "once");
        telemetry.update();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.y) {
            endGame = true;
        }
        telemetry.addData("Endgame", endGame);

        if(!endGame) {
            //double rightPower;
            //double leftPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x;
            telemetry.addData("Forward", drive);
            telemetry.addData("Turn", turn);

            //leftPower    = Range.clip(drive + turn, 1.0, -1.0) ;
            //rearLeftPower    = Range.clip(drive + turn, 1.0, -1.0) ;
            //frontRightPower  = Range.clip(drive - turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            rover.setForwardSpeed(drive);

            rover.setTurnSpeed(turn);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.


            /*frontLeftPower  = gamepad1.left_stick_y ;
            rearLeftPower  = gamepad1.left_stick_y ;
            frontRightPower = gamepad1.right_stick_y ;
            rearRightPower = gamepad1.right_stick_y ;

            frontLeftPower += gamepad1.right_trigger;
            rearLeftPower += -gamepad1.right_trigger;
            rearRightPower += gamepad1.right_trigger;
            frontRightPower += -gamepad1.right_trigger;

            frontLeftPower += -gamepad1.left_trigger;
            rearLeftPower += gamepad1.left_trigger;
            frontRightPower += gamepad1.left_trigger;
            rearRightPower += -gamepad1.left_trigger;*/

            /*frontLeftPower = gamepad1.left_stick_y;
            rearLeftPower = gamepad1.left_stick_x;
            frontRightPower = gamepad1.right_stick_y;
            rearRightPower = gamepad1.right_stick_x;*/

            // Send calculated power to wheels
            rover.move();

            /*rightRear.setPower(rightPower);
            leftRear.setPower(leftPower);
            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);*/
            int inOrOut = 1;
            if (gamepad2.right_bumper) {
                inOrOut = 2;
            } else if (gamepad2.left_bumper) {
                inOrOut = 0;
            }

            telemetry.addData("in or out", inOrOut);

            double clampArmDirection = gamepad2.left_stick_y;
            rover.moveArm(clampArmDirection, inOrOut);

            if(gamepad1.dpad_left)
            {
                markerLocked = false;
            } else if(gamepad1.dpad_right) {
                markerLocked = true;
            }

            rover.setMarkerLock(markerLocked);

            if (gamepad2.dpad_left) {
                isLocked = false;
            } else if (gamepad2.dpad_right) {
                isLocked = true;
            }

            rover.setArmLocks(isLocked);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, rearLeftPower, frontRightPower, rearRightPower);
        }
        else
        {
            rover.moveArm(-0.5, 1);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
        rover.stop();

        telemetry.addData("Status", "Stopped");
    }

}