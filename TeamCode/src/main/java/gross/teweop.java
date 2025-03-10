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

package gross;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import gross.TECHMAP;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//start position thingy Ltarget = 440 , Rtarget = 439

@TeleOp(name="pain", group="COMP")
//@Disabled
public class teweop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    TECHMAP robot   = new TECHMAP();
    double SpeedAdjust = 1;
    double toggleTime = .25;
    ElapsedTime toggle = new ElapsedTime(); //STOP


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //init lift motorrs
        robot.L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // robot.R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftClaw.setPosition(L_OPEN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        START_POS();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive equation
            drive();


            if (gamepad2.dpad_up) {
                HIGH();
            } else if (gamepad2.dpad_left) {
                MID();
            } else if (gamepad2.dpad_down) {
                LOW();
            } else if (gamepad2.dpad_right) {
                MID();
            } else if (gamepad2.cross) {
                START_POS();
            }

           
        }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Endgame:", endgame);
            telemetry.update();
    }
}


    //TODO: put in an auto NVM I BROKW IT

//tune lift positions

    public void START_POS(){
    lift(230);
    }

public void LOW(){
    lift(1208);
}

public void MID(){
    lift(2078);
}

public void HIGH(){
    lift(2900);
}


    public void lift(double counts) {

        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = (int) counts;
            //inches *
            robot.L.setTargetPosition(newTarget);
           // robot.R.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // robot.R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.L.setPower(Math.abs(-1)); //left arm positive
            //robot.R.setPower(Math.abs(1)); //right arm negative

            while (opModeIsActive()
                    //&& (runtime.seconds() < timeoutS)
                    &&
                    (robot.L.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d");
                telemetry.addData("Path2", "Running at %7d :%7d");
                telemetry.update();
                drive();
            }
        }
    }






//TODO: change the drive eq to mecdrive one

    private void drive() {


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = -(y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = -(y + x - rx) / denominator;

        robot.leftFront.setPower(frontLeftPower*1);
        robot.leftBack.setPower(backLeftPower*1);
        robot.rightFront.setPower(frontRightPower*1);
        robot.rightBack.setPower(backRightPower*1);


        //todo idk why this doesnt work :(
        if (gamepad1.left_bumper) {
             SpeedAdjust = 4;
        } else if (gamepad1.right_bumper) {
            SpeedAdjust = 1;
        }
    }



