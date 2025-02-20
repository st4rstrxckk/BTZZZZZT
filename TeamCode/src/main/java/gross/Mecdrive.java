package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
//@Disabled
public class Mecdrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backR");
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor hang = hardwareMap.dcMotor.get("hang");
        DcMotor hang2 = hardwareMap.dcMotor.get("hang2");

        Servo wristL;
        Servo wristR;
        Servo bucket;
        Servo horizontal;
        Servo turn;

        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        bucket = hardwareMap.get(Servo.class, "bucket");
        horizontal = hardwareMap.get(Servo.class, "horiz");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //prev forward
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //prev forward
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //lift.setDirection(DcMotorSimple.Directions.REVERSE);


        horizontal.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.REVERSE);
        wristL.setDirection(Servo.Direction.FORWARD);
        wristR.setDirection(Servo.Direction.REVERSE);

        // wristL.setPosition(0);
        // wristR.setPosition(0);
        // bucket.setPosition(0.3);
        // horizontal.setPosition(0.2);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            frontLeftMotor.setPower(frontLeftPower*1);
            backLeftMotor.setPower(backLeftPower*1);
            frontRightMotor.setPower(frontRightPower*1);
            backRightMotor.setPower(backRightPower*1);


            // intake.setPower(0);
            // hang.setPower(0);
            // hang2.setPower(0);


            if(gamepad1.a){
                horizontal.setPosition(.5); //dec from .6

                wristL.setPosition(.8);
                wristR.setPosition(.8);
            }

            if(gamepad1.b){
                horizontal.setPosition(0.2);
                wristL.setPosition(0);
                wristR.setPosition(0);


            }

            if(gamepad1.left_bumper){
                intake.setPower(1);
            }
            if(gamepad1.right_bumper){
                intake.setPower(-.7); //power inc from -.3
            }
            if(gamepad1.right_trigger>.1){
                intake.setPower(0);
            }
            if(gamepad1.dpad_down){
                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(-.5);
                frontRightMotor.setPower(-.5);
                backRightMotor.setPower(.5);
            }
            if(gamepad1.dpad_up){
                frontLeftMotor.setPower(-.5);
                backLeftMotor.setPower(.5);
                frontRightMotor.setPower(.5);
                backRightMotor.setPower(-.5);
            }
            if(gamepad1.dpad_left){
                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(.5);
                frontRightMotor.setPower(.5);
                backRightMotor.setPower(.5);
            }
            if(gamepad1.dpad_right){
                frontLeftMotor.setPower(-.5);
                backLeftMotor.setPower(-.5);
                frontRightMotor.setPower(-.5);
                backRightMotor.setPower(-.5);
            }
            if(gamepad2.dpad_down){
                hang.setPower(.90);
                hang2.setPower(1);
            }
            if(gamepad2.dpad_right){
                hang.setPower(0);
                hang2.setPower(0);
            }






            // if(gamepad2.b){
            // hang.setPower(1);
            // }

            // if(gamepad2.y){
            // hang2.setPower(1);
            // }

            // if(gamepad2.a){
            // hang.setPower(-1);
            // }

            // if(gamepad2.x){
            // hang2.setPower(-1);
            // }

            if(gamepad2.left_bumper){
                bucket.setPosition(0.3);
            }

            if(gamepad2.right_bumper){
                bucket.setPosition(.63);
            }
            // keep these very specific numbers
            if(gamepad2.left_trigger>.1){
                lift.setPower(.89);
            }
            else if(gamepad2.right_trigger>.1){
                lift.setPower(-.91);
            }else{
                lift.setPower(0);
            }


            if(gamepad2.dpad_up){
                hang.setPower(-.90);
                hang2.setPower(-1);
            }
        }

    }
}