
package gross;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */



//TODO: Double check hw map bc idk if the directions are right


@Config
public class TECHMAP {

    
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;


     public DcMotor L = null;
    public DcMotor intake = null;
    public DcMotor hang = null;
    public DcMotor hang2 = null;

    public Servo wristL = null;
    public Servo wristR = null;
    public Servo bucket = null;
    public Servo horiz = null;
  //  public Servo turn = null;



   // public static final double R_CLOSE    =  0.67 ;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TECHMAP(){

    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                .008,0,0,.06
        );

        L.setPIDFCoefficients(runMode, compensatedCoefficients);
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
       // TODO: Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "frontL"); //drive stuff is in the expansion hub
        rightFront = hwMap.get(DcMotor.class, "frontR");
        leftBack  = hwMap.get(DcMotor.class, "backL");
        rightBack = hwMap.get(DcMotor.class, "backR");


        //TODO: ADD THE LIFT??? WHERE IS THE LIFT
        wristL = hwMap.get(Servo.class, "wristL");
        wristR = hwMap.get(Servo.class, "wristR");
        bucket = hwMap.get(Servo.class, "bucket");
        horiz = hwMap.get(Servo.class, "horiz");
        L = hwMap.get(DcMotor.class, "lift");
      //  turn = hwMap.get(Servo.class, "RE");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        hang.setDirection(DcMotor.Direction.REVERSE);
        hang2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        horiz.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.REVERSE);
        wristL.setDirection(Servo.Direction.REVERSE);
        wristR.setDirection(Servo.Direction.FORWARD);


        //can be set to coast if you want to drive funny but this might suck in auton

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//

        // Define and initialize ALL installed servos.

  //      leftClaw = hwMap.get(Servo.class, "lc");
     //   rightClaw = hwMap.get(Servo.class, "rc");
      //  duck = hwMap.get(Servo.class,"duck");

        //rightClaw.setPosition(R_CLOSE);

    }
}
