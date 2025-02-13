
package gross;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class botmap
{
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor larm = null;
    public DcMotor rarm = null;
    public DcMotor wrist = null;
    public DcMotor intake = null;

    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo duck   = null;


   // public static final double R_CLOSE    =  0.67 ;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public botmap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "lf"); //0
        rightFront = hwMap.get(DcMotor.class, "rf"); //2
        leftBack  = hwMap.get(DcMotor.class, "lb"); //1
        rightBack = hwMap.get(DcMotor.class, "rb"); //3
        // Duck2 = hwMap.get(DcMotor.class, "d2");
        larm  = hwMap.get(DcMotor.class, "la"); //0 ex
        rarm = hwMap.get(DcMotor.class, "ra");
        intake = hwMap.get(DcMotor.class, "intake");
        wrist = hwMap.get(DcMotor.class, "w");
        //arm2  = hwMap.get(DcMotor.class, "a2");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        larm.setDirection(DcMotor.Direction.REVERSE);
        rarm.setDirection(DcMotor.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        larm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        larm.setPower(0);
        rarm.setPower(0);
        intake.setPower(0);
        wrist.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.

        leftClaw = hwMap.get(Servo.class, "lc");
        rightClaw = hwMap.get(Servo.class, "rc");
        duck = hwMap.get(Servo.class,"duck");

        //rightClaw.setPosition(R_CLOSE);

    }
}
