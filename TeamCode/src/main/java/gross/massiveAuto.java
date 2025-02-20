package gross;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//import Tessts.PoseStorage;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(name = "im so mad", group = "COMP")
@Disabled

//  _______ _     _ _____ _______      ______   _____  _______ _______ __   _ _______      _  _  _  _____   ______ _     _
//     |    |_____|   |   |______      |     \ |     | |______ |______ | \  |    |         |  |  | |     | |_____/ |____/
//     |    |     | __|__ ______|      |_____/ |_____| |______ ______| |  \_|    |         |__|__| |_____| |    \_ |    \_



//DONT LOOK AT THIS ITS NOT GOOD

public class massiveAuto extends LinearOpMode {

    public PIDController controller;

    //TODO: edit variables on dashboard lol

    // p = increase if not reaching target position

 //   public static double p = .006, i = 0, d = 0; // d = dampener (dampens arm movement and is scary). ignore i
 //   public static double f = .05;  // prevents arm from falling from gravity

    public static int LiftTarget = 0; // target position


    public static int LOW = 0; //1208 = LOW
    public static int MID = 180; //2078 = MID
    public static int HIGH = 500; //2900 = HIGH


    double OPEN = .01;
    double CLOSE = 0.1376;

    double startPos = 0.69;



    public static int left = 1;
    public static int center = 1;
    public static int right = 1;






    /*GENERAL IDEA
     * go forward while lifting the lift
     * turn
     * bring out arm
     * drop cone  (VERY IMPORTANT THAT THESE ARE SEPARATE)
     * bring arm back in and  drop lift to cone 5 while doing spline
     * go towards cone
     * grab
     * lift and go to deliver trajectory
     * flip arm
     *
     * */

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        START,
        TURN_1,
        WAIT_1,
        FLIP1,

        IDLE// Our bot will enter the IDLE state when done
    }

    enum elbowUpState {
        START,
        GRAB,
        LIFT_TWIST,
        OUTTAKE,
        IDLE
    }
    enum elbowDownState {
        START,
        CLOSE,
        LIFT_TWIST,
        OUTTAKE,
        IDLE
    }
    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;
    elbowUpState elbowUp = elbowUpState.IDLE;
    elbowDownState elbowDown = elbowDownState.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift

        Lift lift = new Lift(hardwareMap);
        // colorS colorS = new colorS(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TECHMAP robot = new TECHMAP();

        // Set inital pose based on MeepMeep
        Pose2d startPose = new Pose2d(35, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // TODO: DEFINE AND NAME TRAJECTORIES
        Trajectory START = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36,-18.00, Math.toRadians(348.00)))
                .build();


        TrajectorySequence LINE_AND_CONES1 = drive.trajectorySequenceBuilder(START.end())
                .lineToLinearHeading(new Pose2d(36.61, -8.50, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(65.250, -8.00, Math.toRadians(0.00)))
                .build();


        //Pose2d newPose1 = new Pose2d(63.50, -9.75,LINE_AND_CONES1.end().getHeading());

        //drive.setPoseEstimate(newPose1);
        //TODO RELOCALAIZE
        Trajectory DELIVER1 = drive.trajectoryBuilder(LINE_AND_CONES1.end(),true)
                .splineTo(new Vector2d(35.00, -15.50), Math.toRadians(-152.48))
                .build();

        Trajectory CONES2 = drive.trajectoryBuilder(DELIVER1.end())
                .splineTo(new Vector2d(64.50, -9.00), Math.toRadians(0.00))
                .build();

        Trajectory DELIVER2 = drive.trajectoryBuilder(CONES2.end(),true)
                .splineTo(new Vector2d(35.00, -15.50), Math.toRadians(-152.48))
                .build();

        Trajectory CONES3 = drive.trajectoryBuilder(DELIVER2.end())
                .splineTo(new Vector2d(64.75, -9.750), Math.toRadians(0.00))
                .build();

        Trajectory DELIVER3 = drive.trajectoryBuilder(CONES3.end(),true)
                .splineTo(new Vector2d(35.00, -15.5), Math.toRadians(-152.48))
                .build();

        Trajectory CONES4 = drive.trajectoryBuilder(DELIVER3.end())
                .splineTo(new Vector2d(64.500, -10.00), Math.toRadians(0.00))
                .build();

        Trajectory DELIVER4 = drive.trajectoryBuilder(CONES4.end(),true)
                .splineTo(new Vector2d(30.00, -15.5), Math.toRadians(-152.48))
                .build();


        Trajectory PARK_C = drive.trajectoryBuilder(DELIVER4.end())
                .splineTo(new Vector2d(63.34, -15.50), Math.toRadians(0.00))
                .build();

        Trajectory PARK_B = drive.trajectoryBuilder(DELIVER4.end())
                .forward(2.5)
                .build();

        TrajectorySequence PARK_A = drive.trajectorySequenceBuilder(DELIVER4.end())
                .lineTo(new Vector2d(36, -3))
                .lineToLinearHeading(new Pose2d(9,-5.00,Math.toRadians(0)))
                .build();




        // Define a 1.5 second wait time
        double upTime = .5;
        double upTime1 = 1.2;
        ElapsedTime upTimer = new ElapsedTime();

        double downTime = .5;
        double downTime1 = 1.0;
        ElapsedTime downTimer = new ElapsedTime();

        double waitTime1 = .5;
        ElapsedTime waitTimer = new ElapsedTime();

        double waitTime2 = .3;

        double waitTime3 = 0.2;
        double waitTime4 = 0.8;

        telemetry.setMsTransmissionInterval(50);

        robot.init(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

//program is running now, these are starting commands
        drive.followTrajectoryAsync(START);
        LiftTarget = MID;
        robot.grab.setPosition(CLOSE);
        upTimer.reset();
        elbowUp = elbowUpState.START;
        currentState = State.TURN_1;
        elbowDown = elbowDownState.IDLE;

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            if (isStopRequested()) return;
            // We essentially define the flow of the state machine through this switch statement

            switch (currentState) {
                case TURN_1:
                    if (!drive.isBusy()) {
                        //  drive.followTrajectoryAsync(START1);
                        robot.grab.setPosition(OPEN);
                        currentState = State.WAIT_1;
                    }
                    break;
                case WAIT_1:
                    if (elbowUp == elbowUpState.IDLE) {
                        currentState = State.FLIP1;
                        upTime1 = .65;
                        waitTimer.reset();
                    }
                    break;
                case FLIP1:
                    if ((waitTimer.seconds() >= waitTime1) && !drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(LINE_AND_CONES1);
                        //drive.followTrajectoryAsync(LINE1);
                        LiftTarget = 170;//cone 5
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES1;
                    }
                    break;
//                case SPLINE1:
//                    if (!drive.isBusy()) {
//                        drive.followTrajectoryAsync(CONES1);
//                        currentState = State.CONES1;
//                    }
//                    break;
//                case CONES1:
//                    if (!drive.isBusy()) {
//                        //drive.followTrajectoryAsync(SLOWCONES1);
//                        if (left == gray && center == gray && right == notGray){
//                            //aka left + center see tile and right sees tape
//                            drive.followTrajectoryAsync(Strafe_SEESRIGHTONLY);
//                            drive.setPoseEstimate(newPose1);
//                            currentState = State.STRAFE_ADJ1;
//
//                        } else if (left == gray && center == notGray && right == notGray) {
//                            //aka left sees tile and center + right see tape
//                            drive.followTrajectoryAsync(Strafe_SEESRIGHTANDCENTER);
//                            drive.setPoseEstimate(newPose1);
//                            currentState = State.STRAFE_ADJ1;
//
//                        } else if (left == notGray && center == notGray && right == notGray) {
//                            //aka left + center + right see tape
//                            drive.setPoseEstimate(newPose1);
//                            currentState = State.STRAFE_ADJ1;
//
//                        } else if (left == notGray && center == notGray && right == gray) {
//                            //aka left + center see tape and right sees tile
//                            drive.followTrajectoryAsync(Strafe_SEESLEFTANDCENTER);
//                            drive.setPoseEstimate(newPose1);
//                            currentState = State.STRAFE_ADJ1;
//
//                        } else if (left == notGray && center == gray && right == gray) {
//                            //aka left sees tape and center + right see tile
//                            drive.followTrajectoryAsync(Strafe_SEESLEFTONLY);
//                            drive.setPoseEstimate(newPose1);
//                            currentState = State.STRAFE_ADJ1;
//                        } else {
//                            drive.setPoseEstimate(newPose1);
//                            currentState = State.STRAFE_ADJ1;
//                        }
//                        //currentState = State.STRAFE_ADJ;
//                    }
//                    break;
                case CONES1:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        waitTimer.reset();
                        currentState = State.LIFT1;

                    }
                    break;
                case LIFT1:
                    if (waitTimer.seconds()>=waitTime2) {
                        LiftTarget = MID;
                        waitTimer.reset();

                        upTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB1;
                    }
                    break;
                case GRAB1:
                    if (waitTimer.seconds()>=waitTime3) {
                        drive.followTrajectoryAsync(DELIVER1);
                        currentState = State.DELIVER1;

                        waitTimer.reset();
                    }
                    break;
                case DELIVER1:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP1;
                    }
                    break;
                case DROP1:
                    if (waitTimer.seconds()>=waitTime1) {
                        drive.followTrajectoryAsync(CONES2);
                        LiftTarget = 125;//cone 4
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTime1 = 0.5;
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES2;
                    }
                    break;
//                case CONES2:
//
//                    break;
                case CONES2:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        waitTimer.reset();
                        currentState = State.LIFT2;

                    }
                    break;
                case LIFT2:
                    if (waitTimer.seconds()>=waitTime2) {
                        LiftTarget = MID;
                        waitTimer.reset();

                        upTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB2;
                    }
                    break;
                case GRAB2:
                    if (waitTimer.seconds()>=waitTime3) {
                        drive.followTrajectoryAsync(DELIVER2);
                        currentState = State.DELIVER2;

                        waitTimer.reset();
                    }
                    break;
                case DELIVER2:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP2;
                    }
                    break;
                case DROP2:
                    if (waitTimer.seconds() >= waitTime1) {
                        drive.followTrajectoryAsync(CONES3);
                        LiftTarget = 80;//cone 3
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES3;
                    }
                    break;
                case CONES3:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        waitTimer.reset();
                        currentState = State.LIFT3;

                    }
                    break;
                case LIFT3:
                    if (waitTimer.seconds()>=waitTime2) {
                        LiftTarget = MID;
                        waitTimer.reset();

                        upTimer.reset();
                        upTime1 = .7;
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB3;
                    }
                    break;
                case GRAB3:
                    if (waitTimer.seconds()>=waitTime3) {

                        drive.followTrajectoryAsync(DELIVER3);
                        currentState = State.DELIVER3;

                        waitTimer.reset();

                    }
                    break;
                case DELIVER3:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP3;
                        // upTime1 = 1.0;
                    }
                    break;
                case DROP3:
                    if (waitTimer.seconds() >= waitTime1) {
                        drive.followTrajectoryAsync(CONES4);
                        LiftTarget = 35;//cone 3
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES4;
                    }
                    break;
                case CONES4:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        waitTimer.reset();
                        currentState = State.LIFT4;

                    }
                    break;
                case LIFT4:
                    if (waitTimer.seconds()>=waitTime2) {
                        LiftTarget = MID;
                        waitTimer.reset();

                        upTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB4;
                    }
                    break;
                case GRAB4:
                    if (waitTimer.seconds()>=waitTime3) {
                        drive.followTrajectoryAsync(DELIVER4);
                        currentState = State.DELIVER4;

                        waitTimer.reset();
                    }
                case DELIVER4:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP4;

                    }
                    break;
                case DROP4:
                    if (waitTimer.seconds()>=waitTime4) {

                        if (!drive.isBusy()) {
                            currentState = State.PARK;
                            if(tagOfInterest == null)
                            {
                                /*
                                 * Insert your autonomous code here, presumably running some default configuration
                                 * since the tag was never sighted during INIT
                                 */
                                robot.twist.setPosition(startPos);
                                drive.followTrajectoryAsync(PARK_B);
                            }
                            else //TODO:auto code
                            {
                                /*
                                 * Insert your autonomous code here, probably using the tag pose to decide your configuration.
                                 */
                                //drive.followTrajectoryAsync(PARK_B);

                                if(tagOfInterest.id == 0)
                                {
                                    // do something

                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectorySequenceAsync(PARK_A);
                                }
                                else if(tagOfInterest.id == 1)
                                {
                                    // do something else
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectoryAsync(PARK_B);
                                }
                                else if(tagOfInterest.id == 2)
                                {
                                    // do something else
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectoryAsync(PARK_C);
                                }
                                robot.lelbow.setPosition(0);
                                robot.relbow.setPosition(0);
                                LiftTarget = 0;
                            }
                            // Start the wait timer once we switch to the next state
                            // This is so we can track how long we've been in the WAIT_1 state
                        }
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            /* TO USE IN PROGRAM
             * set state to START in other switch case
             * AT THE SAME TIME
             *  set claw to close
             *  set twist to startPos
             *  reset up timer
             * */
            switch (elbowUp) {
                case START:
                    if (upTimer.seconds() >= upTime1) {
                        robot.relbow.setPosition(.55);
                        robot.lelbow.setPosition(.55);
                        upTimer.reset();
                        elbowUp = elbowUpState.GRAB;
                    }
                    break;
                case GRAB:
                    if(upTimer.seconds() >= upTime) {
                        robot.twist.setPosition(flipPos);
                        elbowUp = elbowUpState.LIFT_TWIST;
                    }
                    break;
                case LIFT_TWIST:
                    if (Math.abs(robot.twist.getPosition() - flipPos) <.005 ) {
                        elbowUp = elbowUpState.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }

            /* TO USE IN PROGRAM
             * set state to START in other switch case
             * AT THE SAME TIME
             *  set claw to close
             *  set twist to startPos
             *  reset down timer
             * */
            switch (elbowDown) {
                case START:
                    if(downTimer.seconds() >= downTime1) {
                        robot.relbow.setPosition(0.03);
                        robot.lelbow.setPosition(0.03);
                        downTimer.reset();
                        elbowDown = elbowDownState.CLOSE;                    }
                    break;
                case CLOSE:
                    if (downTimer.seconds() >= downTime) {
                        robot.grab.setPosition(OPEN);
                        elbowDown = elbowDownState.LIFT_TWIST;
                    }
                    break;
                case LIFT_TWIST:
                    if (Math.abs(robot.grab.getPosition() - OPEN) <.005) {
                        elbowDown = elbowDownState.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }


            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // colorS.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

   tring.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
}

// Assume we have a hardware class called lift
// Lift uses a PID controller to maintain its height
// Thus, update() must be called in a loop
//    class colorS {
//        public colorS(HardwareMap hardwareMap){
//
//            L_color = hardwareMap.get(ColorSensor.class, "Left");
//            C_color = hardwareMap.get(ColorSensor.class, "Center");
//            R_color = hardwareMap.get(ColorSensor.class,"Right");
//        }
//        public void update() {
//
//            int red_blue_L = (L_color.red())/(L_color.blue());
//            int red_blue_C = (C_color.red())/(C_color.blue());
//            int red_blue_R = (R_color.red())/(R_color.blue());
//
//            int blue_red_L = (L_color.blue())/(L_color.red());
//            int blue_red_C = (C_color.blue())/(C_color.red());
//            int blue_red_R = (R_color.blue())/(R_color.red());
//
//
//            if(red_blue_L<=1 && blue_red_L<=1) {
//                telemetry.addData("color from left", "gray");
//                left = gray;
//            } else if (blue_red_L>=2){
//                telemetry.addData("color from left", "blue");
//                left = notGray;
//            } else if (red_blue_L>=2 && blue_red_L<=1) {
//                telemetry.addData("color from left", "red");
//                left = notGray;
//            }
//
//            if(red_blue_C<=1 && blue_red_C<=1) {
//                telemetry.addData("color from center", "gray");
//                center = gray;
//            } else if (blue_red_C>=2){
//                telemetry.addData("color from center", "blue");
//                center = notGray;
//            } else if (red_blue_C>=2 && blue_red_C<=1) {
//                telemetry.addData("color from center", "red");
//                center = notGray;
//            }
//
//            if(red_blue_R<=1 && blue_red_R<=1) {
//                telemetry.addData("color from right", "gray");
//                right = gray;
//            } else if (blue_red_R>=2){
//                telemetry.addData("color from right", "blue");
//                right = notGray;
//            } else if (red_blue_R>=2 && blue_red_R<=1) {
//                telemetry.addData("color from right", "red");
//                right = notGray;
//            }
//        }
//    }

class Lift {
    public Lift(HardwareMap hardwareMap) {
        // Beep boop this is the the constructor for the lift
        // Assume this sets up the lift hardware

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        larm = hardwareMap.get(DcMotorEx.class,"Llift");
        rarm = hardwareMap.get(DcMotorEx.class,"Rlift");


        larm.setDirection(DcMotorEx.Direction.REVERSE);
        rarm.setDirection(DcMotorEx.Direction.FORWARD);

        larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        // Beep boop this is the lift update function
        // Assume this runs some PID controller for the lift

        controller.setPID(p, i, d);

        int larmPos = larm.getCurrentPosition();
        int rarmPos = rarm.getCurrentPosition();

        double Lpid = controller.calculate(larmPos, LiftTarget);
        double Rpid = controller.calculate(rarmPos, LiftTarget);

        double Lpower = Lpid + f;
        double Rpower = Rpid + f;

        larm.setPower(Lpower);
        rarm.setPower(Rpower);

        telemetry.addData("Lpos", larmPos);
        telemetry.addData("Rpos", rarmPos);
        telemetry.addData("Ltarget", LiftTarget);
        telemetry.addData("Rtarget", LiftTarget);
        telemetry.update();
    }
}
}
// hi there :]