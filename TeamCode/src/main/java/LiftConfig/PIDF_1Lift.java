package LiftConfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
//@Disabled
@TeleOp (group = "tests") //ONE MOTOR LIFT
public class PIDF_1Lift extends OpMode {
    private PIDController controller;

    // p = increase if not reaching target position
    public static double p = 0.00, i = 0, d = 0.000; // d = dampener (dampens arm movement woah). ignore i
    public static double f = 0.0;  // prevents arm from falling from gravity

    public static int target = 0; // target position
    private final double ticks_in_degree = 700/180.0; //look this up later please :"" 537.6 / 360.0   700/ 180.0

    //dumb stupid motor
    private DcMotorEx LIFT;

    @Override
    public void init(){

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TODO: CHANGE THE DEVICE NAME
        LIFT = hardwareMap.get(DcMotorEx.class,"w");
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int motorPos = LIFT.getCurrentPosition();
        double pid = controller.calculate(motorPos, target);
       // double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + f;

        LIFT.setPower(power);

        telemetry.addData("pos", motorPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
