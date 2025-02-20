package LiftConfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Disabled
@TeleOp (group = "tests") //2 MOTOR LIFT
public class PIDF_2Lift extends OpMode {
    public PIDController controller;

    //TODO: edit variables on dashboard lol

    // p = increase if not reaching target position
    public static double p = .0000, i = 0, d = 0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .00;  // prevents arm from falling from gravity, p = run to position accuracy

    public static int Ltarget = 0; // target position
    public static int Rtarget = 0;
    private final double ticks_in_degree = 700/180.0; //look this up later please :"" 537.6 / 360.0   700/ 180.0




    //names of lift motors
    private DcMotorEx L;
    private DcMotorEx R;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //todo: change device names idk

        L = hardwareMap.get(DcMotorEx.class,"Llift");
        R = hardwareMap.get(DcMotorEx.class,"Rlift");

        L.setDirection(DcMotorEx.Direction.REVERSE);
        R.setDirection(DcMotorEx.Direction.FORWARD);

    }

    @Override
    public void loop(){

        controller.setPID(p, i, d);

        int L_Pos = L.getCurrentPosition();
        int R_Pos = R.getCurrentPosition();

        double Lpid = controller.calculate(L_Pos, Ltarget);
        double Rpid = controller.calculate(R_Pos, Rtarget);

//        double Lff = Math.cos(Math.toRadians(Ltarget / ticks_in_degree)) * f;
        //      double Rff = Math.cos(Math.toRadians(Rtarget / ticks_in_degree)) * f;

        double Lpower = Lpid + f;
        double Rpower = Rpid + f;

        L.setPower(Lpower);
        R.setPower(Rpower);

        telemetry.addData("Lpos", L_Pos);
        telemetry.addData("Rpos", R_Pos);
        telemetry.addData("Ltarget", Ltarget);
        telemetry.addData("Rtarget", Rtarget);
        telemetry.update();
    }
}

//         __  __     ______     __         ______
///      \ \ _ \ \   /\  ___\   /\ \       /\  == \
//        \ \  __ \  \ \  __\   \ \ \____  \ \  _-/
//         \ \_\ \_\  \ \_____\  \ \_____\  \ \_\
//          \/_/\/_/   \/_____/   \/_____/   \/_/