package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robothardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@Config
@Autonomous
public class PIDTuning extends LinearOpMode {

    public static int POZITIE = 0;
    public static double Kp=0, Ki=0, Kd=0;
    double lastKp,lastKi,lastKd;
    robothardware robot = new robothardware(this);
    PIDController controller = new PIDController(Kp,Ki,Kd);
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        robot.init();
        double power = 0;

//        robot.pendulare.setPosition(0.3);


        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            if(lastKp != Kp || lastKd != Kd || lastKi != Ki){
            controller.setK(Kp,Ki,Kd);
                }

//            robot.lift.target= POZITIE;
//            robot.lift.update();
            lastKp = Kp;
            lastKi = Ki;
            lastKd = Kd;
            power = controller.update(POZITIE,robot.pend1.getPosition());
            

            dashboardTelemetry.addData("Target Pos",POZITIE);
            dashboardTelemetry.addData("Current Pos",robot.backLeft.getCurrentPosition());
            dashboardTelemetry.addData("Motord Power", robot.backLeft.getPower());
            dashboardTelemetry.addData("Power PID", power);
            dashboardTelemetry.addData("Timer",controller.timer.seconds());
//            dashboardTelemetry.addData("K",controller.);
            dashboardTelemetry.update();

        }

    }
}
