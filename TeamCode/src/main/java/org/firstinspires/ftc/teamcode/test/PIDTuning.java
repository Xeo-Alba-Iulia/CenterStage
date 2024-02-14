package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robothardware;

@Config
@Autonomous
public class PIDTuning extends LinearOpMode {

    public static int POZITIE = 0;
    public static double Kp=0, Ki=0, Kd=0;
    double lastKp,lastKi,lastKd;
    robothardware robot = new robothardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        robot.init();
//        robot.pendulare.setPosition(0.3);


        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            if(lastKp != Kp || lastKd != Kd || lastKi != Ki){
//                robot.lift.controller = new PIDController(Kp,Ki,Kd);
            }
//            robot.lift.target= POZITIE;
//            robot.lift.update();
            lastKp = Kp;
            lastKi = Ki;
            lastKd = Kd;
            dashboardTelemetry.addData("Target Pos",POZITIE);
//            dashboardTelemetry.addData("Current Pos",robot.lift.getCurrentPosition());
//            dashboardTelemetry.addData("Motord Power", robot.lift.getPower());
//            dashboardTelemetry.addData("Timer",robot.lift.controller.timer.seconds());
            dashboardTelemetry.update();

        }

    }
}
