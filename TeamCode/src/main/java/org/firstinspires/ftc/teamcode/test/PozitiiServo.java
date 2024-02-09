package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robothardware;

@Config
@Autonomous
public class PozitiiServo extends LinearOpMode {
    robothardware robot = new robothardware(this);
    public static double pozitie_pendul = 0;
    public static double pozitie_aligner = 0;
    public static double pozitie_aveon = 0;
    public static double pozitie_usa = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        robot.init();
        robot.pendulare.setPosition(0);
        robot.aligner.setPosition(0);
        robot.plane.setPosition(0);
        robot.usa.setPosition(0);


        waitForStart();

        if(isStopRequested()) return;
        while (opModeIsActive()){
            robot.pendulare.setPosition(pozitie_pendul);
            robot.aligner.setPosition(pozitie_aligner);
            robot.plane.setPosition(pozitie_aveon);
            robot.usa.setPosition(pozitie_usa);
            dashboardTelemetry.addData("Pozitie Pendul",robot.pendulare.getPosition());
            dashboardTelemetry.addData("Pozitie Aligmer",robot.aligner.getPosition());
            dashboardTelemetry.addData("Pozitie Avion", robot.plane.getPosition());
            dashboardTelemetry.addData("Pozitie Usa",robot.usa.getPosition());
            dashboardTelemetry.update();
            if (gamepad1.a)
                robot.intec.setPower(1);
            else if (gamepad2.b)
                robot.intec.setPower(-1);
            else
                robot.intec.setPower(0);

        }
    }
}
