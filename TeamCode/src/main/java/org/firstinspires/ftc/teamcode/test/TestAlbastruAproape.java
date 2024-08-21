package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.albastruAproape;
import org.firstinspires.ftc.teamcode.robothardware;

@Autonomous(group = "Test", name = "TestAproapeAlbastru")
public class TestAlbastruAproape extends albastruAproape {
    @Override
    public void init() {
        robot = new robothardware(this);
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.pendulare.setPosition(robot.pendul_intake);
        robot.aligner.setPosition(robot.aligner_intake);
        robot.usa.setPosition(robot.door.usa_intake);

        spikemark = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16, 35, Math.toRadians(270+40)))
                .build();
        spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                .back(5)
                .build();

        heading_align = drive.trajectoryBuilder(spikemarkspate.end())
                .lineToLinearHeading(new Pose2d(24,50,Math.toRadians(180)))
                .build();
        backdrop_align = drive.trajectoryBuilder(heading_align.end())
                .lineToLinearHeading(new Pose2d(46,35,Math.toRadians(180)))
                .build();
        backdrop_fata = drive.trajectoryBuilder(backdrop_align.end())
                .forward(15)
                .build();

        parcare_align = drive.trajectoryBuilder(backdrop_fata.end())
                .strafeRight(24)
                .build();
        parcare = drive.trajectoryBuilder(parcare_align.end())
                .back(2)
                .build();
    }

    @Override
    public void init_loop() {}
}
