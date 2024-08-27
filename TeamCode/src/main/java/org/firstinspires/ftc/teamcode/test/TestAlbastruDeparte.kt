@file:Suppress("SpellCheckingInspection")

package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.auto.albastruDeparte
import org.firstinspires.ftc.teamcode.robothardware
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.EnumMap

/**
 * Test pentru scenariul autonom [Albastru Departe][albastruDeparte]
 *
 * Testeaza traiectoriile (fara camera) pentru pornirea din spate in alianta albastra
 */
@TeleOp(name = "Test Albastru Departe", group = "Tests")
class TestAlbastruDeparte : albastruDeparte() {
    /**
     * Initializarea normala fara camere
     */
    override fun init() {
        robot = robothardware(this)
        robot.init()
        drive = org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose
        robot.pendulare.position = robot.pendul_intake
        robot.aligner.position = robot.aligner_intake
        robot.usa.position = robot.door.usa_intake

        stagedoor1 = drive.trajectoryBuilder(heading_align.end())
            .back(80.0)
            .build()
        stagedoor2 = drive.trajectoryBuilder(stagedoor1.end())
            .back(5.0)
            .build()
        backdrop_align = drive.trajectoryBuilder(stagedoor2.end())
            .lineToLinearHeading(
                Pose2d(
                    55.0,
                    51.0,
                    Math.toRadians(180.0)
                )
            )
            .build()
        backdrop_fata = drive.trajectoryBuilder(backdrop_align.end())
            .forward(15.0)
            .build()
        park_align = drive.trajectoryBuilder(backdrop_fata.end())
            .strafeLeft(45.0)
            .build()
        park = drive.trajectoryBuilder(park_align.end())
            .back(15.0)
            .build()
    }

    /**
     * Seteaza traiectoria din dpad controller
     */
    @Suppress("KDocUnresolvedReference")
    override fun init_loop() {
        posGameObj = when {
            gamepad1.dpad_left -> caseState.STANGA
            gamepad1.dpad_right -> caseState.DREAPTA
            gamepad1.dpad_up || gamepad1.dpad_down -> caseState.MIJLOC
            else -> posGameObj
        }
        telemetry.addData("Pozitie obiect de joc", posGameObj)
        /**
         * `trajectoryMap[posGameObj]` este intotdeuna NonNull
         *
         * Vezi [Operator !!](https://kotlinlang.org/docs/null-safety.html#the-operator)
         */
         val (spikemark, spikemarkspate, heading_align) = trajectoryMap[posGameObj]!!

        this.spikemark = spikemark as Trajectory
        this.spikemarkspate = spikemarkspate as Trajectory
        this.heading_align = heading_align as TrajectorySequence
    }

    private var posGameObj = caseState.STANGA

    /**
     * Map de traiectorii pentru pozitia elementului de joc
     *
     * Folosita in init_loop din test pentru a facilita testarea tuturor pozitiilor posibile
     *
     * Traiectoriile sunt identice cu cele din [albastruDeparte]
     */
    private val trajectoryMap = EnumMap(mapOf(
        caseState.STANGA to arrayOf(
            drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(-32.0, 30.0, Math.toRadians((360 - 30).toDouble())))
                .build(),
            drive.trajectoryBuilder(spikemark.end())
                .back(5.0)
                .build(),
            drive.trajectorySequenceBuilder(spikemarkspate.end())
                .lineToLinearHeading(Pose2d(-46.0, 10.0, Math.toRadians(180.0)))
                .build()
        ),
        caseState.MIJLOC to arrayOf(
            drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(-40.0, 35.0, Math.toRadians((270 - 40).toDouble())))
                .build(),
            drive.trajectoryBuilder(spikemark.end())
                .lineToLinearHeading(Pose2d(-34.0, 38.0, Math.toRadians(180.0)))
                .build(),
            drive.trajectorySequenceBuilder(spikemarkspate.end())
                .lineToLinearHeading(Pose2d(-35.0, 10.0, Math.toRadians(180.0)))
                .build()
        ),
        caseState.DREAPTA to arrayOf(
            drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(-33.0, 32.0, Math.toRadians((270 + 20).toDouble())))
                .build(),
            drive.trajectoryBuilder(spikemark.end())
                .back(5.0)
                .build(),
            drive.trajectorySequenceBuilder(spikemarkspate.end())
                .lineToLinearHeading(Pose2d(-45.0, 33.0, Math.toRadians(180.0)))
                .lineToLinearHeading(Pose2d(-45.0, 10.0, Math.toRadians(180.0)))
                .build()
        )
    ))
}