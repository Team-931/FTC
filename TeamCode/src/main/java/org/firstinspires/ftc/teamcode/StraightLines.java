package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "Straight line tests")
public class StraightLines extends LinearOpMode {
public static double fwdEnd = 24, rgtEnd = 0, angleEnd = 90, angleStart = 90;

    @Override
    public void runOpMode() {
        Vector2d startPos = new Vector2d(0, 0);
        Vector2d scorePos1 = new Vector2d(rgtEnd, fwdEnd);

        // Headings in RoadRunner are angles counterclockwise from the +X direction.
        // Thus if we consider the X axis to be side-to-side and the +Y direction to
        // be forward, then forward is a heading of +90 degrees.
        //SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(startPos, Math.toRadians(90)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPos, Math.toRadians(90)));


        // DcMotor motor1 = hardwareMap.get(DcMotor.class,  "motor");

        // Scores first Specimen, then moves behind samples
        TrajectoryActionBuilder a1 = drive.actionBuilder(drive.pose)
                .setTangent(angleStart)
                .splineToConstantHeading(scorePos1, Math.toRadians(angleEnd));
                //.splineToConstantHeading(scorePos1, Math.toRadians(-90));

        Action scoreOne =         a1.build();


        drive.updatePoseEstimate();
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("r", drive.pose.heading.toDouble());
        telemetry.update();
        waitForStart();
        Actions.runBlocking(
                scoreOne

        );
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("r", Math.toRadians(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}