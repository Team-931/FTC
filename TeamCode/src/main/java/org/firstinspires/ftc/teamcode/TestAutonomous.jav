package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.VectorEnabledTintResources;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Vector;

@Config
@Autonomous(name = "Test Autonomous Mode")
public class TestAutonomous extends LinearOpMode {


    @Override
    public void runOpMode() {
        Vector2d startPos = new Vector2d(0, 0);
        Pose2d moveToSamplesStart = new Pose2d(0 , 30, Math.toRadians(90));
        Vector2d scorePos1 = new Vector2d(0, 32);
        //Vector2d moveToSample;
        Vector2d scorePos2 = new Vector2d(4, 24);
        Vector2d scorePos3 = new Vector2d(8, 24);

        // Headings in RoadRunner are angles counterclockwise from the +X direction.
        // Thus if we consider the X axis to be side-to-side and the +Y direction to
        // be forward, then forward is a heading of +90 degrees.
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(startPos, Math.toRadians(90)));
        autoScoringController scoringController = new autoScoringController(telemetry,  hardwareMap);

        class ElevatorScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.elevatorScore();
                return false;
                // TODO implement other scoringController methods by running drive function continuously and adding each method into it
            }
        }
        class intakeClawToggle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.intakeClawToggle();
                return false;
            }
        }
        class upperArmScoring implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.upperArmPosition(0.25);
                return false;
            }
        }
        class upperShoulderPresetBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.uppShoulderPresetBasket();
                return false;
            }
        }
        class upperShoulderPresetWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.upperShoulderPresetWall();
                return false;
            }
        }
        class upperClawToggle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.upperClawToggle();
                return false;
            }
        }
        class runScoringControls implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.drive(0,0,0,0,false);
                return true;
            }
        }
        class startingPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                scoringController.startingPos();
                return false;
            }
        }

        // DcMotor motor1 = hardwareMap.get(DcMotor.class,  "motor");

        // Scores first Specimen, then moves behind samples
        TrajectoryActionBuilder a1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.6)
                .afterTime(0, new ElevatorScore())
                .splineToConstantHeading(scorePos1, Math.toRadians(-90))
                .afterTime(0, new upperClawToggle())
                .afterTime(0, new upperShoulderPresetWall())
                .splineTo(new Vector2d(24,18), Math.toRadians(0))
                .splineTo(new Vector2d(41,32), Math.toRadians(90))
                .splineTo(new Vector2d(45,40), Math.toRadians(0))
                .splineTo(new Vector2d(46,38), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50,5), Math.toRadians(115))
                .splineToConstantHeading(new Vector2d(41, 3), Math.toRadians(180))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(41, 0), Math.toRadians(180)),

                // Moves two samples, one at a time, into the human player zone, then moves to grab 2nd specimen for scoring
                //.splineToConstantHeading(new Vector2d(45,5), Math.toRadians(115))
                /*.splineToConstantHeading(new Vector2d(48, 52), Math.toRadians(20))
                .splineToConstantHeading(new Vector2d(52,50), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(48,5), Math.toRadians(140)).*//*
                .splineToConstantHeading(new Vector2d(51, 50), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(54, 50), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54,5), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(45,5), Math.toRadians(-90))*/
                //.splineToConstantHeading(new Vector2d(45, 0), Math.toRadians(180)),
        // Grabs 2nd specimen for scoring, then scores it and releases it.
        a3 = a1.fresh()
                .afterTime(0.0, new upperClawToggle())
                .afterTime(0.3, new ElevatorScore())
                /*.strafeTo(new Vector2d(5, 10))
                .strafeToLinearHeading(new Vector2d(5,32), Math.toRadians(90))*/
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(8,10), Math.toRadians(90))
                .splineTo(new Vector2d(5,33), Math.toRadians(90))
                .afterTime(0, new upperClawToggle())
                .afterTime(0.3, new upperShoulderPresetWall()),
        // Moves to collect 3rd specimen, then grabs it and moves to scoring position, then scores and releases it.
        a4 = a3.fresh()
                /*.strafeTo(new Vector2d(5,25))
                .strafeTo(new Vector2d(45,5))*/
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(15,15), Math.toRadians(-45))
                .splineTo(new Vector2d(45,5), Math.toRadians(-90))
                .strafeTo(new Vector2d(45, -2))
                .afterTime(0, new upperClawToggle())
                .afterTime(0.3, new ElevatorScore())
               /* .strafeTo(new Vector2d(5, 10))
                .strafeTo(new Vector2d(6,35))*/
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(10,10), Math.toRadians(90))
                .splineTo(new Vector2d(2.5,35), Math.toRadians(90))
                .afterTime(0, new upperClawToggle())
                //.splineToConstantHeading(new Vector2d(5,29), Math.toRadians(90))//Temporary ! ! !, replaced by a6
                .afterTime(0, new startingPos()),
        // Moves to collect 4th specimen, then grabs it and moves to scoring position, then scores and releases it.
        a5 = a4.fresh()
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(15,15), Math.toRadians(-45))
                .splineTo(new Vector2d(45,10), Math.toRadians(-90))
                .splineTo(new Vector2d(45, -2), Math.toRadians(-90))
                .afterTime(0, new upperClawToggle())
                .afterTime(0.3, new ElevatorScore())
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-2,10), Math.toRadians(180))
                .splineTo(new Vector2d(-8,35), Math.toRadians(90))
                .afterTime(0, new upperClawToggle())
                .afterTime(0.3, new upperShoulderPresetWall()),
        // TODO remove the collection and scoring of 5th specimen, a6 will just move from 4th specimen score to park
        a6 = a4.fresh()
                /*.strafeTo(new Vector2d(5,25))
                .strafeTo(new Vector2d(45,5))
                .strafeTo(new Vector2d(45, -2))
                .afterTime(0, new upperClawToggle())
                .afterTime(0.5, new ElevatorScore())
                .strafeTo(new Vector2d(5, 10))
                .strafeTo(new Vector2d(-5,35))
                .afterTime(0, new upperClawToggle())
                .afterTime(0.5, new upperShoulderPresetWall())
                .strafeTo(new Vector2d(5,25))
                 */
                .splineToConstantHeading(new Vector2d(5,10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50,5),Math.toRadians(0));

        Action scoreOne = new RaceAction(

                a1.build(),
                new runScoringControls()
                );

        /*Action retrieveSamples = new RaceAction(

                a2.build(),
                new runScoringControls()
                );

         */
        Action scoreTwo = new RaceAction(

                a3.build(),
                new runScoringControls()
                );
        Action scoreThree = new RaceAction(

                a4.build(),
                new runScoringControls()
                );
        Action scoreFour = new RaceAction(

                a5.build(),
                new runScoringControls()
                );
        Action scoreFivePark = new RaceAction(

                a6.build(),
                new runScoringControls()
                );


        waitForStart();
        Actions.runBlocking(new SequentialAction(
                scoreOne,//restored a2 after a1 looked O.K.
                scoreTwo,
                scoreThree,
                //scoreFour,
                scoreFivePark


                )

        );
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("r", drive.pose.heading.toDouble());
            telemetry.update();
        }
    }
}