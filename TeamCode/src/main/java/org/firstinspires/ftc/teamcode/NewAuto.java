package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "New Auto")
public class NewAuto extends LinearOpMode {
    final Point start = new Point(0,0),
            control1 = new Point(12, 24),
            goal1 = new Point(36, 52),
            goal2 = new Point(0,24);
    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        follower.setPose(new Pose(0,0,0));
        PathChain path1 = follower.pathBuilder()
                .addBezierCurve(start, control1, goal1)
                .setTangentHeadingInterpolation()
                .build(),
                path2 = follower.pathBuilder()
                        .addBezierLine(start, goal2)
                        .setConstantHeadingInterpolation(0)
                        .build(),
                path3 = follower.pathBuilder()
                        .addPath(new BezierPoint(start))
                        .build()
        ;

        follower.followPath(path3,true);

        telemetry.addLine("Moving to specimen.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            follower.telemetryDebug(telemetry);
        }
    }
}
