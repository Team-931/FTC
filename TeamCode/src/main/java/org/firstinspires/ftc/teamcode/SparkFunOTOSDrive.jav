package org.firstinspires.ftc.teamcode;



import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the SparkFun OTOS sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by SparkFun
 * Unless otherwise noted, comments are from SparkFun
 */
public class SparkFunOTOSDrive extends MecanumDrive {
    public static class Params {
        // RR localizer note: These units are inches and radians.
        // Heading offset -1.5708 radians / -90 degrees from tuning
        // Position Offsets: (-2.1987, -4.091) (-1.8323, -4.2232) (-1.9464, -4.1571)
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.8894, -4.1902, Math.toRadians(180));
        public double linearScalar = 0.9985;
        public double angularScalar = 0.9958; // Average of 0.9974 and 0.9942 in two test runs
    }

    public static SparkFunOTOSDrive.Params PARAMS = new SparkFunOTOSDrive.Params();
    public SparkFunOTOSCorrected otos;
    private Pose2d lastOtosPose = pose;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("OTOS_PARAMS",PARAMS);
        otos = hardwareMap.get(SparkFunOTOSCorrected.class,"SparkFunOTOS Corrected");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(PARAMS.offset);
        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
        System.out.println(otos.setAngularScalar(PARAMS.angularScalar));

        otos.setPosition(RRPoseToOTOSPose(pose));

        System.out.println(otos.calibrateImu(255, true));
        System.out.println("OTOS calibration complete!");
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastOtosPose != pose) {

            otos.setPosition(RRPoseToOTOSPose(pose));
        }



        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        pose = OTOSPoseToRRPose(otosPose);
        lastOtosPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y),otosVel.h);
    }


}
