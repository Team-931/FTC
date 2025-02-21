package org.firstinspires.ftc.teamcode.LimelightTesting;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp (name = "LimelightTest", group = "Sensor")
public class LimelightTest extends LinearOpMode {
    Limelight3A limelight;


    public LimelightTest() {
       //limelight = new Limelight3A(0, "limelight", ) //TODO collect params for LimeLight3A from Docs
    }
    public void colorChange(int input) {
        limelight.pipelineSwitch(input);
    }

    /*
    public Pose3D cameraPos() {
        List<LLResultTypes.ColorResult> list;
        list = limelight.getLatestResult().getColorResults()
    }

     */

    public double xPos() {
        return limelight.getLatestResult().getTx();
    }

    public double yPos() {
        return limelight.getLatestResult().getTy();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
        }

    }

}
