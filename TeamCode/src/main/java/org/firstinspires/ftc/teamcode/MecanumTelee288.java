package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

// Recommended reading: https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
// I'm illiterate

@TeleOp(name = "Mecanum Telee", group = "")
public class MecanumTelee288 extends LinearOpMode {




    final double JOYSTICK_DEAD_ZONE = 0.20;
    final double JOYSTICK_MOVEMENT_SENSITIVITY = 0.75;
    final double JOYSTICK_ROTATION_SENSITIVITY = 1.00;

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);



    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        scoringController robotScoring = new scoringController(telemetry, hardwareMap);

        // Put initialization blocks here.
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // IMU parameters and initialization
        follower.startTeleopDrive();

        // Previous iteration gamepad states for edge detection
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // Put run blocks here.

        while (opModeIsActive()) {
            // Update sampled gamepad states. Sampling the current gamepad state as 'currentGamepadN'
            // is necessary for reliable edge triggering on button presses.
            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (gamepad1.right_trigger > 0) {
                telemetry.addData("info","not resetting imu orientation");
                //follower.setHeadingOffset(- follower.getTotalHeading());
            }

            double movementY = inputScaling(-gamepad1.left_stick_y) * JOYSTICK_MOVEMENT_SENSITIVITY;  // Note: pushing stick forward gives negative value
            double movementX = inputScaling(gamepad1.left_stick_x) * JOYSTICK_MOVEMENT_SENSITIVITY;
            double yaw = (inputScaling(gamepad1.right_stick_x) * JOYSTICK_ROTATION_SENSITIVITY) * 0.75;


            if (gamepad1.left_trigger > 0.001) {
                movementX = movementX * 0.45;
                movementY = movementY * 0.45;
                yaw = yaw * 0.45;
            }

            follower.setTeleOpMovementVectors(movementY, -movementX, -yaw, false);
            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));


            //Robot Scoring control
            double liftControl = -inputScaling(currentGamepad2.left_stick_y);
            double intakeSlideControl = inputScaling(-currentGamepad2.right_stick_y);
            double intakeShoulderControl = inputScaling(-currentGamepad2.right_stick_x);
            double intakeWristControl = inputScaling(currentGamepad2.right_trigger - currentGamepad2.left_trigger);
            if (currentGamepad2.b && !prevGamepad2.b) {
                robotScoring.intakeClawToggle();
            }
            if (currentGamepad2.a && !prevGamepad2.a) {
                robotScoring.upperShoulderPresetWall();
            }
            if (currentGamepad2.y && !prevGamepad2.y) {
                robotScoring.elevatorScore();
            }
            if (currentGamepad2.x && !prevGamepad2.x) {
                robotScoring.upperClawToggle();
            }
            if (currentGamepad1.x && !prevGamepad2.x) {
                robotScoring.elevatorScore();
            }
            if (currentGamepad2.dpad_down && !prevGamepad2.dpad_down) {
                robotScoring.pickupStageOne();
            }
            if (currentGamepad2.dpad_up && !prevGamepad2.dpad_up) {
                robotScoring.uppShoulderPresetBasket();
            }
            if (currentGamepad2.left_bumper && !prevGamepad2.left_bumper) {
                robotScoring.pickupPrepare();
            } else if (currentGamepad2.right_bumper && !prevGamepad2.right_bumper) {
                robotScoring.chickenPecker();
            }
            if (currentGamepad2.dpad_left && !prevGamepad2.dpad_left) {
                robotScoring.elbowRetract();
            }
            if (currentGamepad2.dpad_right && !prevGamepad2.dpad_right) {
                robotScoring.pickupStageTwo();
            }
            robotScoring.drive(liftControl, intakeSlideControl, intakeShoulderControl, intakeWristControl, gamepad1.a);
            /* Update Telemetry to the Driver Hub */
            telemetry.update();
        }
    }

    double inputScaling(double x) {
        double sign = Math.signum(x);
        double magnitude = Math.abs(x);
        if (magnitude < JOYSTICK_DEAD_ZONE) {
            magnitude = 0.0;
        } else {
            magnitude = (magnitude - JOYSTICK_DEAD_ZONE) / (1.0 - JOYSTICK_DEAD_ZONE);
        }
        magnitude = Math.pow(magnitude, 2.0);
        return sign * magnitude;
    }
}