package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Recommended reading: https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
// I'm illiterate

@TeleOp(name = "Competition Telee", group = "")
public class CompetitionTelee288 extends LinearOpMode {
    //private DigitalChannel limitSwitch;




    final double JOYSTICK_DEAD_ZONE = 0.20;
    final double JOYSTICK_MOVEMENT_SENSITIVITY = 0.75;
    final double JOYSTICK_ROTATION_SENSITIVITY = 0.5;


    IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


        imu = hardwareMap.get(IMU.class, "imu");
        YawPitchRollAngles imuOutput;
        final VoltageSensor voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        double failureVoltage = 0;
        int failureCt = 0;

        SwerveDriveWheel LFWheel = new SwerveDriveWheel(
                telemetry,
                "LF",
                hardwareMap.dcMotor.get("LFDrive"),
                hardwareMap.crservo.get("LFSteer"),
                hardwareMap.analogInput.get("LFsteer")
        ),
        LRWheel = new SwerveDriveWheel(
                telemetry,
                "LR",
                hardwareMap.dcMotor.get("LRDrive"),
                hardwareMap.crservo.get("LRSteer"),
                hardwareMap.analogInput.get("LRsteer")
        ),
        RFWheel = new SwerveDriveWheel(
                telemetry,
                "RF",
                hardwareMap.dcMotor.get("RFDrive"),
                hardwareMap.crservo.get("RFSteer"),
                hardwareMap.analogInput.get("RFsteer")
        ),
        RRWheel = new SwerveDriveWheel(
                telemetry,
                "RR",
                hardwareMap.dcMotor.get("RRDrive"),
                hardwareMap.crservo.get("RRSteer"),
                hardwareMap.analogInput.get("RRsteer")
        );
        SwerveDriveCoordinator SwerveDrive = new SwerveDriveCoordinator(telemetry, LFWheel, LRWheel, RFWheel, RRWheel);

        //driveController mechDrive = new driveController(telemetry, hardwareMap);
        scoringController robotScoring = new scoringController(telemetry, hardwareMap);

        // Put initialization blocks here.
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart(); // should this be just before while (opmodeactive) ?

        // IMU parameters and initialization
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(imuParams);


        // Previous iteration gamepad states for edge detection
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // Put run blocks here.

        //double offsetHeading = 0, oldHeading = 0;
        while (opModeIsActive()) {
            //boolean voltageBad = false && voltSensor.getVoltage() < 12; //TODO: adjust this limit
            // Update sampled gamepad states. Sampling the current gamepad state as 'currentGamepadN'
            // is necessary for reliable edge triggering on button presses.
            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (gamepad1.right_trigger > 0) {
                telemetry.addData("info","resetting imu orientation");
                imu.initialize(imuParams);
                imu.resetYaw();
                //offsetHeading = 0; oldHeading = 0;
            }

            double joystickMovementY = inputScaling(gamepad1.left_stick_y) * JOYSTICK_MOVEMENT_SENSITIVITY;  // Note: pushing stick forward gives negative value
            double joystickMovementX = inputScaling(-gamepad1.left_stick_x) * JOYSTICK_MOVEMENT_SENSITIVITY;
            double yaw = (inputScaling(-gamepad1.right_stick_x) * JOYSTICK_ROTATION_SENSITIVITY);

            //get robot orientation from imu

            imuOutput = imu.getRobotYawPitchRollAngles();
                    //offsetHeading + imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//pih: assume it's clockwise
            if (imuOutput.getAcquisitionTime() == 0) {
                failureVoltage = voltSensor.getVoltage();
                ++failureCt;
                imu.initialize(imuParams);
                //imu.resetYaw();
                //offsetHeading = oldHeading;
                while (imuOutput.getAcquisitionTime() == 0)
                    imuOutput = imu.getRobotYawPitchRollAngles();
            }
            double robotHeading = imuOutput.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", robotHeading);
            telemetry.addData("Failure Voltage", failureVoltage);
            telemetry.addData("Failure Count", failureCt);
            //else oldHeading = robotHeading;//TODO: test this fix

            //input movement values into vector translation in 2d theorem
            double theta = -robotHeading;
            double movementX = joystickMovementX * cos(toRadians(theta)) - joystickMovementY * sin(toRadians(theta));
            double movementY = joystickMovementX * sin(toRadians(theta)) + joystickMovementY * cos(toRadians(theta));

            if (gamepad1.left_trigger >= 0.001) {
                movementX = movementX * 0.45;
                movementY = movementY * 0.45;
                yaw = yaw * 0.45;
            }

            /*if(! voltageBad)*/ SwerveDrive.drive(movementX, movementY, yaw);
            //else SwerveDrive.drive(0, 0, 0);

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