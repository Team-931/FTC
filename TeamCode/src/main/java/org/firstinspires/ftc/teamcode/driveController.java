package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class driveController {
    private Telemetry telemetry;

    private DcMotor leftFront, rightFront, leftRear, rightRear;

    int LFstartTicks, RFstartTicks, LRstartTicks, RRstartTicks;

    //static public boolean debug = false;

    public void resetTicks() {
        LFstartTicks = leftFront.getCurrentPosition();
        RFstartTicks = rightFront.getCurrentPosition();
        LRstartTicks = leftRear.getCurrentPosition();
        RRstartTicks = rightRear.getCurrentPosition();
    }
    public driveController(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        leftRear = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        rightRear = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetTicks();
    }
    public void drive(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower){
        leftFront.setPower(leftFrontPower);
        telemetry.addData("LF ticks", leftFront.getCurrentPosition() - LFstartTicks);
        //telemetry.addData("LF current", hardwareMap.get(DcMotorEx.class,"frontLeftDriveMotor").getCurrent(CurrentUnit.MILLIAMPS));
        rightFront.setPower(rightFrontPower);
        telemetry.addData("RF ticks", rightFront.getCurrentPosition() - RFstartTicks);
        //telemetry.addData("RF current", hardwareMap.get(DcMotorEx.class,"frontRightDriveMotor").getCurrent(CurrentUnit.MILLIAMPS));
        leftRear.setPower(leftRearPower);
        telemetry.addData("LR ticks", leftRear.getCurrentPosition() -LRstartTicks);
        //telemetry.addData("LR current", hardwareMap.get(DcMotorEx.class,"backLeftDriveMotor").getCurrent(CurrentUnit.MILLIAMPS));
        rightRear.setPower(rightRearPower);
        telemetry.addData("RR ticks", rightRear.getCurrentPosition() - RRstartTicks);
        //telemetry.addData("RR current", hardwareMap.get(DcMotorEx.class,"backRightDriveMotor").getCurrent(CurrentUnit.MILLIAMPS));
    }

}
