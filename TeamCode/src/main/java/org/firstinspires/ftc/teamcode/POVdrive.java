package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name = "Robot: POVdrive", group = "Robot")


public class POVdrive extends LinearOpMode {
    public DcMotor mFL = null;
    public DcMotor mFR = null;
    public DcMotor mBL = null;
    public DcMotor mBR = null;


    public IMU imu;


    @Override
    public void runOpMode() {
        double initZ;
        double currentZ;
        double zDifference;


        double changeX;
        double changeY;
        double changeZ;


        double cosA;
        double sinA;


        double xRotated;
        double yRotated;


        double powerFL;
        double powerFR;
        double powerBL;
        double powerBR;
        double maxPower;


        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");


        imu = hardwareMap.get(IMU.class, "imu");


        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);


        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);


        imu.initialize(new IMU.Parameters(orientation));
        initZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        // could also have one for angle facing goal


        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {


// Drivetrain - gamepad 1: left stick y (drive), left stick x (strafe), right stick x (turn)
            currentZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            zDifference = currentZ - initZ;


            changeX = gamepad1.left_stick_x;
            changeY = -gamepad1.left_stick_y;
            changeZ = gamepad1.right_stick_x;


            cosA = Math.cos(Math.toRadians(-zDifference));
            sinA = Math.sin(Math.toRadians(-zDifference));


            xRotated = (changeX * cosA) - (changeY * sinA);
            yRotated = (changeX * sinA) + (changeY * cosA);


            powerFL = yRotated + xRotated + changeZ;
            powerFR = yRotated - xRotated - changeZ;
            powerBL = yRotated - xRotated + changeZ;
            powerBR = yRotated + xRotated - changeZ;


            maxPower = Math.max(
                    Math.max(Math.abs(powerFL), Math.abs(powerFR)),
                    Math.max(Math.abs(powerBL), Math.abs(powerBR)));
            if (maxPower > 1.0) {
                powerFL /= maxPower;
                powerFR /= maxPower;
                powerBL /= maxPower;
                powerBR /= maxPower;
            }


            mFL.setPower(powerFL);
            mFR.setPower(powerFR);
            mBL.setPower(powerBL);
            mBR.setPower(powerBR);


// Telemetry


            telemetry.addData("zDifference", ".%2f", zDifference);
            telemetry.update();


        }
    }
}





