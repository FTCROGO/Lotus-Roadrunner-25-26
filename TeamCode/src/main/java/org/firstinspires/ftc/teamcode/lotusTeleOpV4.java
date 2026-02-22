package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;


@TeleOp(name = "Robot: lotusTeleOpV4", group = "Robot")


public class lotusTeleOpV4 extends LinearOpMode {
    public DcMotor mFL, mFR, mBL, mBR, mI, mLM, mFW;
    public Servo sG, sF;
    public IMU imu;
    public Limelight3A limeLight;
    public Servo sLED;
    public PinpointLocalizer pinpoint;
    public ColorSensor color;

    @Override
    public void runOpMode() {
        double initZ, currentZ, differenceZ;
        double changeX, changeY, changeZ;
        double cosA, sinA;
        double xRotated, yRotated;
        double powerFL, powerFR, powerBL, powerBR, maxPower;
        double counterI = 1;
        double counterFW = 1;
        double counterLM = 1;
        double counterG = 1;
        double counterF = 1;
        double outGate = 0.55;
        double inGate = -1.0;
        double outFlicker = -1.0;
        double inFlicker = 1.0;
        int position1 = 1;
        int position2 = 2;
        int position3 = 3;


        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");
        mI = hardwareMap.get(DcMotor.class, "mI");
        mLM = hardwareMap.get(DcMotor.class, "mLM");
        mFW = hardwareMap.get(DcMotor.class, "mFW");

        sG = hardwareMap.get(Servo.class, "sG");
        sF = hardwareMap.get(Servo.class, "sF");

        imu = hardwareMap.get(IMU.class, "imu");

        limeLight = hardwareMap.get(Limelight3A.class, "LimeLight");
        sLED = hardwareMap.get(Servo.class, "sLED");
        color = hardwareMap.get(ColorSensor.class, "color");
        float[] hsvValues = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double scale_factor = 255;


        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        mI.setDirection(DcMotor.Direction.FORWARD);
        mLM.setDirection(DcMotor.Direction.REVERSE);
        mFW.setDirection(DcMotor.Direction.FORWARD);

        sG.setDirection(Servo.Direction.REVERSE);
        sF.setDirection(Servo.Direction.REVERSE);

        mLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        mLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sG.setPosition(outGate);
        sF.setPosition(outFlicker);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);


        imu.initialize(new IMU.Parameters(orientation));
        initZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        // Start with AprilTag pipeline (pipeline 0)
        limeLight.pipelineSwitch(0);
        limeLight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("mLM encoder", "%d", mLM.getCurrentPosition());
            telemetry.update();
// Drivetrain - gamepad 1: left stick y (drive), left stick x (strafe), right stick x (turn)
            currentZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            differenceZ = currentZ - initZ;

            changeX = gamepad1.left_stick_x;
            changeY = -gamepad1.left_stick_y;
            changeZ = gamepad1.right_stick_x;

            cosA = Math.cos(-differenceZ);
            sinA = Math.sin(-differenceZ);

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


// Flywheel - gamepad 2: y (start & stop)
            if (gamepad2.yWasPressed()) {
                counterFW += 1;
            }
            if (counterFW % 2 == 0) {
                mFW.setPower(1.0);
            }
            if (counterFW % 2 == 1) {
                mFW.setPower(0.0);
            }

// Intake - gamepad 1: right bumper (start & stop)
            if (gamepad1.rightBumperWasPressed()) {
                counterI += 1;
            }
            if (counterI % 2 == 0) {
                mI.setPower(0.8);
            }
            if (counterI % 2 == 1) {
                mI.setPower(0.0);
            }

// Lotus Motus - gamepad 2: left bumper (start & stop)
            if (gamepad1.leftBumperWasPressed()) {
                counterLM += 1;
            }
            if (counterLM % 2 == 0) {
                mLM.setPower(0.1);
            }
            if (counterLM % 2 == 1) {
                mLM.setPower(0.0);
            }

//            if (gamepad1.dpadUpWasPressed()) {
//                mLM.setTargetPosition(position1);
//            }
//            if (gamepad1.dpadRightWasPressed()) {
//                mLM.setTargetPosition(position2);
//            }
//            if (gamepad1.dpadDownWasPressed()) {
//                mLM.setTargetPosition(position3);
//            }

// Gate - gamepad 2: left bumper (start & stop)
            if (gamepad2.rightBumperWasPressed()) {
                counterG += 1;
            }
            if (counterG % 2 == 0) {
                sG.setPosition(inGate);
            }
            if (counterG % 2 == 1) {
                sG.setPosition(outGate);
            }

// Flicker - gamepad 2: left bumper (start & stop)
            if (gamepad2.leftBumperWasPressed()) {
                counterF += 1;
            }
            if (counterF % 2 == 0) {
                sF.setPosition(inFlicker);
                sF.setPosition(outFlicker);
            }
            if (counterF % 2 == 1) {
                sF.setPosition(outFlicker);
            }

// Color sensor
            Color.RGBToHSV((int) (color.red() * scale_factor),
                    (int) (color.green() * scale_factor),
                    (int) (color.blue() * scale_factor), hsvValues);

// Telemetry
        }
    }
}