package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

        // Initialize limelight
        limeLight = hardwareMap.get(Limelight3A.class, "LimeLight");
        sLED = hardwareMap.get(Servo.class, "sLED");


        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        mI.setDirection(DcMotor.Direction.FORWARD);
        mLM.setDirection(DcMotor.Direction.REVERSE);
        mFW.setDirection(DcMotor.Direction.FORWARD);

        sG.setDirection(Servo.Direction.REVERSE);
        sF.setDirection(Servo.Direction.REVERSE);

        sG.setPosition(0.4);
        sF.setPosition(-1);

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

// Gate - gamepad 2: left bumper (start & stop)
            if (gamepad2.rightBumperWasPressed()) {
                counterG += 1;
            }
            if (counterG % 2 == 0) {
                sG.setPosition(1.0);
            }
            if (counterG % 2 == 1) {
                sG.setPosition(-0.3);
            }

// Flicker - gamepad 2: left bumper (start & stop)
            if (gamepad2.leftBumperWasPressed()) {
                counterF += 1;
            }
            if (counterF % 2 == 0) {
                sF.setPosition(1.0);
            }
            if (counterF % 2 == 1)
                sF.setPosition(-1.0);
            }

//            LLResult result = LimeLight.getLatestResult();
//
//            if (result != null) {
//
//                // APRIL TAG DETECTION (for obelisk sides)
//                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//
//                double distance = 0;
//                if (!fiducials.isEmpty()) {
//                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                        int id = fiducial.getFiducialId();
//                        double tx = fiducial.getTargetXDegrees();
//                        double ty = fiducial.getTargetYDegrees();
//                        distance = result.getBotposeAvgDist();
//                        telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
//
//                        Object aprilTagID = null;
//                        telemetry.addData("AprilTag ID", (Object) null);
//                        telemetry.addData("AprilTag X", tx);
//                        telemetry.addData("AprilTag Y", ty);
//                    }
//                }
//
//                YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
//                telemetry.addData("angle", "%.2f", angle);

//                if (distance < 1.35 && distance > 0.58 && angle > x && angle < y
//                        || distance > 1.75 && distance < 1.93 && angle > x && angle < y) {
//
//                    led.setPosition(0.5);
//                } else {
//                    led.setPosition(0.2);
//                }


//                // COLOR DETECTION (for green/purple balls)
//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//
//                if (!colorResults.isEmpty()) {
//                    for (LLResultTypes.ColorResult color : colorResults) {
//                        double ballX = color.getTargetXDegrees();
//                        double ballY = color.getTargetYDegrees();
//                        double ballArea = color.getTargetArea();
//
//                        telemetry.addData("Ball X", ballX);
//                        telemetry.addData("Ball Y", ballY);
//                        telemetry.addData("Ball Area", ballArea);
//                    }
//                } else {
//                    telemetry.addData("Balls", "None detected");
//                }

            // Telemetry
//            telemetry.addData("controller left trigger", gamepad2.left_bumper);
//            telemetry.addData("controller right trigger", gamepad2.right_bumper);
//            telemetry.addData("counterG", "%.2f", counterG);
//            telemetry.addData("counterF", "%.2f", counterF);
//            telemetry.addData("sG position", "%.2f", sG.getPower());
//            telemetry.addData("sF", "%.2f", sF.getPower());
//            telemetry.update();
        }
    }
//        LimeLight.stop();
