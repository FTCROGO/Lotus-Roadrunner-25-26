package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Robot: lotusTeleOpV2", group="Robot")

public class lotusTeleOpV2 extends LinearOpMode {
    public DcMotor mFL = null;
    public DcMotor mFR = null;
    public DcMotor mBL = null;
    public DcMotor mBR = null;
    public DcMotor mFW = null;

    public CRServo sI = null;
    public CRServo sRW1 = null;
    public CRServo sRW2 = null;     //RW --> RW2, consider changing to non-CR
//    public Servo sH = null;

    public IMU imu;


    @Override
    public void runOpMode() {
        double initZ;
        double currentZ;
        double zDifference;

        double changeX;
        double changeY;
        double changeZ;

        double powerFL;
        double powerFR;
        double powerBL;
        double powerBR;
        double maxPower;

        double counterI = 1;
        double counterRW1 = 1;
        double counterRW2a = 1;
        double counterRW2x = 1;
        double counterFW = 1;


        mFL  = hardwareMap.get(DcMotor.class, "leftFront");
        mFR  = hardwareMap.get(DcMotor.class, "rightFront");
        mBL  = hardwareMap.get(DcMotor.class, "leftBack");
        mBR  = hardwareMap.get(DcMotor.class, "rightBack");
        mFW = hardwareMap.get(DcMotor.class, "mFW");

        sI =  hardwareMap.get(CRServo.class, "sI");
        sRW1  = hardwareMap.get(CRServo.class, "sRW1");
        sRW2  = hardwareMap.get(CRServo.class, "sRW2");
//        sH = hardwareMap.get(Servo.class, "sH");

        imu = hardwareMap.get(IMU.class, "imu");


        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        mFW.setDirection(DcMotor.Direction.FORWARD);

        sI.setDirection(CRServo.Direction.REVERSE);
        sRW1.setDirection(CRServo.Direction.REVERSE);
        sRW2.setDirection(CRServo.Direction.REVERSE);
//        sH.setDirection(Servo.Direction.FORWARD);


//        sH  = hardwareMap.get(Servo.class, "sH");
//        sH.setPosition(0.5);

// Note: Cannot set position for RW

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
        initZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); //angle facing goal

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();




        waitForStart();

        while(opModeIsActive()) {

// Drivetrain - gamepad 1: left stick y (drive), left stick x (strafe), right stick x (turn)
            currentZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            zDifference = currentZ - initZ;

            changeX = gamepad1.left_stick_x;
            changeY = -gamepad1.left_stick_y;
            changeZ = gamepad1.right_stick_x;

            powerFL = (changeX + changeY + changeZ);
            powerFR = (-changeX + changeY - changeZ);
            powerBL = (-changeX + changeY + changeZ);
            powerBR = (changeX + changeY - changeZ);


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
            if (gamepad2.yWasPressed())
            {
                counterFW +=1;
            }
            if (counterFW % 2 == 0)
            {
                mFW.setPower(1.0);
            }
            if (counterFW % 2 == 1)
            {
                mFW.setPower(0.0);
            }

// trigger flywheel w/ rw1 & counter for spinning previously

// Intake - gamepad 1: right bumper (start & stop)
            if (gamepad2.rightBumperWasPressed())
            {
                counterI +=1;
            }
            if (counterI % 2 == 0)
            {
                sI.setPower(1.0);
            }
            if (counterI % 2 == 1)
            {
                sI.setPower(0.0);
            }



// Ramp Wheel 1 - gamepad 2: left bumper (start & stop)
            if (gamepad1.rightBumperWasPressed())
            {
                counterRW1 +=1;
            }
            if (counterRW1 % 2 == 0)
            {
                sRW1.setPower(1.0);
            }
            if (counterRW1 % 2 == 1)
            {
                sRW1.setPower(0.0);
            }



// Ramp Wheel 2 - gamepad 2: a (start & stop)
            if (gamepad2.aWasPressed())
            {
                counterRW2a +=1;
            }
            if (counterRW2a % 2 == 0)
            {
                sRW2.setPower(1.0);
            }
            if (counterRW2a % 2 == 1  && counterRW2x % 2 == 1)
            {
                sRW2.setPower(0.0);
            }


            if (gamepad2.xWasPressed())
            {
                counterRW2x +=1;
            }
            if (counterRW2x % 2 == 0)
            {
                sRW2.setPower(-0.25);
            }
            if (counterRW2x % 2 == 1 && counterRW2a % 2 == 1)
            {
                sRW2.setPower(0.0);
            }



// Hood - gamepad 1: a (near position - hood back), b (far position - hood forward)
//            if (gamepad1.a)
//            {
////                sH.setPosition(0.5);
//            }
//            else if (gamepad1.b)
//            {
//                sH.setPosition(0.9);
//            }



// Telemetry
//            telemetry.addData("sRW2 Position", "%.2f", sRW2.getPosition());   --> needs to be non-CR
//            telemetry.addData("sH Position", "%.2f", sH.getPosition());
            telemetry.addData("rw1 power", "%.2f", sRW1.getPower());
            telemetry.addData("rw2 power", "%.2f", sRW2.getPower());
//            telemetry.addData("fw power", "%.")
            telemetry.update();

        }
    }
}
