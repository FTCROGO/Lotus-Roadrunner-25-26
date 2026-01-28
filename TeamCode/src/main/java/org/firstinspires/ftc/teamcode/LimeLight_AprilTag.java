package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;
//this is for timer!


@TeleOp(name = "LimeLight_AprilTag", group = "Robot")


public class LimeLight_AprilTag extends LinearOpMode {
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
    boolean mFWOn = false;
    ElapsedTime flywheelTimer = new ElapsedTime();
    //this is also for timer

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

        double counterI = 1;
        double counterRW1 = 1;
        double counterRW2a = 1;
        double counterRW2x = 1;
        double counterFW = 1;

        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");

        DcMotorEx mFW;
        mFW = hardwareMap.get(DcMotorEx.class, "mFW");
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        sI = hardwareMap.get(CRServo.class, "sI");
        sRW1 = hardwareMap.get(CRServo.class, "sRW1");
        sRW2 = hardwareMap.get(CRServo.class, "sRW2");


        imu = hardwareMap.get(IMU.class, "imu");


        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);
        sI.setDirection(CRServo.Direction.REVERSE);
        sRW1.setDirection(CRServo.Direction.REVERSE);
        sRW2.setDirection(CRServo.Direction.REVERSE);


