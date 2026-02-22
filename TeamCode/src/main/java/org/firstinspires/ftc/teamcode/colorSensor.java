package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Objects;


@TeleOp(name = "Robot: colorSensor", group = "Robot")


public class colorSensor extends LinearOpMode {
    public DcMotor mLM, mFW;
    public Servo sF;
    public ColorSensor color;
    public TouchSensor touch;

    @Override
    public void runOpMode() {
        double counterFW = 1;
        double counterF = 1;
        double outFlicker = -1.0;
        double inFlicker = 1.0;

        String slot1Color = "None"; //color
        String slot2Color = "None"; //color
        String slot3Color = "None"; //color
        String color1 = "Green"; //pattern color 1
        String color2 = "Purple"; //pattern color 2
        String color3 = "Green"; //pattern color 3 **BASED ON MOTIF & LIMELIGHT
        String shootColor = "None";
        double artifactCounter = 0;
        int colorDetected = 0;
        int ref1 = 0;
        int ref2 = 180;
        int ref3 = 360;

        mLM = hardwareMap.get(DcMotor.class, "mLM");
        mFW = hardwareMap.get(DcMotor.class, "mFW");

        sF = hardwareMap.get(Servo.class, "sF");

        color = hardwareMap.get(ColorSensor.class, "color");
        float[] hsvValues = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double scale_factor = 255;

        touch = hardwareMap.get(TouchSensor.class, "touch");


        mLM.setDirection(DcMotor.Direction.REVERSE);
        mFW.setDirection(DcMotor.Direction.FORWARD);

        sF.setDirection(Servo.Direction.REVERSE);

//        mLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLM.setTargetPosition(0);
        mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sF.setPosition(outFlicker);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {

// Flywheel
            if (gamepad2.yWasPressed()) {
                counterFW += 1;
            }
            if (counterFW % 2 == 0) {
                mFW.setPower(1.0);
            }
            if (counterFW % 2 == 1) {
                mFW.setPower(0.0);
            }

// Flicker
            if (gamepad2.leftBumperWasPressed()) {
                counterF += 1;
            }
            if (counterF % 2 == 0) {
                sF.setPosition(inFlicker);
                sF.setPosition(outFlicker);
            }
            if (counterF % 2 == 1)
                sF.setPosition(outFlicker);

// Touch sensor
            if (touch.isPressed()) {
                artifactCounter += 1;
                sleep(100);
                mLM.setTargetPosition(ref2);
            }
            else if (touch.isPressed() && artifactCounter == 1) {
                artifactCounter += 1;
                sleep(100);
                mLM.setTargetPosition(ref3);
            }
            else if (touch.isPressed() && artifactCounter == 2) {
                artifactCounter += 1;
                sleep(100);
                mLM.setTargetPosition(ref1);
            }
            else if (touch.isPressed() && artifactCounter == 3) {
                artifactCounter = 0;
            }
            else if (artifactCounter > 3) {
                artifactCounter = 0;
            }



// Color Sensor
            Color.RGBToHSV((int) (color.red() * scale_factor),
                    (int) (color.green() * scale_factor),
                    (int) (color.blue() * scale_factor), hsvValues);


            if (Objects.equals(slot1Color, "None")) {
                colorDetected = color.argb();
                float[] hsv = new float[3];
                Color.colorToHSV(colorDetected, hsv);
                float hue = hsv[0];

                if (hue >= 100 && hue <= 180) {
                    slot1Color = "Green";
                } else if (hue >= 200 && hue <= 270) {
                    slot1Color = "Purple";
                }
            } else if (Objects.equals(slot2Color, "None")) {
                colorDetected = color.argb();
                float[] hsv = new float[3];
                Color.colorToHSV(colorDetected, hsv);
                float hue = hsv[0];

                if (hue >= 100 && hue <= 180) {
                    slot2Color = "Green";
                } else if (hue >= 200 && hue <= 270) {
                    slot2Color = "Purple";
                }
            } else if (Objects.equals(slot3Color, "None")) {
                colorDetected = color.argb();
                float[] hsv = new float[3];
                Color.colorToHSV(colorDetected, hsv);
                float hue = hsv[0];

                if (hue >= 100 && hue <= 180) {
                    slot3Color = "Green";
                } else if (hue >= 200 && hue <= 270) {
                    slot3Color = "Purple";
                }
            }


            if (color1.equals(slot1Color)) {
                mLM.setTargetPosition(ref3);
                shootColor = slot1Color;
            } else if (color1.equals(slot2Color)) {
                mLM.setTargetPosition(ref1);
                shootColor = slot2Color;
            } else if (color1.equals(slot3Color)) {
                mLM.setTargetPosition(ref2);
                shootColor = slot3Color;
            }
// need to reset slot colors

            if (color2.equals(slot1Color)) {
                mLM.setTargetPosition(ref3);
                shootColor = slot1Color;
                slot1Color = "None";
            } else if (color2.equals(slot2Color)) {
                mLM.setTargetPosition(ref1);
                shootColor = slot2Color;
                slot2Color = "None";
            } else if (color2.equals(slot3Color)) {
                mLM.setTargetPosition(ref2);
                shootColor = slot3Color;
                slot3Color = "None";
            }


            if (color3.equals(slot1Color)) {
                mLM.setTargetPosition(ref3);
                shootColor = slot1Color;
                slot1Color = "None";
            } else if (color3.equals(slot2Color)) {
                mLM.setTargetPosition(ref1);
                shootColor = slot2Color;
                slot2Color = "None";
            } else if (color3.equals(slot3Color)) {
                mLM.setTargetPosition(ref2);
                shootColor = slot3Color;
                slot3Color = "None";
            }


            if (!Objects.equals(shootColor, "None") && gamepad2.yWasPressed()) { // aim align?
                mFW.setPower(1.0); // change to be based on distance and target velocity
                sF.setPosition(inFlicker);
                sleep(200);
                sF.setPosition(outFlicker);
                shootColor = "None";
            }


// Telemetry
            telemetry.addData("artifact counter", artifactCounter);
            telemetry.addData("mLM position", mLM.getCurrentPosition());
            telemetry.addData("mLM target position", mLM.getTargetPosition());
            telemetry.addData("color detected", colorDetected);
            telemetry.addData("slot 1 color", slot1Color);
            telemetry.addData("slot 2 color", slot2Color);
            telemetry.addData("slot 3 color", slot3Color);
            telemetry.addData("shoot color", shootColor);
            telemetry.addData("touch", touch.isPressed());
            telemetry.update();
        }
    }
}
