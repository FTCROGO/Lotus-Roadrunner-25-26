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


@TeleOp(name = "Robot: colorSensorV2", group = "Robot")


public class colorSensorV2 extends LinearOpMode {
    public DcMotor mLM, mFW, mI;
    public Servo sF;
    public ColorSensor color;
    public TouchSensor touch;

    @Override
    public void runOpMode() {
        double counterFW = 1;
        double counterF = 1;
        double outFlicker = -1.0;
        double inFlicker = 1.0;
        double counterI = 1;

        String slot1Color = "None";
        String slot2Color = "None";
        String slot3Color = "None";
        String color1 = "Green";   // pattern color 1
        String color2 = "Purple";  // pattern color 2
        String color3 = "Green";   // pattern color 3 **BASED ON MOTIF & LIMELIGHT
        String shootColor = "None";
        int artifactCounter = 0;
        int colorDetected = 0;
        int shootPhase = 0;        // 0 = not shooting, 1 = shoot color1, 2 = shoot color2, 3 = shoot color3
        boolean needsColorRead = false;
        boolean touchWasPressed = false;

        // Encoder positions from slot 1 reference point
        int ref1 = -20;    // slot1=intake, slot2=shoot, slot3=store
        int ref2 = 160;  // slot1=store, slot2=intake, slot3=shoot
        int ref3 = 340;  // slot1=shoot, slot2=store, slot3=intake

        mLM = hardwareMap.get(DcMotor.class, "mLM");
        mFW = hardwareMap.get(DcMotor.class, "mFW");
        mI = hardwareMap.get(DcMotor.class, "mI");

        sF = hardwareMap.get(Servo.class, "sF");

        color = hardwareMap.get(ColorSensor.class, "color");
        float[] hsvValues = {0F, 0F, 0F};
        final double scale_factor = 255;

        touch = hardwareMap.get(TouchSensor.class, "touch");


        mLM.setDirection(DcMotor.Direction.REVERSE);
        mFW.setDirection(DcMotor.Direction.FORWARD);

        sF.setDirection(Servo.Direction.REVERSE);

        mLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLM.setTargetPosition(-20);
        mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLM.setPower(1.0); // need power for RUN_TO_POSITION to work
        mLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sF.setPosition(outFlicker);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {

// Intake - gamepad 1: right bumper (start & stop)
            if (gamepad2.yWasPressed()) {
                counterI += 1;
            }
            if (counterI % 2 == 0) {
                mI.setPower(1.0);
            }
            if (counterI % 2 == 1) {
                mI.setPower(0.0);
            }

// ========== FLYWHEEL ==========
            if (gamepad2.yWasPressed()) {
                counterFW += 1;
            }
            if (counterFW % 2 == 0) {
                mFW.setPower(1.0);
            }
            if (counterFW % 2 == 1) {
                mFW.setPower(0.0);
            }

// ========== FLICKER ==========
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

// ========== TOUCH SENSOR / INTAKE ==========
            // Touch sensor triggers rotation, but doesn't increment counter yet
            int rotationBase = 0;
            int fullRotation = ref3 - ref1 + (ref2 - ref1); // one full cycle

// ========== TOUCH SENSOR / INTAKE ==========
            if (touch.isPressed() && !touchWasPressed && artifactCounter < 3) {
                sleep(800);
                needsColorRead = true;

                if (artifactCounter == 0) {
                    mLM.setTargetPosition(rotationBase + ref2);
                } else if (artifactCounter == 1) {
                    mLM.setTargetPosition(rotationBase + ref3);
                } else if (artifactCounter == 2) {
                    mLM.setTargetPosition(rotationBase + ref3 + (ref2 - ref1));
                }
            }
            touchWasPressed = touch.isPressed();

// ========== COLOR SENSOR ==========
            if (needsColorRead && !mLM.isBusy()) {
                Color.RGBToHSV((int) (color.red() * scale_factor),
                        (int) (color.green() * scale_factor),
                        (int) (color.blue() * scale_factor), hsvValues);
                float hue = hsvValues[0];

                String detected = "None";
                if (hue >= 100 && hue <= 180) {
                    detected = "Green";
                } else if (hue >= 200 && hue <= 270) {
                    detected = "Purple";
                }

                // Only count the ball if we actually detected a color
                if (!"None".equals(detected)) {
                    if (artifactCounter == 0) {
                        slot1Color = detected;
                    } else if (artifactCounter == 1) {
                        slot2Color = detected;
                    } else if (artifactCounter == 2) {
                        slot3Color = detected;
                    }

                    artifactCounter++;

                    if (artifactCounter == 3) {
                        shootPhase = 1;
                    }
                    needsColorRead = false;
                }
                // If nothing detected, keep needsColorRead true
                // so it tries again next loop
            }

// ========== SHOOTING LOGIC ==========
            if (artifactCounter == 3 && shootPhase > 0 && "None".equals(shootColor)) {
                String targetColor = "None";
                if (shootPhase == 1) {
                    targetColor = color1;
                } else if (shootPhase == 2) {
                    targetColor = color2;
                } else if (shootPhase == 3) {
                    targetColor = color3;
                }

                // Use current position as base for shooting rotations
                int shootBase = mLM.getCurrentPosition();

                if (targetColor.equals(slot1Color)) {
                    mLM.setTargetPosition(shootBase + ref3);
                    shootColor = slot1Color;
                    slot1Color = "None";
                } else if (targetColor.equals(slot2Color)) {
                    mLM.setTargetPosition(shootBase + ref1);
                    shootColor = slot2Color;
                    slot2Color = "None";
                } else if (targetColor.equals(slot3Color)) {
                    mLM.setTargetPosition(shootBase + ref2);
                    shootColor = slot3Color;
                    slot3Color = "None";
                }
            }

// In the reset after all 3 shots:
            if (shootPhase >= 3) {
                shootPhase = 0;
                artifactCounter = 0;
                rotationBase = mLM.getCurrentPosition(); // new base for next cycle
                mFW.setPower(0.0);
            }

// ========== FIRE ==========
            // Driver presses button to fire when aligned
            if (!"None".equals(shootColor) && gamepad2.rightBumperWasPressed() && !mLM.isBusy()) {
                mFW.setPower(1.0);
                sleep(500); // let flywheel spin up
                sF.setPosition(inFlicker);
                sleep(200);
                sF.setPosition(outFlicker);
                shootColor = "None";

                // Move to next shoot phase
                if (shootPhase < 3) {
                    shootPhase++;
                } else {
                    // All 3 shot, reset everything
                    shootPhase = 0;
                    artifactCounter = 0;
                    mFW.setPower(0.0);
                }
            }

// ========== MANUAL OVERRIDE ==========
            // Nudge motor if something jams
            if (gamepad1.leftBumperWasPressed()) {
                mLM.setTargetPosition(mLM.getCurrentPosition() + 50);
            }
            if (gamepad1.rightBumperWasPressed()) {
                mLM.setTargetPosition(mLM.getCurrentPosition() - 50);
            }

// ========== TELEMETRY ==========
            telemetry.addData("artifact counter", artifactCounter);
            telemetry.addData("shoot phase", shootPhase);
            telemetry.addData("mLM position", mLM.getCurrentPosition());
            telemetry.addData("mLM target position", mLM.getTargetPosition());
            telemetry.addData("slot 1 color", slot1Color);
            telemetry.addData("slot 2 color", slot2Color);
            telemetry.addData("slot 3 color", slot3Color);
            telemetry.addData("shoot color", shootColor);
            telemetry.addData("touch", touch.isPressed());
            telemetry.update();
        }
    }
}