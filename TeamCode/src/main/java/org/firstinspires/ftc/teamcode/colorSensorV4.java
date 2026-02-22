package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Robot: colorSensorV4", group = "Robot")


public class colorSensorV4 extends LinearOpMode {
    public DcMotor mLM, mFW, mI;
    public Servo sF;
    public ColorSensor color;
    public TouchSensor touch;

    @Override
    public void runOpMode() {
        double counterI = 1;
        double outFlicker = -1.0;
        double inFlicker = 1.0;

        String slot1Color = "None";
        String slot2Color = "None";
        String slot3Color = "None";
        String color1 = "Green";
        String color2 = "Purple";
        String color3 = "Green";
        int artifactCounter = 0;
        int shootPhase = 0;
        boolean touchWasPressed = false;
        boolean dpadWasPressed = false;
        boolean shootPositioned = false;
        boolean readyToShoot = false;

        int orientation = 0;
        int stepSize = 180;
        int currentTarget = 0; // running encoder target, never resets during a cycle

        float[] hsvValues = {0F, 0F, 0F};
        final double scale_factor = 255;

        mLM = hardwareMap.get(DcMotor.class, "mLM");
        mFW = hardwareMap.get(DcMotor.class, "mFW");
        mI = hardwareMap.get(DcMotor.class, "mI");
        sF = hardwareMap.get(Servo.class, "sF");
        color = hardwareMap.get(ColorSensor.class, "color");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        mLM.setDirection(DcMotor.Direction.REVERSE);
        mFW.setDirection(DcMotor.Direction.FORWARD);
        sF.setDirection(Servo.Direction.REVERSE);

        mLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLM.setTargetPosition(0);
        mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLM.setPower(0.0);
        mLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sF.setPosition(outFlicker);

        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

// ========== INTAKE MOTOR (gamepad2 Y toggle) ==========
            if (gamepad2.yWasPressed()) {
                counterI += 1;
            }
            if (counterI % 2 == 0) {
                mI.setPower(1.0);
            } else {
                mI.setPower(0.0);
            }

// ========== INTAKE TRIGGER (touch sensor OR dpad_up) ==========
            boolean intakeTriggered = false;

            if (touch.isPressed() && !touchWasPressed && !readyToShoot && artifactCounter < 3) {
                intakeTriggered = true;
            }
            touchWasPressed = touch.isPressed();

            if (gamepad1.dpad_up && !dpadWasPressed && !readyToShoot && artifactCounter < 3) {
                intakeTriggered = true;
            }
            dpadWasPressed = gamepad1.dpad_up;

            if (intakeTriggered) {
                sleep(800);

                // Move forward 1 step using running target
                currentTarget += stepSize;
                mLM.setTargetPosition(currentTarget);
                mLM.setPower(0.5);
                while (mLM.isBusy() && opModeIsActive()) {
                    idle();
                }
                mLM.setPower(0);
                sleep(200);

                // Read color — same method that worked in old code
                Color.RGBToHSV(
                        (int) (color.red() * scale_factor),
                        (int) (color.green() * scale_factor),
                        (int) (color.blue() * scale_factor),
                        hsvValues);
                float hue = hsvValues[0];

                String detected = "None";
                if (hue >= 100 && hue <= 180) {
                    detected = "Green";
                } else if (hue >= 200 && hue <= 270) {
                    detected = "Purple";
                }

                // Retry if nothing detected
                int retries = 0;
                while ("None".equals(detected) && retries < 10 && opModeIsActive()) {
                    sleep(100);
                    Color.RGBToHSV(
                            (int) (color.red() * scale_factor),
                            (int) (color.green() * scale_factor),
                            (int) (color.blue() * scale_factor),
                            hsvValues);
                    hue = hsvValues[0];
                    if (hue >= 100 && hue <= 180) {
                        detected = "Green";
                    } else if (hue >= 200 && hue <= 270) {
                        detected = "Purple";
                    }
                    retries++;
                }

                if (!"None".equals(detected)) {
                    if (artifactCounter == 0) {
                        slot1Color = detected;
                    } else if (artifactCounter == 1) {
                        slot2Color = detected;
                    } else if (artifactCounter == 2) {
                        slot3Color = detected;
                    }
                    artifactCounter++;
                }

                orientation = (orientation + 1) % 3;

                if (artifactCounter == 3) {
                    readyToShoot = true;
                    shootPhase = 1;
                    shootPositioned = false;
                }
            }

// ========== READY TO SHOOT (manual, for less than 3 balls) ==========
            if (gamepad1.aWasPressed() && artifactCounter > 0 && shootPhase == 0) {
                readyToShoot = true;
                shootPhase = 1;
                shootPositioned = false;
            }

// ========== SHOOTING LOGIC ==========
            if (readyToShoot && shootPhase > 0 && !shootPositioned) {
                String targetColor = "None";
                if (shootPhase == 1) {
                    targetColor = color1;
                } else if (shootPhase == 2) {
                    targetColor = color2;
                } else if (shootPhase == 3) {
                    targetColor = color3;
                }

                int stepsForSlot1 = 0;
                int stepsForSlot2 = 0;
                int stepsForSlot3 = 0;

                if (orientation == 0) {
                    stepsForSlot1 = 2;
                    stepsForSlot2 = 0;
                    stepsForSlot3 = 1;
                } else if (orientation == 1) {
                    stepsForSlot1 = 1;
                    stepsForSlot2 = 2;
                    stepsForSlot3 = 0;
                } else if (orientation == 2) {
                    stepsForSlot1 = 0;
                    stepsForSlot2 = 1;
                    stepsForSlot3 = 2;
                }

                int bestSteps = 99;
                int bestSlot = -1;

                if (targetColor.equals(slot1Color) && stepsForSlot1 < bestSteps) {
                    bestSteps = stepsForSlot1;
                    bestSlot = 1;
                }
                if (targetColor.equals(slot2Color) && stepsForSlot2 < bestSteps) {
                    bestSteps = stepsForSlot2;
                    bestSlot = 2;
                }
                if (targetColor.equals(slot3Color) && stepsForSlot3 < bestSteps) {
                    bestSteps = stepsForSlot3;
                    bestSlot = 3;
                }

                if (bestSlot != -1) {
                    if (bestSlot == 1) {
                        slot1Color = "None";
                    } else if (bestSlot == 2) {
                        slot2Color = "None";
                    } else if (bestSlot == 3) {
                        slot3Color = "None";
                    }

                    if (bestSteps > 0) {
                        currentTarget += bestSteps * stepSize;
                        mLM.setTargetPosition(currentTarget);
                        mLM.setPower(0.5);
                        while (mLM.isBusy() && opModeIsActive()) {
                            idle();
                        }
                        mLM.setPower(0);
                        sleep(100);
                        orientation = (orientation + bestSteps) % 3;
                    }

                    shootPositioned = true;
                } else {
                    if (shootPhase < 3) {
                        shootPhase++;
                    } else {
                        shootPhase = 0;
                        artifactCounter = 0;
                        readyToShoot = false;
                        shootPositioned = false;
                    }
                }
            }

// ========== FIRE (gamepad2 right bumper) ==========
            if (shootPositioned && gamepad2.rightBumperWasPressed()) {
                mFW.setPower(1.0);
                sleep(500);

                sF.setPosition(inFlicker);
                sleep(300);
                sF.setPosition(outFlicker);
                sleep(300);

                shootPositioned = false;

                if (shootPhase < 3) {
                    shootPhase++;
                } else {
                    shootPhase = 0;
                    artifactCounter = 0;
                    readyToShoot = false;
                    mFW.setPower(0.0);
                }
            }

// ========== MANUAL: Flywheel toggle (gamepad2 X) ==========
            if (gamepad2.xWasPressed()) {
                if (mFW.getPower() > 0) {
                    mFW.setPower(0.0);
                } else {
                    mFW.setPower(1.0);
                }
            }

// ========== MANUAL: Flicker (gamepad2 left bumper) ==========
            if (gamepad2.leftBumperWasPressed()) {
                sF.setPosition(inFlicker);
                sleep(300);
                sF.setPosition(outFlicker);
            }

// ========== MANUAL: Nudge mLM (gamepad1 bumpers) ==========
            if (gamepad1.leftBumperWasPressed()) {
                currentTarget += 50;
                mLM.setTargetPosition(currentTarget);
                mLM.setPower(0.5);
            }
            if (gamepad1.rightBumperWasPressed()) {
                currentTarget -= 50;
                mLM.setTargetPosition(currentTarget);
                mLM.setPower(0.5);
            }

// ========== MANUAL: Full reset (gamepad1 B) ==========
            if (gamepad1.bWasPressed()) {
                shootPhase = 0;
                artifactCounter = 0;
                readyToShoot = false;
                shootPositioned = false;
                slot1Color = "None";
                slot2Color = "None";
                slot3Color = "None";
                orientation = 0;
                currentTarget = 0;
                mFW.setPower(0.0);
                mLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mLM.setTargetPosition(0);
                mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mLM.setPower(0.0);
            }

// ========== TELEMETRY ==========
            Color.RGBToHSV(
                    (int) (color.red() * scale_factor),
                    (int) (color.green() * scale_factor),
                    (int) (color.blue() * scale_factor),
                    hsvValues);

            telemetry.addData("--- STATE ---", "");
            telemetry.addData("artifacts", artifactCounter);
            telemetry.addData("orientation", orientation);
            telemetry.addData("shoot phase", shootPhase);
            telemetry.addData("positioned", shootPositioned);
            telemetry.addData("ready", readyToShoot);
            telemetry.addData("--- SLOTS ---", "");
            telemetry.addData("slot1", slot1Color);
            telemetry.addData("slot2", slot2Color);
            telemetry.addData("slot3", slot3Color);
            telemetry.addData("pattern", color1 + " " + color2 + " " + color3);
            telemetry.addData("--- MOTOR ---", "");
            telemetry.addData("mLM pos", mLM.getCurrentPosition());
            telemetry.addData("mLM target", currentTarget);
            telemetry.addData("--- SENSOR ---", "");
            telemetry.addData("hue", hsvValues[0]);
            telemetry.addData("R", color.red());
            telemetry.addData("G", color.green());
            telemetry.addData("B", color.blue());
            telemetry.addData("touch", touch.isPressed());
            telemetry.update();
        }
    }
}