package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Robot: colorSensorV3", group = "Robot")


public class colorSensorV3 extends LinearOpMode {
    public DcMotor mLM, mFW, mI;
    public Servo sF;
    public ColorSensor color;
    public TouchSensor touch;

    private float lastHue = 0;
    private int lastR = 0;
    private int lastG = 0;
    private int lastB = 0;

    private void stepForward(int steps, int stepSize) {
        if (steps <= 0) return;
        int target = steps * stepSize;
        mLM.setTargetPosition(target);
        mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLM.setPower(0.5);
        while (mLM.isBusy() && opModeIsActive()) {
            telemetry.addData("MOVING", "target=%d, current=%d", target, mLM.getCurrentPosition());
            telemetry.update();
            idle();
        }
        mLM.setPower(0);
        sleep(100);
        mLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLM.setTargetPosition(0);
        mLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLM.setPower(0.0);
    }

    private String readColor() {
        float[] hsv = {0F, 0F, 0F};
        lastR = color.red();
        lastG = color.green();
        lastB = color.blue();
        Color.RGBToHSV(
                (int) (lastR * 255),
                (int) (lastG * 255),
                (int) (lastB * 255),
                hsv);
        lastHue = hsv[0];

        if (lastHue >= 100 && lastHue <= 180) {
            return "Green";
        } else if (lastHue >= 200 && lastHue <= 310) {
            return "Purple";
        }
        return "None";
    }

    @Override
    public void runOpMode() {
        double counterFW = 1;
        double counterI = 1;
        double outFlicker = -1.0;
        double inFlicker = 1.0;

        String slot1Color = "None";
        String slot2Color = "None";
        String slot3Color = "None";
        String color1 = "Green";
        String color2 = "Purple";
        String color3 = "Green";
        String shootColor = "None";
        int artifactCounter = 0;
        int shootPhase = 0;
        boolean touchWasPressed = false;
        boolean shootPositioned = false;
        boolean readyToShoot = false;
        boolean dpadWasPressed = false;

        int stepSize = 165;
        int orientation = 0;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

// ========== INTAKE MOTOR ==========
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

// ========== FLICKER (manual) ==========
            if (gamepad2.leftBumperWasPressed()) {
                sF.setPosition(inFlicker);
                sleep(300);
                sF.setPosition(outFlicker);
            }

// ========== INTAKE TRIGGER (touch sensor OR manual dpad_up) ==========
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

                stepForward(1, stepSize);
                orientation = (orientation + 1) % 3;

                // Wait for sensor to stabilize after motor stops
                sleep(300);

                // Read color with retries
                String detected = readColor();
                int retries = 0;
                while ("None".equals(detected) && retries < 20 && opModeIsActive()) {
                    sleep(100);
                    detected = readColor();
                    retries++;

                    // Show what sensor sees during retries
                    telemetry.addData("READING COLOR", "retry %d", retries);
                    telemetry.addData("hue", lastHue);
                    telemetry.addData("R", lastR);
                    telemetry.addData("G", lastG);
                    telemetry.addData("B", lastB);
                    telemetry.addData("detected", detected);
                    telemetry.update();
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

                if (artifactCounter == 3) {
                    readyToShoot = true;
                    shootPhase = 1;
                    shootPositioned = false;
                }
            }

// ========== READY TO SHOOT (manual trigger for less than 3 balls) ==========
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
                    shootColor = targetColor;
                    if (bestSlot == 1) {
                        slot1Color = "None";
                    } else if (bestSlot == 2) {
                        slot2Color = "None";
                    } else if (bestSlot == 3) {
                        slot3Color = "None";
                    }

                    if (bestSteps > 0) {
                        stepForward(bestSteps, stepSize);
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
                        mFW.setPower(0.0);
                    }
                }
            }

// ========== FIRE ==========
            if (shootPositioned && !"None".equals(shootColor) && gamepad2.rightBumperWasPressed()) {
                mFW.setPower(1.0);
                sleep(500);

                sF.setPosition(inFlicker);
                sleep(300);
                sF.setPosition(outFlicker);
                sleep(300);

                shootColor = "None";
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

// ========== MANUAL OVERRIDE ==========
            if (gamepad1.leftBumperWasPressed()) {
                mLM.setTargetPosition(mLM.getCurrentPosition() + 50);
                mLM.setPower(0.5);
            }
            if (gamepad1.rightBumperWasPressed()) {
                mLM.setTargetPosition(mLM.getCurrentPosition() - 50);
                mLM.setPower(0.5);
            }

// ========== TELEMETRY ==========
            telemetry.addData("--- STATE ---", "");
            telemetry.addData("artifact counter", artifactCounter);
            telemetry.addData("orientation", orientation);
            telemetry.addData("shoot phase", shootPhase);
            telemetry.addData("shoot positioned", shootPositioned);
            telemetry.addData("ready to shoot", readyToShoot);
            telemetry.addData("--- MOTOR ---", "");
            telemetry.addData("mLM position", mLM.getCurrentPosition());
            telemetry.addData("mLM target", mLM.getTargetPosition());
            telemetry.addData("--- COLORS ---", "");
            telemetry.addData("slot 1", slot1Color);
            telemetry.addData("slot 2", slot2Color);
            telemetry.addData("slot 3", slot3Color);
            telemetry.addData("shoot color", shootColor);
            telemetry.addData("pattern", color1 + " " + color2 + " " + color3);
            telemetry.addData("--- SENSOR RAW ---", "");
            telemetry.addData("last hue", lastHue);
            telemetry.addData("R", color.red());
            telemetry.addData("G", color.green());
            telemetry.addData("B", color.blue());
            telemetry.addData("touch", touch.isPressed());
            telemetry.update();
        }
    }
}