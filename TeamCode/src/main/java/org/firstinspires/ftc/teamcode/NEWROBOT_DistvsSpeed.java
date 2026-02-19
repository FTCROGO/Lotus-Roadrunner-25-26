package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "NEWROBOT_DistvsSpeed", group = "Robot")
public class NEWROBOT_DistvsSpeed extends LinearOpMode {

    // =========================
    // Hardware
    // =========================

    private DcMotor mFL, mFR, mBL, mBR, mI;
    private DcMotorEx mFW;
    //private CRServo sG, sF;
    private IMU imu;
    private Limelight3A limelight;
    //private Servo sLED;

    // =========================
    // Constants (tune)
    // =========================
    private static final double TICKS_PER_REV = 28;       // change to your motor spec!
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;

    // Aim assist tuning
    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_TURN = 0.35;

    // -------- Trig distance constants (inches + degrees) --------
    private static final double GOAL_TAG_CENTER_HEIGHT_IN = 29.49; // measured in the field
    private static final double LL_LENS_HEIGHT_IN = 13.5;          // measured on robot
    private static final double LL_MOUNT_ANGLE_DEG = 0;            // camera pitch up from horizontal
    private static final double LL_LATERAL_OFFSET_IN = 5;        // camera from robot center (inches)

    // -------- Flywheel speed prediction model (distanceIn inches -> rpm) --------
    // Linear regression: y=5.51221x+3022.06487
    private static final double RPM_SLOPE = 5.51221; //may need to redo equation
    private static final double RPM_INTERCEPT = 3022.06487;

    private static final double PRED_RPM_MIN = 1500;
    private static final double PRED_RPM_MAX = 5800;

    // Manual fallback RPMs if goal not seen
    private static final double MANUAL_RPM_UP = 3300;    // near
    private static final double MANUAL_RPM_DOWN = 3900;  // far

    // ---------------- LED light -------------------
    private static final double LED_GREEN_POS = 0.5;   // Green
    private static final double LED_RED_POS = 0.277;   // Red (optional)
    private static final double LED_OFF_POS = 0.0;     // off (try 1 if 0 doesn't work)
    private static final double LED_RPM_TOL = 150;     // speed tolerance

    // =========================
    // State
    // =========================
    private int goalTagId = BLUE_GOAL_TAG_ID;
    private boolean flywheelOn = false;

    // Flywheel targets
    private double flywheelTargetRPM = MANUAL_RPM_UP;
    private double lastPredictedRPM = MANUAL_RPM_UP;
    private double manualFallbackRPM = MANUAL_RPM_UP;

    // Flywheel tuning
    private static final double RPM_STEP = 100;              // change per button press
    private static final double RPM_MIN = 1500;
    private static final double RPM_MAX = 6000;



    // For field-centric drive
    private double initYawRad;

    @Override
    public void runOpMode() {

        initHardware();

        initYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("Ready. Drive to position.");
        telemetry.addLine("Aim: hold gamepad1 LEFT TRIGGER for aim assist to goal tag.");
        telemetry.addLine("Goal tag: gamepad1 X=Blue(20), B=Red(24)");
        //telemetry.addLine("Auto-shoot: hold gamepad2 RIGHT TRIGGER (uses RPM dip detection).");
        //telemetry.addLine("Manual RW2 pulse: gamepad2 LEFT BUMPER. Unjam: hold gamepad2 X.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1) Pick which goal tag you are aiming at
            if (gamepad1.xWasPressed()) goalTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.bWasPressed()) goalTagId = RED_GOAL_TAG_ID;

            // 2) Manual drive + optional aim assist overlay
            driveFieldCentricWithOptionalAimAssist();

            // 3) Limelight trig distance + predicted RPM (or manual fallback if no tag)
            Double predictedRPM = updateLimelightPoseAndDistanceTelemetry(goalTagId);

            // 4) Flywheel control (toggle Y); LED green when within tolerance
            applyFlywheelControl(predictedRPM);

            // 3) Flywheel RPM tuning
            //updateFlywheelRPMControls();
            //applyFlywheelControl();

            telemetry.update();
        }

        limelight.stop();
    }
    // =========================================================
    // HARDWARE INIT
    // =========================================================
    private void initHardware() {
        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");

        mFW = hardwareMap.get(DcMotorEx.class, "mFW");
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sG = hardwareMap.get(CRServo.class, "sG");
        //sF = hardwareMap.get(CRServo.class, "sF");

        imu = hardwareMap.get(IMU.class, "imu");

        // Directions: keep your working settings
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        //sG.setDirection(CRServo.Direction.REVERSE);
        //sF.setDirection(CRServo.Direction.REVERSE);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(7);  // AprilTag pipeline
        limelight.start();

        // LED (servo-like controller)
        //sLED = hardwareMap.get(Servo.class, "sLED");
    }
    // =========================================================
    // DRIVE + AIM ASSIST
    // =========================================================
    private void driveFieldCentricWithOptionalAimAssist() {

        // Field-centric drive from IMU
        double yawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double yawDeltaRad = yawRad - initYawRad;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turnManual = gamepad1.right_stick_x;

        // Rotate (x,y) by -yawDelta
        double cosA = Math.cos(-yawDeltaRad);
        double sinA = Math.sin(-yawDeltaRad);
        double xRot = (x * cosA) - (y * sinA);
        double yRot = (x * sinA) + (y * cosA);

        // Optional aim assist: hold left trigger
        double turnAssist = 0.0;
        boolean aimAssistEnabled = gamepad1.left_trigger > 0.2;

        LLResult result = limelight.getLatestResult();

        if (aimAssistEnabled) {
            Double txDeg = getTxToGoalTag(goalTagId, result);

            // If additional adjustment is used: need ty + trig distance
            Double tyDeg = getTyToGoalTag(goalTagId, result);
            Double trigDistIn = (tyDeg != null) ? trigDistanceToGoalInchesFromTy(tyDeg) : null;

            if (txDeg != null && trigDistIn != null) {
                // txSetpoint = atan(offset / distance)
                double txSetpointDeg = Math.toDegrees(Math.atan2(LL_LATERAL_OFFSET_IN, trigDistIn));
                //double txErrorDeg = txDeg - txSetpointDeg;
                double txErrorDeg = txDeg;
                turnAssist = clamp(txErrorDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);

                telemetry.addData("AimAssist", "ON tx=%.1f° set=%.1f° err=%.1f° turn=%.2f",
                        txDeg, txSetpointDeg, txErrorDeg, turnAssist);
            } else if (txDeg != null) {
                // Fall back to simple tx=0 targeting if distance not available
                turnAssist = clamp(txDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);
                telemetry.addData("AimAssist", "ON tx=%.1f° turn=%.2f (no trig dist)", txDeg, turnAssist);
            } else {
                telemetry.addData("AimAssist", "ON (no goal tag in view)");
            }
        } else {
            telemetry.addData("AimAssist", "OFF");
        }

        double turn = turnManual + turnAssist;

        // Mecanum mixing
        double pFL = yRot + xRot + turn;
        double pFR = yRot - xRot - turn;
        double pBL = yRot - xRot + turn;
        double pBR = yRot + xRot - turn;

        // Normalize
        double max = Math.max(Math.max(Math.abs(pFL), Math.abs(pFR)),
                Math.max(Math.abs(pBL), Math.abs(pBR)));
        if (max > 1.0) { pFL /= max; pFR /= max; pBL /= max; pBR /= max; }

        mFL.setPower(pFL);
        mFR.setPower(pFR);
        mBL.setPower(pBL);
        mBR.setPower(pBR);

        telemetry.addData("Goal Tag", goalTagId);
        telemetry.addData("IMU Yaw", "%.1f°", yawRad);
    }
    // =========================================================
    // 2) FLYWHEEL RPM: toggle + adjust RPM
    //    - gamepad2 Y toggles flywheel ON/OFF
    //    - gamepad2 dpad up/down changes RPM target
    //    - uses DcMotorEx.setVelocity(ticksPerSecond)
    // =========================================================
    private void updateFlywheelRPMControls() {
        if (gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;
        }

        if (gamepad2.dpad_up) {
            flywheelTargetRPM += RPM_STEP;
        } else if (gamepad2.dpad_down) {
            flywheelTargetRPM -= RPM_STEP;
        }

        flywheelTargetRPM = clamp(flywheelTargetRPM, RPM_MIN, RPM_MAX);
    }

    private void applyFlywheelControl() {
        double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;

        if (!flywheelOn) {
            mFW.setPower(0.0);
            telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
            return;
        }

        // Convert RPM -> ticks/sec for setVelocity
        double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
        mFW.setVelocity(targetTicksPerSec);


        telemetry.addData("Flywheel", "ON Target=%.0f RPM  Current=%.0f RPM",
                flywheelTargetRPM, currentRPM);
    }

    // =========================================================
    // LIMELIGHT TRIG DIST + PREDICTED RPM (or manual fallback)
    // =========================================================
    private Double updateLimelightPoseAndDistanceTelemetry(int goalTagId) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {

            // Manual select (held dpad, simple & reliable)
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("LL", "No valid result");
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        telemetry.addData("LL Pipeline", "Index=%d Type=%s",
                result.getPipelineIndex(), result.getPipelineType());

        Double tyDeg = getTyToGoalTag(goalTagId, result);
        Double txDeg = getTxToGoalTag(goalTagId, result);

        if (tyDeg == null) {
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("TrigDist", "n/a (goal tag %d not in view)", goalTagId);
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        Double trigDistIn = trigDistanceToGoalInchesFromTy(tyDeg);
        if (trigDistIn == null) {
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("TrigDist", "invalid (ty=%.2f°)", tyDeg);
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        // Predicted RPM from your linear model
        double predictedRPM = RPM_SLOPE * trigDistIn + RPM_INTERCEPT;
        predictedRPM = clamp(predictedRPM, PRED_RPM_MIN, PRED_RPM_MAX);

        telemetry.addData("TrigDist", "%.1f in (Tx=%.1f Ty=%.1f) to Goal %d",
                trigDistIn, (txDeg != null ? txDeg : 999.0), tyDeg, goalTagId);
        telemetry.addData("PredRPM", "%.0f RPM", predictedRPM);
        telemetry.addData("ManualRPM", "%.0f (use only if goal not seen)", manualFallbackRPM);

        return predictedRPM;
    }


    // =========================================================
    // FLYWHEEL CONTROL (toggle gamepad2 Y)
    // =========================================================
    private void applyFlywheelControl(Double predictedRPM) {

        double currentRPM = getFlywheelRPM();

        // Track last good prediction
        if (predictedRPM != null) {
            lastPredictedRPM = predictedRPM;
        }

        // Toggle flywheel ON/OFF with gamepad2 Y
        if (gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                double target = (predictedRPM != null) ? predictedRPM : lastPredictedRPM;
                flywheelTargetRPM = clamp(target, PRED_RPM_MIN, PRED_RPM_MAX);
            }
        }
        // OFF behavior
        if (!flywheelOn) {
            mFW.setPower(0.0);
            //sLED.setPosition(LED_OFF_POS);
            telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
            telemetry.addData("TargetRPM", "%.0f", flywheelTargetRPM);
            return;
        }

        // ON behavior
        double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
        mFW.setVelocity(targetTicksPerSec);

        boolean atSpeed = Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL;
        //sLED.setPosition(atSpeed ? LED_GREEN_POS : LED_OFF_POS);

        telemetry.addData("Flywheel", "ON  Target=%.0f  Current=%.0f", flywheelTargetRPM, currentRPM);
        telemetry.addData("AtSpeed", atSpeed ? "YES" : "NO");
    }


    //----------------------------------
// Helper class
// ----------------------------------


    private double getFlywheelRPM() {
        return mFW.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private boolean flywheelReady() {
        if (!flywheelOn) return false;
        double rpm = getFlywheelRPM();
        return Math.abs(rpm - flywheelTargetRPM) <= LED_RPM_TOL;
    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private Double getTxToGoalTag(int desiredId, LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == desiredId) {
                return f.getTargetXDegrees();
            }
        }
        return null;
    }

    private Double getTyToGoalTag(int desiredId, LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == desiredId) {
                return f.getTargetYDegrees();
            }
        }
        return null;
    }

    private Double trigDistanceToGoalInchesFromTy(double tyDeg) {
        double totalAngleDeg = LL_MOUNT_ANGLE_DEG + tyDeg;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        double heightDiffIn = GOAL_TAG_CENTER_HEIGHT_IN - LL_LENS_HEIGHT_IN;

        double tanVal = Math.tan(totalAngleRad);
        if (Math.abs(tanVal) < 1e-6) return null;

        double dIn = heightDiffIn / tanVal;

        if (dIn <= 0) return null;
        return dIn;
    }
}

