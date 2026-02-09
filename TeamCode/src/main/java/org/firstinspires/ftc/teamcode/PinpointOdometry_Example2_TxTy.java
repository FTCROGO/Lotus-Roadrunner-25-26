
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "PinpointOdometry_Example2_TxTy", group = "Robot")
public class PinpointOdometry_Example2_TxTy extends LinearOpMode {

    private static final String PINPOINT_NAME = "pinpoint";

    // ===== Tuning (start conservative) =====
    private static final double KP_POS = 1.2;      // power per meter of error
    private static final double MIN_POWER = 0.12;  // helps overcome static friction
    private static final double TOL_M = 0.02;      // 2 cm tolerance
    private static final long TIMEOUT_MS = 5000;   // safety timeout per move

    // ===== Target distances (meters) =====
    private static final double TARGET_X_M = 0.80; // forward
    private static final double TARGET_Y_M = 0.30; // left

    // ===== Max power =====
    private static final double MAX_POWER = 0.8
            ;

    // Hardware
    private DcMotor mFL, mFR, mBL, mBR;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // --- Drive motors (rename to match your config) ---
        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");

        // Common mecanum direction pattern
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.REVERSE);

        // Optional: braking helps stop closer to target
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Pinpoint ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Required Pinpoint configuration (your working settings)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setOffsets(-114.3, 38.1, DistanceUnit.MM);

        // Pre-start telemetry
        telemetry.addLine("Decode Auto Ready");
        telemetry.addLine("Will drive: +X 0.50m, then +Y 0.30m");
        telemetry.addLine("Keep robot still after INIT.");
        telemetry.update();

        waitForStart();

        // Safety
        if (isStopRequested()) return;

        // Reset pose and IMU at the start of auto
        pinpoint.resetPosAndIMU();
        sleep(250); // brief settle

        // Step 1: forward
        driveToX(TARGET_X_M, MAX_POWER);
        sleep(250);

        // Step 2: left strafe
        driveToY(TARGET_Y_M, MAX_POWER);
        sleep(250);

        stopDrive();

        // Final pose report
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        telemetry.addLine("Auto complete.");
        telemetry.addData("Final X (m)", "%.3f", pose.getX(DistanceUnit.METER));
        telemetry.addData("Final Y (m)", "%.3f", pose.getY(DistanceUnit.METER));
        telemetry.update();

        sleep(1000);
    }

    // =========================================================
    // Reusable motion helpers (odometry-based)
    // =========================================================

    /** Drives robot forward/back until Pinpoint X reaches targetX_m (meters). */
    private void driveToX(double targetX_m, double maxPower) {
        long start = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < TIMEOUT_MS) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double x = pose.getX(DistanceUnit.METER);
            double error = targetX_m - x;

            if (Math.abs(error) <= TOL_M) break;

            double cmd = clip(KP_POS * error, -maxPower, maxPower);
            if (Math.abs(cmd) < MIN_POWER) cmd = Math.copySign(MIN_POWER, cmd);

            setDriveXY(cmd, 0.0);

            telemetry.addData("driveToX target/current (m)", "%.2f / %.2f", targetX_m, x);
            telemetry.addData("error (m)", "%.3f", error);
            telemetry.update();
        }

        stopDrive();
    }

    /** Strafes robot left/right until Pinpoint Y reaches targetY_m (meters). */
    private void driveToY(double targetY_m, double maxPower) {
        long start = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < TIMEOUT_MS) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double y = pose.getY(DistanceUnit.METER);
            double error = targetY_m - y;

            if (Math.abs(error) <= TOL_M) break;

            double cmd = clip(KP_POS * error, -maxPower, maxPower);
            if (Math.abs(cmd) < MIN_POWER) cmd = Math.copySign(MIN_POWER, cmd);

            setDriveXY(0.0, cmd);

            telemetry.addData("driveToY target/current (m)", "%.2f / %.2f", targetY_m, y);
            telemetry.addData("error (m)", "%.3f", error);
            telemetry.update();
        }

        stopDrive();
    }

    // xPower: forward/back (+ forward), yPower: left/right (+ left)
    private void setDriveXY(double xPower, double yPower) {
        double pFL = xPower + yPower;
        double pFR = xPower - yPower;
        double pBL = xPower - yPower;
        double pBR = xPower + yPower;

        // Normalize so no motor exceeds 1.0
        double max = Math.max(Math.max(Math.abs(pFL), Math.abs(pFR)),
                Math.max(Math.abs(pBL), Math.abs(pBR)));
        if (max > 1.0) {
            pFL /= max; pFR /= max; pBL /= max; pBR /= max;
        }

        mFL.setPower(pFL);
        mFR.setPower(pFR);
        mBL.setPower(pBL);
        mBR.setPower(pBR);
    }

    private void stopDrive() {
        mFL.setPower(0);
        mFR.setPower(0);
        mBL.setPower(0);
        mBR.setPower(0);
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}