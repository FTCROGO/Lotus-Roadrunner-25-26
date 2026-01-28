package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


    // MaxSpeed without using Encoder (code example from ChatGPT)

    @TeleOp(name="Flywheel Max Speed Test")
    public class Motor_MaxSpeed extends LinearOpMode {

        // goBILDA 5203 motor encoder counts per MOTOR revolution
        static final double TICKS_PER_REV = 28.0;

        boolean flywheelOn = false;
        boolean lastY = false;

        double maxTicksPerSecObserved = 0.0;
        double maxRPMObserved = 0.0;

        @Override
        public void runOpMode() {

            DcMotorEx mFW = hardwareMap.get(DcMotorEx.class, "mFW");
            mFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {

                // Edge-detect Y to toggle ON/OFF
                boolean y = gamepad2.y;
                if (y && !lastY) {
                    flywheelOn = !flywheelOn;

                    // Optional: reset peak each time you turn it ON
                    if (flywheelOn) {
                        maxTicksPerSecObserved = 0.0;
                        maxRPMObserved = 0.0;
                    }
                }
                lastY = y;

                if (flywheelOn) {
                    mFW.setPower(1.0);

                    double ticksPerSec = mFW.getVelocity();               // encoder ticks/sec
                    double rpm = ticksPerSec * 60.0 / TICKS_PER_REV;      // motor RPM

                    if (ticksPerSec > maxTicksPerSecObserved) maxTicksPerSecObserved = ticksPerSec;
                    if (rpm > maxRPMObserved) maxRPMObserved = rpm;

                    telemetry.addData("Flywheel", "ON (power=1.0)");
                    telemetry.addData("Current (ticks/s)", "%.0f", ticksPerSec);
                    telemetry.addData("Peak (ticks/s)", "%.0f", maxTicksPerSecObserved);
                    telemetry.addData("Current (motor RPM)", "%.0f", rpm);
                    telemetry.addData("Peak (motor RPM)", "%.0f", maxRPMObserved);

                } else {
                    mFW.setPower(0.0);
                    telemetry.addData("Flywheel", "OFF");
                    telemetry.addData("Peak (ticks/s)", "%.0f", maxTicksPerSecObserved);
                    telemetry.addData("Peak (motor RPM)", "%.0f", maxRPMObserved);
                }

                telemetry.update();
            }
        }
    }