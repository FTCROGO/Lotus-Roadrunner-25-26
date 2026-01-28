package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Motor_TicksPerRev {
    public DcMotorEx flywheelMotor;

   // @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "mFW");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Init complete");

    }

    //@Override
    public void loop() {
        // to test TICKS_PER_REV (Code example from ChatGPT)
//Results interpretation
//If it shows ~28 → use 28
//If it shows ~112 → use 112
//If it shows ~537 → gearbox is involved (old value)
        //This test is bulletproof and worth doing once.

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Slowly turn flywheel by hand exactly ONE revolution
        // then read:
        telemetry.addData("Encoder position", flywheelMotor.getCurrentPosition());
    }
}
