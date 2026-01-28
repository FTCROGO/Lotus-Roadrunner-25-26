package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor_TicksPerRev", group = "Robot")
public class Motor_TicksPerRev extends LinearOpMode {
    public DcMotorEx flywheelMotor;

    @Override
    public void runOpMode() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "mFW");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Init complete");
        telemetry.update();

        waitForStart();

        // Keep the OpMode alive so you can read the encoder
        while (opModeIsActive()) {
            telemetry.addData("Encoder position", flywheelMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
