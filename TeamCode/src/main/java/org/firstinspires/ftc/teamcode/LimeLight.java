package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name="LimeLight")
public class LimeLight extends LinearOpMode {
    private Limelight3A LimeLight;
    private Servo led;

    @Override
    public void runOpMode() {
        // Initialize Limelight
        LimeLight = hardwareMap.get(Limelight3A.class, "LimeLight");
        led = hardwareMap.get(Servo.class, "led");


        // Start with AprilTag pipeline (pipeline 0)
        LimeLight.pipelineSwitch(0);
        LimeLight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = LimeLight.getLatestResult();


            if (result != null) {

                // APRILTAG DETECTION (for obelisk sides)
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                double distance = 0;
                if (!fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();
                        double tx = fiducial.getTargetXDegrees();
                        double ty = fiducial.getTargetYDegrees();
                        distance = result.getBotposeAvgDist();
                        telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

                        Object aprilTagID = null;
                        telemetry.addData("AprilTag ID", (Object) null);
                        telemetry.addData("AprilTag X", tx);
                        telemetry.addData("AprilTag Y", ty);
                    }
                }

                if (distance < 1 && distance > 0.8) {
                    led.setPosition(0.5);
                }
                else {
                    led.setPosition(0.2);
                }

                // COLOR DETECTION (for green/purple balls)
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                if (!colorResults.isEmpty()) {
                    for (LLResultTypes.ColorResult color : colorResults) {
                        double ballX = color.getTargetXDegrees();
                        double ballY = color.getTargetYDegrees();
                        double ballArea = color.getTargetArea();

                        telemetry.addData("Ball X", ballX);
                        telemetry.addData("Ball Y", ballY);
                        telemetry.addData("Ball Area", ballArea);
                    }
                } else {
                    telemetry.addData("Balls", "None detected");
                }
            }

            telemetry.update();
        }

        LimeLight.stop();
    }
}

