
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
@Autonomous(name = "AutoV1_RedGoal", group = "Autonomous")

public class AutoV1_RedGoal extends LinearOpMode {

    // Intake servo initialization
    public class SI {
        private CRServo sI;

        public SI(HardwareMap hardwareMap) {
            sI = hardwareMap.get(CRServo.class, "sI");
            sI.setDirection(CRServo.Direction.REVERSE);
        }

        public class SISpin implements Action {
            @Override

            public boolean run (@NonNull TelemetryPacket packet) {
                sI.setPower(1);
                sleep(2000);
                return false;
            }
        }
        public Action sISpin() {
            return new SISpin();
        }


        public class SIStop implements Action {
            @Override

            public boolean run (@NonNull TelemetryPacket packet) {
                sI.setPower(0);
                return false;
            }
        }
        public Action sIStop() {
            return new SIStop();
        }
    }




    // Ramp wheel 1 servo initialization
    public class SRW1 {
        private CRServo sRW1;

        public SRW1(HardwareMap hardwareMap) {
            sRW1 = hardwareMap.get(CRServo.class, "sRW1");
            sRW1.setDirection(CRServo.Direction.REVERSE);
        }

        public class SRW1Spin implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                sRW1.setPower(1);
                sleep(1700);
                return false;
            }
        }

        public Action sRW1Spin() {
            return new SRW1Spin();
        }


        public class SRW1Stop implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                sRW1.setPower(0);
                return false;
            }
        }

        public Action sRW1Stop() {
            return new SRW1Stop();
        }
    }



    // Ramp wheel 2 servo initialization
    public class SRW2 {
        private CRServo sRW2;

        public SRW2(HardwareMap hardwareMap) {
            sRW2 = hardwareMap.get(CRServo.class, "sRW2");
            sRW2.setDirection(CRServo.Direction.REVERSE);
        }

        public class SRW2Spin implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                sRW2.setPower(1);
                sleep(1300);
                return false;
            }
        }

        public Action sRW2Spin() {
            return new SRW2Spin();
        }


        public class SRW2Stop implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                sRW2.setPower(0);
                return false;
            }
        }

        public Action sRW2Stop() {
            return new SRW2Stop();
        }
    }



    // Flywheel initialization
    public class MFW {
        private DcMotorEx mFW;

        public MFW(HardwareMap hardwareMap) {
            mFW = hardwareMap.get(DcMotorEx.class, "mFW");
            mFW.setDirection(DcMotor.Direction.FORWARD);
        }

        public class MFWSpin implements Action {
            private boolean initialized = false;


            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    mFW.setPower(1);
                    initialized = true;
                }
                return false;
            }
        }
        public Action mFWSpin() {
            return new MFWSpin();
        }


        public class MFWStop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    mFW.setPower(0);
                    initialized = true;
                }
                return false;
            }
        }
        public Action mFWStop() {
            return new MFWStop();
        }
    }



    @Override
    public void runOpMode() {
        Pose2d initPose = new Pose2d(-63.5, 48, Math.toRadians(315));
        Pose2d shootPose = new Pose2d(-2, 15, Math.toRadians(138));
        Pose2d intake1Pose = new Pose2d(-11, 29, Math.toRadians(90));
        Pose2d intake2Pose = new Pose2d(-11, 34, Math.toRadians(90));
        Pose2d intake3Pose = new Pose2d(-11, 40, Math.toRadians(90));
        Vector2d shootVec = new Vector2d(-2, 15);
        Vector2d intake1Vec = new Vector2d(-11, 29);
        Vector2d intake2Vec = new Vector2d(-11, 34);
        Vector2d intake3Vec = new Vector2d(-11, 40);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        SI sI = new SI(hardwareMap);
        SRW1 sRW1 = new SRW1(hardwareMap);
        SRW2 sRW2 = new SRW2(hardwareMap);
        MFW mFW = new MFW(hardwareMap);


        TrajectoryActionBuilder initToShoot = drive.actionBuilder(initPose)
                .strafeToLinearHeading(shootVec, Math.toRadians(138));
        TrajectoryActionBuilder shootToIntake1 = drive.actionBuilder(shootPose)
                .splineTo(intake1Vec, Math.toRadians(90));
        TrajectoryActionBuilder intake1ToIntake2 = drive.actionBuilder(intake1Pose)
                .splineTo(intake2Vec, Math.toRadians(90));
        TrajectoryActionBuilder intake2ToIntake3 = drive.actionBuilder(intake2Pose)
                .splineTo(intake3Vec, Math.toRadians(90));
        TrajectoryActionBuilder intake3ToShoot = drive.actionBuilder(intake3Pose)
                .strafeToLinearHeading(shootVec, Math.toRadians(138));


        if (isStopRequested())
            return;


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        initToShoot.build(),
                        mFW.mFWSpin(),
                        sRW2.sRW2Spin(),
                        sRW2.sRW2Stop(),
                        sRW1.sRW1Spin(),
                        mFW.mFWStop(),
                        sRW1.sRW1Stop(),
                        mFW.mFWSpin(),
                        sRW2.sRW2Spin(),
                        mFW.mFWStop(),
                        sRW2.sRW2Stop(),

                        shootToIntake1.build(),
                        sI.sISpin(),
                        sRW1.sRW1Spin(),
                        intake1ToIntake2.build(),
                        sRW1.sRW1Spin(),
                        sI.sISpin(),
                        intake2ToIntake3.build(),
                        sI.sIStop(),

                        intake3ToShoot.build(),
                        mFW.mFWSpin(),
                        sRW1.sRW1Spin(),
                        sRW2.sRW2Spin(),
                        sRW1.sRW1Stop(),
                        sI.sISpin(),
                        sRW1.sRW1Spin(),
                        sI.sIStop(),
                        sRW2.sRW2Spin(),
                        sRW1.sRW1Stop(),
                        mFW.mFWStop()
                )
        );
    }
}