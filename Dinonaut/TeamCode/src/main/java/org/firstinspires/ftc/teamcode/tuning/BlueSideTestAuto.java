package org.firstinspires.ftc.teamcode.tuning;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {

    public class Lift {
        private DcMotorEx liftR;
        private DcMotorEx liftL;

        public Lift(HardwareMap hardwareMap) {
            liftR = hardwareMap.get(DcMotorEx.class, "liftMotorR");
            liftL = hardwareMap.get(DcMotorEx.class, "liftMotorL");
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-0.9);
                    liftL.setPower(0.9);
                    initialized = true;
                }

                double posR = liftR.getCurrentPosition();
                double posL = liftL.getCurrentPosition();

                packet.put("liftPos", posL);
                if (posL > -1000) {
                    return true;
                } else {
                    liftR.setPower(0);
                    liftL.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-0.8);
                    liftL.setPower(0.8);
                    initialized = true;
                }

                double pos = liftR.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    liftR.setPower(0);
                    liftL.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }







    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "S0");
            gripper.setPosition(0.65);
        }

        public class MidUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.5);
                return false;
            }
        }
        public Action midUp() {
            return new MidUP();
        }

        public class PushDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.15);
                return false;
            }
        }
        public Action pushDown() {
            return new PushDown();
        }
        public class PushUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.65);
                return false;
            }
        }
        public Action pushUp() {
            return new PushUp();
        }
    }
    public class Slide {
        private Servo slide;

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(Servo.class, "S5");
            slide.setPosition(0);
        }

        public class Full implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setPosition(0);
                return false;
            }
        }

        public Action full() {
            return new Full();
        }

        public class Back implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setPosition(0.5);
                return false;
            }
        }

        public Action back() {
            return new Back();
        }
    }
    public class Rot {
        private Servo rotate;

            public Rot(HardwareMap hardwareMap) {
                rotate = hardwareMap.get(Servo.class, "S1");
                rotate.setPosition(0);
            }

            public class Down implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    rotate.setPosition(0);
                    return false;
                }
            }

            public Action down() {
                return new Down();
            }

            public class Up implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    rotate.setPosition(0.82);
                    return false;
                }
            }

            public Action up() {
                return new Up();
            }
        }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Gripper gripper = new Gripper(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Rot rot = new Rot(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                //.setTangent(Math.toRadians(90))
                //.lineToY(0)
                //.setTangent(0)
                .splineToSplineHeading( new Pose2d(25,-23.5,0),-Math.PI*0)
                //.setTangent(Math.toRadians(0))
                //.lineToX(20)
                //.waitSeconds(4)
                //.lineToX(40)
                //.waitSeconds(1)
                //.strafeTo( new Vector2d(24,-23.5))
                //.waitSeconds(1)
                //.strafeTo( new Vector2d(24,-33.5  ))
                //.splineTo( new Vector2d(20,20),Math.PI/2)
                //.splineTo( new Vector2d(20,-20),-Math.PI/2)
                //.setTangent(Math.toRadians(0))
                //.lineToX(20)
                //.strafeTo(new Vector2d(44.5, 30))
                //.turn(Math.toRadians(180))
                //.lineToX(47.5)
                .waitSeconds(0.3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(4,-23.5,180),-Math.PI*0)
                //.setTangent(0)
                //.lineToXSplineHeading(7,Math.toRadians(180))
                //.lineToX(7)
                .waitSeconds(0.3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(25.5,-33.5,0),-Math.PI*0)
                //.setTangent(180)
                //.lineToX(7)
                .waitSeconds(0.3);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(5,-23.5,180),-Math.PI*0)
                .waitSeconds(0.3);
        TrajectoryActionBuilder Tomid = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(24,20,0),Math.PI*0)
                .turnTo(Math.toRadians(180))
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(26.2,-38.2,0),-Math.PI*0)
                .waitSeconds(0.3)
                .turnTo(Math.toRadians(-30));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(5,-33.5,180),-Math.PI*0)
                .waitSeconds(0.3);
      /*Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

         */

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(gripper.pushUp());
        Actions.runBlocking(slide.back());
        Actions.runBlocking(rot.down());
        //Actions.runBlocking(lift.liftUp());
        /*while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

         */

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        Action trajectoryActionChosen2;
        Action trajectoryActionChosen3;
        Action trajectoryActionChosen4;
        Action trajectoryActionChosen5;
        Action trajectoryActionChosen6;
        Action Middle;
        trajectoryActionChosen = tab1.build();
        trajectoryActionChosen2 = tab2.build();
        trajectoryActionChosen3 = tab3.build();
        trajectoryActionChosen4 = tab4.build();
        trajectoryActionChosen5 = tab5.build();
        trajectoryActionChosen6 = tab6.build();
        Middle = Tomid.build();
        Actions.runBlocking(
                new SequentialAction(
                        Middle,
                        trajectoryActionChosen,
                        //lift.liftUp(),
                        slide.full(),
                        new SleepAction(0.5),
                        gripper.pushDown(),
                        new SleepAction(0.35),
                        gripper.midUp(),
                        trajectoryActionChosen2,
                        gripper.pushUp(),
                        trajectoryActionChosen3,
                        gripper.pushDown(),
                        new SleepAction(0.4),
                        gripper.midUp(),
                        trajectoryActionChosen4,
                        gripper.pushUp(),
                        rot.up(),
                        trajectoryActionChosen5,
                        rot.down(),
                        new SleepAction(1),
                        gripper.pushDown(),
                        trajectoryActionChosen5,
                        new SleepAction(0.4),
                        gripper.midUp(),
                        trajectoryActionChosen6,
                        gripper.pushUp(),
                        new SleepAction(1)

        //lift.liftDown(),
                        //trajectoryActionCloseOut
                )
        );



    }
}