package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)){
        //if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap,beginPose);
            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToSplineHeading( new Pose2d(24,5,1.57),-Math.PI)
                        .waitSeconds(2)
                        .splineToSplineHeading( new Pose2d(0,0,0),Math.PI)
                        //.splineTo(new Vector2d(96, 24), Math.PI / 2)
                        //.splineTo(new Vector2d(0, 0), Math.PI)
                        .build());

        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
