package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Configurable
public class DriveCommands {

    private DriveCommands() {}

    public BooleanSupplier isStationary() {
        return () -> (Constants.localizer.getVelocity().getX() < 2) && (Constants.localizer.getVelocity().getY() < 2) && (Constants.localizer.getVelocity().getHeading() < 0.1);
    }


    public static Command drive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(
                Commands.run(
                        () -> drive.drive(
                                xSupplier,
                                ySupplier,
                                rotationSupplier,isFieldCentric),
                        drive));

    }
    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */

    // Drive to point — precise arrival, holds end, finishes when within tolerance
    public static Command driveToPose(Drive drive, Supplier<Pose> pose) {
        return Drive.followPath(
                drive,
                () -> drive.getPathBuilder()
                        .addPath(new Path(new BezierLine(drive.getPose(), pose.get())))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), pose.get().getHeading())
                        .build());
    }

    public static Command driveToPose(Drive drive, Supplier<Pose> pose, Command command, DoubleSupplier commandActivationPoint) {
        return Commands.parallel(
                driveToPose(drive, pose),
                Commands.sequence(
                        Commands.waitUntil(() -> drive.getPathT() > commandActivationPoint.getAsDouble()),
                        command
                )
        );
    }


    public static Command driveNearPoint(Drive drive, Supplier<Pose> pose, double radiusInches) {
        return Commands.runOnce(() -> {
            Pose current = drive.getPose();
            Pose target = pose.get();
            drive.followPath(drive.getFollower().pathBuilder().addPath(new Path(new BezierLine(current, target)))
                    .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                    .build());
        }, drive).andThen(
                Commands.waitUntil(() -> {
                    Pose current = drive.getPose();
                    Pose target = pose.get();
                    double dx = current.getX() - target.getX();
                    double dy = current.getY() - target.getY();
                    return (dx * dx + dy * dy) <= radiusInches * radiusInches;
                })
        );
    }

    public static Command driveTangential(Drive drive, boolean reversed, boolean red, TuneablePose endPose, TuneablePose... controlPoints) {
        return Commands.runOnce(() -> {
            ArrayList<Pose> poseList = new ArrayList<>();

            poseList.add(drive.getPose());
            PathBuilder builder = drive.getFollower().pathBuilder();
            for (TuneablePose cp : controlPoints) {
                poseList.add(cp.getCorrectPose(red));
            }

            poseList.add(endPose.getCorrectPose(red));

            builder.addPath(
              new BezierCurve(
                      poseList
              )
            );

            if (reversed) {
                builder.setReversed();
            }

            drive.followPath(builder.build());
        }, drive).andThen(
                Commands.waitUntil(drive::isFinished)
        );
    }

    public static Command driveTangentialNear(Drive drive, boolean reversed, boolean red, double radiusInches, TuneablePose endPose, TuneablePose... controlPoints) {
        return Commands.runOnce(() -> {
            ArrayList<Pose> poseList = new ArrayList<>();

            poseList.add(drive.getPose());
            PathBuilder builder = drive.getFollower().pathBuilder();
            for (TuneablePose cp : controlPoints) {
                poseList.add(cp.getCorrectPose(red));
            }

            poseList.add(endPose.getCorrectPose(red));

            builder.addPath(
                    new BezierCurve(
                            poseList
                    )
            );

            if (reversed) {
                builder.setReversed();
            }

            drive.followPath(builder.build());

        }, drive).andThen(
                Commands.waitUntil(() -> {
                    Pose current = drive.getPose();
                    Pose target = endPose.getCorrectPose(red);
                    double dx = current.getX() - target.getX();
                    double dy = current.getY() - target.getY();
                    return (dx * dx + dy * dy) <= radiusInches * radiusInches;
                }));
    }

    public static Command driveLinear(Drive drive, Supplier<Pose> endPose, DoubleSupplier endHeading, Supplier<Pose>... controlPoints) {
        return Commands.runOnce(() -> {
            Pose startPose = drive.getPose();
            PathBuilder builder = drive.getFollower().pathBuilder();
            for (Supplier<Pose> cp : controlPoints) {
                Pose next = cp.get();
                builder.addPath(
                                new BezierCurve(
                                        endPose.get(),
                                        next
                                ))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endHeading.getAsDouble());
            }

            drive.followPath(builder.build());
        }, drive).andThen(
                Commands.waitUntil(drive::isFinished)
        );
    }


    public static Command setPose(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> drive.setPose(pose.get()));
    }

    public static Command setHeading(Drive drive, DoubleSupplier heading) {
        return Commands.runOnce(() -> drive.setPose(new Pose(drive.getPose().getX(), drive.getPose().getY(), heading.getAsDouble())), drive);
    }



    public static Command turn(Drive drive, DoubleSupplier angle) {
        return Drive.followPath(
                drive,
                () -> drive.getPathBuilder()
                        .addPath(new Path(new BezierPoint(drive.getDesiredPose().getPose())))
                        .setConstantHeadingInterpolation(drive.getDesiredPose().getPose().getHeading() + Math.toRadians(angle.getAsDouble()))
                        .build()).until(drive::headingIsFinished).andThen(Commands.waitSeconds(0.1));
    }

    public static Command setPoseStorage(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> {
            PoseStorage.currentPose = pose.get();
        }, drive);
    }

    public static double signSquare(double num) {
        return num * num * Math.signum(num);
    }

}

