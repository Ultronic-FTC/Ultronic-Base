package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.TeleConstants.heading_angle_threshold;
import static org.firstinspires.ftc.teamcode.subsystems.drive.TeleConstants.heading_power_threshold;
import static org.firstinspires.ftc.teamcode.subsystems.drive.TeleConstants.max_heading_power;
import static org.firstinspires.ftc.teamcode.subsystems.drive.TeleConstants.teleOP_Squid_P;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathPoint;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Follower drive;
    private final KTelemetry telemetry;

    private SquIDController headingTelePID;


    private double distanceToGoal = 0;

    public Drive(HardwareMap hwMap, KTelemetry telemetry) {
        drive = Constants.createFollower(hwMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        drive.followPath(new Path(new BezierPoint(new Pose())));
        this.telemetry = telemetry;

        headingTelePID = new SquIDController(teleOP_Squid_P);
    }

    @Override
    public void periodic() {
        try {
            drive.update();
            telemetry.addData("Pose: X", drive.getPose().getX());
            telemetry.addData("Pose: Y", drive.getPose().getY());
            telemetry.addData("Pose: Heading", drive.getPose().getHeading());
            telemetry.addData("Pose: Heading Deg", Math.toDegrees(drive.getPose().getHeading()));


        } catch (Exception ignored) {
        }
    }

    public void setPose(Pose pose) {
        drive.setPose(pose);
    }

    public void startTeleopDrive() {
        drive.startTeleopDrive();
    }

    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        drive.setTeleOpDrive(xSupplier.getAsDouble() , ySupplier.getAsDouble(), rotationSupplier.getAsDouble(), isFieldCentric.getAsBoolean());
    }

    public Pose getPose() {
        return drive.getPose();
    }

    public Vector getVelocity() {
        return drive.getVelocity();
    }

    public boolean isStill() {
        return getVelocity().getMagnitude() < 0.1 && getVelocity().getTheta() < 0.1;
    }

    public double getXVelocity() {
        return drive.getDrivetrain().xVelocity();
    }

    public double getYVelocity() {
        return drive.getDrivetrain().xVelocity();
    }


    public double getHeadingVelocity() {
        return drive.getAngularVelocity();
    }

    public PathPoint getDesiredPose() {
        return drive.getClosestPose();
    }

    public double getPathT() {
        return drive.getCurrentTValue();
    }

    public PathBuilder getPathBuilder() {
        return drive.pathBuilder();
    }

    public void followPath(PathChain path) {
        drive.followPath(path, true);
    }

    public boolean isFinished() {
        return !drive.isBusy();
    }

    public boolean headingIsFinished() {
        return drive.getHeadingError() < 0.001; // Adjust this threshold as needed
    }

    public double speed(){
        return Math.abs(drive.getDrivetrain().xVelocity())+Math.abs(drive.getDrivetrain().yVelocity());

    }

    public static Command followPath(Drive drive, Pose startPose, PathChain path) {
        return Commands.runOnce(() -> drive.setPose(startPose), drive)
                .andThen(Commands.runOnce(() -> drive.followPath(path), drive));
    }

    public static Command followPath(Drive drive, PathChain path) {
        return Commands.runOnce(() -> drive.followPath(path), drive).andThen(Commands.waitUntil(drive::isFinished));
    }

    public static Command followPath(Drive drive, Supplier<PathChain> path) {
        return Commands.runOnce(() -> drive.followPath(path.get()), drive).andThen(Commands.waitUntil(drive::isFinished));
    }
    public Follower getFollower() {
        return drive;
    }

    public void resetPinpoint() throws InterruptedException {
        drive.poseTracker.getLocalizer().resetIMU();
    }
}