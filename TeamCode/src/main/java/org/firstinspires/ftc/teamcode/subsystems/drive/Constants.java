package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import    com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static Mecanum mec;
    public static PinpointLocalizer localizer;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-19.12)
            .lateralZeroPowerAcceleration(-52.45)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.0005,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,0.01,.6,0.02))
            .centripetalScaling(0.00008)
            .drivePIDFSwitch(15)
            .mass(15)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.0682685, 0.0020363124));

    public static PathConstraints pathConstraints = new PathConstraints(0.995, .1, .1, 0.009, 50, 1.25, 10, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(86.68)
            .yVelocity(66.66)
            .useBrakeModeInTeleOp(true)
            .rightRearMotorName("BR")
            .rightFrontMotorName("FR")
            .leftFrontMotorName("FL")
            .leftRearMotorName("BL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-122/25.4)
            .strafePodX(-1.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        mec = new Mecanum(hardwareMap, driveConstants);
        localizer = new PinpointLocalizer(hardwareMap, localizerConstants);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(mec)
                .setLocalizer(localizer)
                .build();
    }
}
