package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.9) // TODO: Change this to the actual weight of the robot
            .forwardZeroPowerAcceleration(-31.6509)
            .lateralZeroPowerAcceleration(-69.7608)
            // Set following parameters to true to enable dual PID
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0002)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.000003, 0.02, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.03, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.002, 0.6, 0.02));

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.4037)
            .yVelocity(57.4415);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.25)
            .strafePodX(-6.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    private static FusionLocalizer fusionLocalizer = null;

    private static PinpointLocalizer pinpointLocalizer = null;

    public static Follower createFollower(HardwareMap hardwareMap) {
        pinpointLocalizer= new PinpointLocalizer(hardwareMap, localizerConstants);

        fusionLocalizer = new FusionLocalizer(
                pinpointLocalizer,
                new Pose(0.5, 0.5, 0.05), // P: initial covariance
                new Pose(1.0, 1.0, 0.1), // Q : process variance
                new Pose(4.0, 4.0, 0.04), // R: measurement variance
                100 // bufferSize
        );


        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(fusionLocalizer)
                .build();
    }


    public static PinpointLocalizer getPinpointLocalizer() {
        return pinpointLocalizer;
    }

    public static FusionLocalizer getFusionLocalizer() {
        return fusionLocalizer;
    }
}
