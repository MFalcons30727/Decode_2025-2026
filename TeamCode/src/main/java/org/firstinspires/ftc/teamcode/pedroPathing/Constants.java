package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.608) //has to be in kilograms, this is just a placeholder
            // the placeholders I have right now are for the tuning part of this
            // TODO need to get values from telemetry by running each under automatic
            .forwardZeroPowerAcceleration(-36.23251140162128)
            .lateralZeroPowerAcceleration(-57.76687780044757)
            // the PIDF needs to be tuned accordingly, these are also placeholders
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.005, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.88, 0, 0.07, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06, 0, 0.0001, 0.4, 0.0025))
            //centripetal scaling is for curves, doesn't matter as much.
            .centripetalScaling(0.0005)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // TODO this is the same as the top, 1 is just acting as a placeholder
            .xVelocity(62.93198040338952)
            .yVelocity(49.822540523499015)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.417)
            .strafePodX(-5.67)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.25,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}