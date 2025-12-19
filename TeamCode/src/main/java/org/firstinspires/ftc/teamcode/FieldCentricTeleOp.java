package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    // private DcMotor intake = null;

    @Override
    public void init() {
        driver = new DriverDanny(hardwareMap,
                                    telemetry,
                                    DriverDanny.Poses.RED_FAR_START_POSE,
                                    DriverDanny.Team.RED);
        //intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        driver.update();
        shooter.update();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driver.fieldCentricDrive(forward, strafe, rotate);

        if (gamepad2.right_trigger > 0) {
            //shooter.startShootingAtVelocity(1100);
            shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
        }

        if (gamepad2.left_trigger > 0) {
            //shooter.startShootingAtVelocity(1400);
            shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
        }

        if(gamepad2.left_bumper) {
            // auto aim testing!
            // get difference in X/Y coordinates from the goal
            Pose currentPose = driver.getPose();
            Pose goalPose = DriverDanny.Poses.BLUE_GOAL_POSE;

            double dx = goalPose.getX() - currentPose.getX();
            double dy = goalPose.getY() - currentPose.getY();

            // use atan2 to get heading adjustment in radians
            double targetHeading = Math.atan2(dy, dx);

            // get how much we need to change our heading
            double headingError = AngleUnit.normalizeRadians(targetHeading - currentPose.getHeading());

            // if headingError > PI, turn left; else turn right
            //boolean isLeft = headingError > Math.PI;

            // use pedro pathing to turn towards target
            driver.turn(headingError, true);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}


