//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name="Sample Autonomous", group = "Autonomous")
//public class Auto extends LinearOpMode
//{
//    // get an instance of the "Robot" class.
//    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
//
//    @Override public void runOpMode()
//    {
//        // Initialize the robot hardware & Turn on telemetry
//        robot.initialize(true);
//
//        // Wait for driver to press start
//        telemetry.addData(">", "Touch Play to run Auto");
//        telemetry.update();
//
//        waitForStart();
//        robot.resetHeading();  // Reset heading to set a baseline for Auto
//
//        // Run Auto if stop was not pressed.
//        if (opModeIsActive())
//        {
//            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.
//
//            // Drive a large rectangle, turning at each corner
//            robot.drive(  84, 0.60, 0.25);
//            robot.turnTo(90, 0.45, 0.5);
//            robot.drive(  72, 0.60, 0.25);
//            robot.turnTo(180, 0.45, 0.5);
//            robot.drive(  84, 0.60, 0.25);
//            robot.turnTo(270, 0.45, 0.5);
//            robot.drive(  72, 0.60, 0.25);
//            robot.turnTo(0, 0.45, 0.5);
//
//            sleep(500);
//
//            // Drive the path again without turning.
//            robot.drive(  84, 0.60, 0.15);
//            robot.strafe( 72, 0.60, 0.15);
//            robot.drive( -84, 0.60, 0.15);
//            robot.strafe(-72, 0.60, 0.15);
//
//            private ElapsedTime runtime = new ElapsedTime();
//
//            private DcMotor shootermotor = null;
//
//            @Override public void runOpMode(){
//
//            shootermotor = hardwareMap.get(DcMotor.class, "shootermotor");
//
//            shootermotor.setDirection(DcMotor.Direction.REVERSE);
//
//            waitForStart();
//            runtime.reset();
//            while (opModeIsActive()) {
//                if (gamepad1.right_bumper) {
//                    shootermotor.setPower(1z);
//                } else {
//                    shootermotor.setPower(0);
//                }
//            }
//        }
//    }
//}
//
