package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Abby Test Omni Drive To AprilTag", group = "linear OpMode")
public class Abbytest extends LinearOpMode
{
    final int TAG_ID_TO_MATCH = 20;
    final double STOP_DISTANCE_FROM_TAG_IN_INCHES = 12.0;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;


    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTagProcessor = null;
    private AprilTagDetection detectedTagInfo = null;

    @Override
    public void runOpMode()
    {
        boolean matchingTagFound = false;

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "PutWebcamNameHere"))
                .addProcessor(aprilTagProcessor)
                .build();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // If having issues detecting tag, experiment with adjusting exposure time to reduce motion blur like this example
        //setManualExposure(6, 250);

        telemetry.addData(">", "Press START to start begin detecting");
        telemetry.update();
        waitForStart();

        // Now that initialization is done, start looping through the actual code
        while (opModeIsActive())
        {
            matchingTagFound = false; // When starting each loop, act as if no tag has been found yet as a reset
            detectedTagInfo  = null; // Clear out any information from previously detected tags

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if it was able to successfully extract information from the tag
                if (detection.metadata != null) {
                    //  Check to see if this tag matches the ID of the tag we want to match to
                    if (detection.id == TAG_ID_TO_MATCH) {
                        matchingTagFound = true;
                        detectedTagInfo = detection;
                        break;  // If the matching tag was found, exit the loop
                    }
                }
            }

            // Write the tag info to the screen so we can confirm it worked
            if (matchingTagFound) {
                telemetry.addData("Found", "ID %d (%s)", detectedTagInfo.id, detectedTagInfo.metadata.name);
                telemetry.addData("Range", "%5.1f inches", detectedTagInfo.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", detectedTagInfo.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", detectedTagInfo.ftcPose.yaw);
                telemetry.update();

                // Now, we can use this information to drive to the target tag.  Need to figure out this piece once the above
                // is working correctly.
                // moveRobotToAprilTag();
            }

            sleep(10);
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}