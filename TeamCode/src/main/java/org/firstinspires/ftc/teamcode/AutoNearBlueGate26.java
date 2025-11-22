package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Near Blue Gate Pos45", group = "Concept")
@Disabled
public class AutoNearBlueGate26 extends LinearOpMode {
    // get the software-hardware links ready
    private final ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private Colored patternDetector;
    public int leftOrRight; // 1 for blue, -1 for red

    private int[] pickupOrder = {1, 2, 3}; // 1: the artifacts row closest to the shooting target

    public void setSide() {
        leftOrRight = 1;
    }
    private double detectedPattern = 23; // limelight detected pattern

    private Vector2d shootPos; // where the robot should shoot
    private double shootHeading; //the direction the robot shoot in

    public Pose2d startPose;

    public void setStartPosition() {
        startPose = new Pose2d(
                (4.76 * Params.HALF_MAT), // add additional 0.5 inch according to testing
                (leftOrRight * 3.9 * Params.HALF_MAT), Math.toRadians(180.0 + leftOrRight * 45.0)
        );
    }

    @Override
    public void runOpMode() {
        setSide();
        setStartPosition();
        Params.leftOrRight = leftOrRight;

        // connect the hardware map to color discrimination system and prepare launcher, intake, and trigger
        patternDetector = new Colored(hardwareMap);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        Params.currentPose = startPose; // save the Position

        // define shoot position at (HALF_MAT, HALF_MAT), (HALF_MAT, -HALF_MAT), calculate shoot angle based on location x, y
        double shootPosX = 1 * Params.HALF_MAT + 2.0;
        double shootPosY = leftOrRight * (Params.HALF_MAT + 2.0);
        // made the following polarity change to shootHeading calculation
        shootHeading = Math.toRadians(180.0) + Math.atan2(leftOrRight * (6 * Params.HALF_MAT - Math.abs(shootPosY)), 6 * Params.HALF_MAT - shootPosX);
        shootPos = new Vector2d(shootPosX, shootPosY);

        // set up driving system
        drive = new MecanumDrive(hardwareMap, startPose);

        // init position of trigger
        motors.triggerClose();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            run_auto();
            Params.currentPose = drive.localizer.getPose(); // save current position
        }
    }

    private void run_auto() {

        // Run the first leg of the path: move to shooting position while detecting pattern
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                //.afterDisp(3 * Params.HALF_MAT, new limeLightCamera()) // start limelight detection after moving 3 half mats
                //.strafeToConstantHeading(shootPos) // move to shooting position
                .afterTime(0.5, new startLauncherAction()) // start launcher motor
                //.turnTo(shootHeading) // turn to shooting direction
                .strafeToLinearHeading(shootPos, shootHeading)
                .build());
        Params.currentPose = drive.localizer.getPose(); // save current position

        // shoot preload artifacts
        shootArtifacts();

        // the following is a velocity constraint for moving to pick up artifacts
        VelConstraint pickupSpeed = (robotPose, _path, _disp) -> 9.0;

        //setupPickupOrder((int)detectedPattern);

        // Loop to go through all 3 rows to pick up artifacts and shoot them
        for (int pickupIndex = 0; pickupIndex < 3; pickupIndex++) {
            Params.currentPose = drive.localizer.getPose(); // save current position

            Vector2d pickupPos;
            Vector2d pickupEndPos;
            // 23 is the closest row to start position, then 22, then 21, so new if statement below will optimize pathing
            int rowNum = pickupOrder[pickupIndex];

            pickupPos = rowChoose(rowNum);
            // fixed polarity below (there was a double negative sign before)
            pickupEndPos = new Vector2d(pickupPos.x, pickupPos.y + 17.0 * leftOrRight);

            // action for picking up artifacts
            Action actMoveToPickup = drive.actionBuilder(drive.localizer.getPose())
                    // add 4 degree more for row1 according to test results
                    .strafeToLinearHeading(pickupPos, Math.toRadians(90.0 * leftOrRight))
                    .build();
            Actions.runBlocking(actMoveToPickup); // ready for pickup artifacts
            Params.currentPose = drive.localizer.getPose(); // save current position

            // close launching trigger before pickup artifacts
            motors.stopLauncher();

            // starting intake motor
            motors.startIntake();
            Action actIntake = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToConstantHeading(pickupEndPos, pickupSpeed) // picking up artifacts
                    .build();
            Actions.runBlocking(actIntake); // complete pickup artifacts
            Params.currentPose = drive.localizer.getPose(); // save current position
            motors.stopIntake();

            // open gate after pickup first row of artifacts
            if (pickupIndex == 0) {
                Vector2d gatePose1 = new Vector2d(5.0, drive.localizer.getPose().position.y);
                Vector2d gatePose2 = new Vector2d(gatePose1.x, gatePose1.y + leftOrRight * 9.0);
                Action openGateAct = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(gatePose1) // move to the gate
                        .strafeToConstantHeading(gatePose2) // open the gate
                        .build();
                Actions.runBlocking(openGateAct); // complete pickup artifacts
                Params.currentPose = drive.localizer.getPose(); // save current position

                sleep(1300); // let artifacts get off
            }

            // only need to go back a little bit for row 2nd and 3rd
            if ((pickupIndex < 2) && (rowNum > pickupOrder[pickupIndex + 1])) {
                // after pickup, need to go back a bit to avoid obstacles from other rows
                Action actMoveBack;
                actMoveBack = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(pickupPos)
                        .build();
                Actions.runBlocking(actMoveBack);
                Params.currentPose = drive.localizer.getPose(); // save current position

            }

            Action actMoveToLaunch = drive.actionBuilder(drive.localizer.getPose())
                    .afterTime(0.3, new startLauncherAction()) // start launcher motor
                    .strafeToLinearHeading(shootPos, shootHeading)
                    .build();
            Actions.runBlocking(actMoveToLaunch);
            Params.currentPose = drive.localizer.getPose(); // save current position


            // shoot picked up artifacts
            shootArtifacts();
        }

        // stop launcher before parking
        motors.stopLauncher();

        // move out of the Triangle
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(0, leftOrRight*3.8*Params.HALF_MAT), Math.toRadians(180))
                .build());
        Params.currentPose = drive.localizer.getPose(); // save current position

    }

    // function to shoot 3 artifacts
    private void shootArtifacts() {
        int waitTimeForTriggerClose = 1000;
        int waitTimeForTriggerOpen = 700;
        int rampUpTime = 1000;
        double targetV = motors.launchSpeedNear;

        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.4) {
            Logging.log("start launcher motor since it is stopped.");
            motors.startLaunchNear();
            reachTargetVelocity(targetV, rampUpTime); // waiting time for launcher motor ramp up
        }

        motors.triggerOpen(); // shoot first
        Logging.log("launcher velocity for #1 one: %.1f.", motors.getLaunchVelocity());
        velocityRampDown(targetV, waitTimeForTriggerClose);
        Logging.log("starting close trigger.");
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching


        // starting shoot 2nd one
        targetV -= 6;
        sleep(200);
        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        reachTargetVelocity(targetV, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        Logging.log("launcher velocity for #2 one: %.2f.", motors.getLaunchVelocity());
        
        velocityRampDown(targetV, waitTimeForTriggerClose);
        motors.triggerClose();

        // starting shoot third one
        targetV -= 4;
        reachTargetVelocity(targetV, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot third
        Logging.log("launcher velocity for #3 one: %f.", motors.getLaunchVelocity());
        velocityRampDown(targetV, waitTimeForTriggerClose + waitTimeForTriggerOpen); // waiting more time for the third one.
    }

    // function that chooses the right row based on detected pattern, returns a Vector2d
    private Vector2d rowChoose(double rownumber) {
        return new Vector2d(
                (-rownumber * 2 + 3) * Params.HALF_MAT, // add additional inch according to testing.
                leftOrRight * (2.6 * Params.HALF_MAT)
        );
    }

    // limelight detection action
    private class limeLightCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (int i = 0; i < 30; i++) { // check for 30 cycles (~30 milliseconds) to detect pattern
                detectedPattern = patternDetector.returnId(false);
                Logging.log("pattern  = %f", detectedPattern);
                telemetry.addData("limelight", "Detected Pattern = %f", detectedPattern);
                telemetry.update();
                if ((detectedPattern > 20) && (detectedPattern < 24)) {
                    return false;
                }
                sleep(1);
            }
            detectedPattern = 23; // default pattern if limelight can't detect
            patternDetector.limelight.close(); //close limelight for safe wifi connection
            return false;
        }
    }

    // action to start launcher motor
    private class startLauncherAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start launcher motor.");
            motors.startLaunchNear();
            return false;
        }
    }

    private void setupPickupOrder(int pt) {
        switch (pt) {
            case 21:
                pickupOrder = new int[]{3, 1, 2};
                break;
            case 22:
                pickupOrder = new int[]{2, 1, 3};
                break;
            case 23:
            default:
                pickupOrder = new int[]{1, 2, 3};
                //pickupOrder = new int[]{3, 1, 2};
                break;
        }
    }

    private void recordVelocity(int msec) {
        double startTime = runtime.milliseconds();
        while ((runtime.milliseconds() - startTime) < msec) {
            double speed = motors.getLaunchVelocity();
            Logging.log("launcher motor velocity : %.1f.", speed);
        }
    }

    private void velocityRampDown(double shootSpeed, int msec) {
        double startTime = runtime.milliseconds();
        boolean artifactReached = false;

        while (!artifactReached && ((runtime.milliseconds() - startTime) < msec)) {
            double currentVel = motors.launcherAverageVelocity(12);
            artifactReached = (currentVel < shootSpeed * 0.87); // when speed reduced to 87%
        }
        Logging.log(" ** Total ramp down duration = %.2f", runtime.milliseconds() - startTime);
    }

    private void reachTargetVelocity(double targetVel, int msec) {
        double startTime = runtime.milliseconds();
        boolean rampedUp = false;
        motors.setLauncherVelocity(targetVel); // update target velocity
        while (!rampedUp && ((runtime.milliseconds() - startTime) < msec)) {
            double currentVel = motors.launcherAverageVelocity(12);
            rampedUp = (currentVel > targetVel * 0.95); // when speed reduced to 85%
        }
        Logging.log(" #### Total ramp up duration = %.2f", runtime.milliseconds() - startTime);
    }

    // For future use when we have an artifact sorter mechanism
    //    private void sortArtifacts(int pattern) {
    //        switch (pattern) {
    //            case 21:
    //                telemetry.addLine("Sorting: GPP");
    //                break;
    //            case 22:
    //                telemetry.addLine("Sorting: PGP");
    //                break;
    //            case 23:
    //                telemetry.addLine("Sorting: PPG");
    //                break;
    //            default:
    //                telemetry.addLine("Sorting: Unknown");
    //                break;
    //        }
    //        telemetry.update();
    //    }
}