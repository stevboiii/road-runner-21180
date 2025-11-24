package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Autonomous26 extends LinearOpMode {

    // Variables which need to be updated for different cases.
    private int blueOrRed = 1; // 1 for blue, -1 for red
    private boolean nearOrFar = true; // true for near, false for far
    private int gateOpenTimes = 1; // how many times to open the gate during autonomous
    private int pickupTimes = 3; // pickup times during autonomous


    // get the software-hardware links ready
    private final ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private Colored patternDetector;
    private double detectedPattern = 23; // limelight detected pattern
    private int[] pickupOrder = {1, 2, 3}; // 1: the artifacts row closest to the shooting target

    private Pose2d startPose;
    private Pose2d launchPose;

    // launching positions and speed
    private Pose2d pickupPose;
    private Pose2d pickupEndPose;
    private double launchVelocity;

    // parking position
    private Pose2d parkingPose;

    @Override
    public void runOpMode() {
        // setup game options
        blueOrRed = setSide();
        nearOrFar = setNearOrFar();
        pickupTimes = setPickupTimes();
        gateOpenTimes = setGateOpenTimes();
        if (blueOrRed != -1) {
            blueOrRed = 1; // treat all other values as 1 except -1.
        }

        // save options in static variable for Teleop.
        Params.blueOrRed = blueOrRed;
        Params.currentPose = startPose; // save the Position

        // Create hardware objects
        patternDetector = new Colored(hardwareMap);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        // init positions according to game options before initial MecanumDrive class.
        setStartAndShootPose(blueOrRed, nearOrFar);

        drive = new MecanumDrive(hardwareMap, startPose);

        // init position of trigger
        motors.triggerClose();
        launchVelocity = nearOrFar? motors.launchSpeedNear : motors.launchSpeedFar;

        // waiting until press the button to start autonomous.
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
                //.strafeToConstantHeading(launchPose.position) // move to shooting position
                .afterTime(0.5, new startLauncherAction()) // start launcher motor
                //.turnTo(launchPose.heading) // turn to shooting direction
                .strafeToLinearHeading(launchPose.position, launchPose.heading)
                .build());
        Params.currentPose = drive.localizer.getPose(); // save current position

        // shoot preload artifacts
        shootArtifacts(launchVelocity);

        // the following is a velocity constraint for moving to pick up artifacts
        VelConstraint pickupSpeed = (robotPose, _path, _disp) -> 9.0;

        //setupPickupOrder((int)detectedPattern);

        // Loop to go through all 3 rows to pick up artifacts and shoot them
        for (int pickupIndex = 0; pickupIndex < pickupTimes; pickupIndex++) {
            Params.currentPose = drive.localizer.getPose(); // save current position

            // set up artifacts pickup start and end positions

            // 23 is the closest row to start position, then 22, then 21, so new if statement below will optimize pathing
            int rowNum = 1; //pickupOrder[pickupIndex]; // not used pattern during autonomous any more
            setupPickupPositions(blueOrRed, nearOrFar, pickupIndex);

            // action for picking up artifacts
            Action actMoveToPickup = drive.actionBuilder(drive.localizer.getPose())
                    // add 4 degree more for row1 according to test results
                    .strafeToLinearHeading(pickupPose.position, pickupPose.heading.toDouble())
                    .build();
            Actions.runBlocking(actMoveToPickup); // ready for pickup artifacts
            Params.currentPose = drive.localizer.getPose(); // save current position

            // close launching trigger before pickup artifacts
            motors.stopLauncher();

            // starting intake motor
            motors.startIntake();

            // moving to pickup end position during intake
            Action actIntake = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToConstantHeading(pickupEndPose.position, pickupSpeed) // picking up artifacts
                    .build();
            Actions.runBlocking(actIntake); // complete pickup artifacts
            Params.currentPose = drive.localizer.getPose(); // save current position

            if (nearOrFar) {
                // only stop intake for near auto cases
                motors.stopIntake();
            }

            // open gate after pickup artifacts only for near autonomous case
            if ((pickupIndex < gateOpenTimes) && nearOrFar) {
                Vector2d gatePose1 = new Vector2d(5.0, drive.localizer.getPose().position.y);
                Vector2d gatePose2 = new Vector2d(gatePose1.x, gatePose1.y + blueOrRed * 9.0);
                Action openGateAct = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(gatePose1) // move to the gate
                        .strafeToConstantHeading(gatePose2) // open the gate
                        .build();
                Actions.runBlocking(openGateAct); // complete pickup artifacts
                Params.currentPose = drive.localizer.getPose(); // save current position
                sleep(1300); // let artifacts get off enough for 6 balls rolling out
            }

            // only need to go back a little bit for row 2nd and 3rd if the 1st row is still on mat.
            if ((pickupIndex < 2) && (rowNum > pickupOrder[pickupIndex + 1])) {
                // after pickup, need to go back a bit to avoid obstacles from other rows
                Action actMoveBack;
                actMoveBack = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(pickupPose.position)
                        .build();
                Actions.runBlocking(actMoveBack);
                Params.currentPose = drive.localizer.getPose(); // save current position
            }

            // moving to launching position
            Action actMoveToLaunch = drive.actionBuilder(drive.localizer.getPose())
                    .afterTime(0.3, new startLauncherAction()) // start launcher motor
                    .strafeToLinearHeading(launchPose.position, launchPose.heading)
                    .build();
            Actions.runBlocking(actMoveToLaunch);
            Params.currentPose = drive.localizer.getPose(); // save current position

            // start to launch artifacts
            shootArtifacts(launchVelocity);
        }
        // stop launcher before parking
        motors.stopLauncher();

        // move out of the Triangle and ready for open the gate at the start of Teleop
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(parkingPose.position, parkingPose.heading)
                .build());
        Params.currentPose = drive.localizer.getPose(); // save current position
    }

    // function to shoot 3 artifacts
    private void shootArtifacts(double launchV) {
        int waitTimeForTriggerClose = 500;
        int waitTimeForTriggerOpen = 700;
        int rampUpTime = 1000;
        double targetV = launchV;

        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < targetV * 0.96) {
            Logging.log("start launcher motor since it is stopped.");
            motors.setLauncherVelocity(targetV);
            reachTargetVelocity(targetV, rampUpTime); // waiting time for launcher motor ramp up
        }

        motors.triggerOpen(); // shoot first
        Logging.log("launcher velocity for #1 one: %.1f.", motors.getLaunchVelocity());
        velocityRampDown(targetV, waitTimeForTriggerClose);
        Logging.log("starting close trigger.");
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching


        // starting shoot 2nd one
        targetV -= 6;
        recordVelocity(200); // start intake 200 ms after 1st launching.
        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        reachTargetVelocity(targetV, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        Logging.log("launcher velocity for #2 one: %.2f.", motors.getLaunchVelocity());
        
        velocityRampDown(targetV, waitTimeForTriggerClose);
        motors.triggerClose();

        // starting shoot third one
        targetV -= 2;
        reachTargetVelocity(targetV, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot third
        Logging.log("launcher velocity for #3 one: %f.", motors.getLaunchVelocity());
        velocityRampDown(targetV, waitTimeForTriggerClose); // waiting more time for the third one.
    }

    // function that chooses the right row based on detected pattern, returns a Vector2d
    private Vector2d rowChoose(double rownumber) {
        return new Vector2d(
                (-rownumber * 2 + 3) * Params.HALF_MAT, // add additional inch according to testing.
                blueOrRed * (2.6 * Params.HALF_MAT)
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
            motors.setLauncherVelocity(launchVelocity);
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


    private void setStartAndShootPose(int sideOption, boolean startOption) {
        Pose2d startP;
        Pose2d launchP;
        if (startOption) {
            // near autonomous
            startP = new Pose2d(
                    (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH - 1.5), // add additional 0.5 inch according to testing
                    (sideOption * (4 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH - 1.0)), Math.toRadians(180.0)
            );

            // define shoot position, calculate shoot angle based on location x, y
            double shootPosX = 1 * Params.HALF_MAT + 2.0;
            double shootPosY = sideOption * (Params.HALF_MAT + 2.0);
            // made the following polarity change to shoot Heading calculation
            double shootH = Math.PI +
                    Math.atan2(sideOption * (6 * Params.HALF_MAT - Math.abs(shootPosY)),
                    6 * Params.HALF_MAT - shootPosX);
            launchP = new Pose2d(shootPosX, shootPosY, shootH);

            parkingPose = new Pose2d(0, blueOrRed * 3.8 * Params.HALF_MAT, Math.PI);
        }
        else {
            // far autonomous
            startP = new Pose2d(
                    (-6 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH),
                    (blueOrRed * Params.CHASSIS_HALF_WIDTH),
                    Math.toRadians(-180)
            );

            launchP = new Pose2d(
                    - 4.5 * Params.HALF_MAT,
                    blueOrRed * Params.HALF_MAT,
                    Math.toRadians(motors.launchDegreeFar * blueOrRed));

            parkingPose = new Pose2d(- 5 * Params.HALF_MAT, sideOption * 3 * Params.HALF_MAT, Math.PI);
        }
        startPose = startP;
        launchPose = launchP;
    }

    private void setupPickupPositions(int sideFlag, boolean nearFlag, int index) {
        Pose2d pickupS;
        Pose2d pickupE;
        if (nearFlag) {
            // near autonomous
            pickupS = new Pose2d(
                    (-index * 2 + 1) * Params.HALF_MAT,
                    sideFlag * (2.6 * Params.HALF_MAT),
                    sideFlag * Math.PI / 2.0);
            pickupE = new Pose2d(pickupS.position.x,
                    pickupS.position.y + 17.0 * sideFlag, //moving 17.0 inch in Y direction
                    pickupS.heading.toDouble());
        }
        else {
            // far autonomous
            switch (index) {
                case 0:
                    // pickup from human player station
                    pickupS = new Pose2d(
                            (-4.3) * Params.HALF_MAT,
                            sideFlag * (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH - 1.0),
                            sideFlag * Math.PI * 130.0/180); // 130 degree
                    pickupE = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, // moving 15 inch
                            pickupS.position.y,
                            pickupS.heading.toDouble());
                    break;
                case 1:
                    // only pickup the 3rd row for far auto
                    pickupS = new Pose2d(
                            (-3) * Params.HALF_MAT,
                            sideFlag * (2.6 * Params.HALF_MAT),
                            sideFlag * Math.PI / 2.0);
                    pickupE = new Pose2d(pickupS.position.x,
                            pickupS.position.y + 17.0 * sideFlag, // moving 17.0 inch in Y direction
                            pickupS.heading.toDouble());
                    break;
                case 2:
                case 3:
                default:
                    // pickup rolling down artifacts according to limelight detected positions
                    pickupS = new Pose2d(
                            (-5) * Params.HALF_MAT,
                            sideFlag * (4 * Params.HALF_MAT),
                            sideFlag * Math.PI / 2.0);
                    pickupE = new Pose2d(pickupS.position.x,
                            sideFlag * (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH), //moving to the wall
                            pickupS.heading.toDouble());
                    break;
            }
        }
        pickupPose = pickupS;
        pickupEndPose = pickupE;
    }


    /*
    set blue ot red options. 1 for blue and -1 for red. Other values are all treat as 1(blue).
     */
    public int setSide() {
        return 1;
    }

    public boolean setNearOrFar() {
        return true;
    }

    public int setGateOpenTimes() {
        return 1;
    }

    public int setPickupTimes() {
        return 3;
    }

}