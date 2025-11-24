/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Hardware config:
 *      imu on control Hub:
 *          "imu"
 *
 *      Four drive motors:
 *          "FrontLeft"
 *          "BackLeft"
 *          "BackRight"
 *          "FrontRight"
 *
 *      Tow arm, wrist motors:
 *          "Arm"
 *          "Wrist"
 *
 *      Tow servo motors:
 *          "Knuckle"
 *          "Finger"
 */

@TeleOp(name="Teleop - 2026", group="Concept")
public class Teleop2026 extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    private MecanumDrive drive;
    // initialize limelight
    private Colored patternDetector;
    //claw and arm unit
    private intakeUnit2026 motors;
    private GamePadButtons2026 gpButtons;

    double[] patternPos;
    boolean farShoot = false;
    boolean debugFlag = true;
    int leftOrRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("param.currentPose heading = %s", Params.currentPose.heading.log());

        patternDetector = new Colored(hardwareMap);
        drive = new MecanumDrive(hardwareMap, Params.currentPose);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");
        gpButtons = new GamePadButtons2026();
        leftOrRight = Params.blueOrRed;

        //set shoot position
        Vector2d shootPosNear; // where the robot should shoot
        double shootHeading; //the direction the robot shoot in
        double shootPosX = 1 * Params.HALF_MAT;
        double shootPosY = leftOrRight * Params.HALF_MAT;
        // made the following polarity change to shootHeading calculation
        shootHeading = Math.toRadians(180.0) + Math.atan2(leftOrRight * (6 * Params.HALF_MAT - Math.abs(shootPosY)), 6 * Params.HALF_MAT - shootPosX);
        shootPosNear = new Vector2d(shootPosX, shootPosY);

        // far shoot position
        Pose2d shootPosFar = new Pose2d(
                - 4.5 * Params.HALF_MAT,
                leftOrRight * Params.HALF_MAT,
                Math.toRadians(motors.launchDegreeFar * leftOrRight));

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //preset positions used for teleop commands
        while (!isStarted()) {
            patternPos = patternDetector.returnPosition();
            if (patternPos.length >= 2) {
                for (int i = 0; i < patternPos.length; i++) {
                    telemetry.addData("Pattern ", " Pos[%d] = %.5f", i, patternPos[i]);
                }
            }
            else
            {
                telemetry.addData("no pattern detected", 0);
            }
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start.");

        telemetry.update();
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            // drive speed control
            double maxDrivePower;
            if (gpButtons.speedUp) {
                maxDrivePower = Params.POWER_HIGH;
            } else if (gpButtons.speedDown) {
                maxDrivePower = Params.POWER_LOW;
            } else {
                maxDrivePower = Params.POWER_NORMAL;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * maxDrivePower,
                            -gpButtons.robotStrafe * maxDrivePower
                    ),
                    -gpButtons.robotTurn * maxDrivePower
            ));
            drive.updatePoseEstimate(); // update dire pose.

            // launch actions
            if (gpButtons.launch) {
                motors.startLaunchNear();
            }
            if (gpButtons.launchOff) {
                motors.stopLauncher();
            }

            // shoot one out
            if (gpButtons.launchOneNear) {
                farShoot = false;
                int rampUpTime = 700;
                int waitTimeForTriggerClose = 1000;
                double launchVelocity = motors.launchSpeedNear;
                motors.startLaunchNear();
                motors.startIntake();
                reachTargetVelocity(launchVelocity, rampUpTime);
                motors.triggerOpen(); // shoot first
                checkingVelocityRampDown(waitTimeForTriggerClose);
                motors.triggerClose();
            }

            // shoot one out for Far launching
            if (gpButtons.launchOneFar) {
                farShoot = true;
                int rampUpTime = 700;
                int waitTimeForTriggerClose = 1000;
                double launchVelocity = motors.launchSpeedFar;
                motors.setLauncherVelocity(launchVelocity);
                motors.startIntake();
                reachTargetVelocity(launchVelocity, rampUpTime);
                motors.triggerOpen(); // shoot first
                checkingVelocityRampDown(waitTimeForTriggerClose);
                motors.triggerClose();
            }

            // dump one out in case there are 4 artifacts on robot
            if (gpButtons.dumpOne) {
                int rampUpTime = 400;
                int waitTimeForTriggerClose = 400;
                double launchVelocity = motors.launchSpeedDump;
                motors.setLauncherVelocity(launchVelocity);
                reachTargetVelocity(launchVelocity, rampUpTime);
                motors.triggerOpen(); // shoot first
                checkingVelocityRampDown(waitTimeForTriggerClose);
                motors.triggerClose();
            }

            // intake actions
            if (gpButtons.intakeOn) {
                motors.startIntake();
            }

            if (gpButtons.intakeOff) {
                motors.stopIntake();
            }

            // trigger servo actions
            if (gpButtons.triggerOpen) {
                motors.triggerOpen();
            }

            if (gpButtons.triggerClose) {
                motors.triggerClose();
            }

            // near shooting
            if (gpButtons.launchArtifacts) {
                farShoot =false;
                shootArtifacts(motors.launchSpeedNear);
            }

            // far shooting
            if (gpButtons.launchArtifactsFar) {
                farShoot = true;
                shootArtifacts(motors.launchSpeedNear);
            }

            // move to far shoot position by RR
            if (gpButtons.alignShootPosFar) {
                farShoot = true;

                Pose2d locP = drive.localizer.getPose();
                double correctionHeading = Math.toDegrees(locP.heading.toDouble() - shootPosFar.heading.toDouble());
                double movingDistance = Math.abs(locP.position.x - shootPosFar.position.x) + Math.abs(locP.position.y - shootPosFar.position.y);

                // make sure there is significant difference moving by roadrunner
                if ((Math.abs(correctionHeading) > 2.0) || (movingDistance > 2.0)) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.localizer.getPose())
                                    .strafeToLinearHeading(shootPosFar.position, shootPosFar.heading)
                                    .build()
                    );
                }
            }

            // move to near shoot position by RR
            if (gpButtons.alignShootPosNear) {
                farShoot = false;
                Pose2d locP = drive.localizer.getPose();
                double correctionHeading = Math.toDegrees(locP.heading.toDouble() - shootHeading);
                double movingDistance = Math.abs(locP.position.x - shootPosNear.x) + Math.abs(locP.position.y - shootPosNear.y);

                // make sure there is significant difference moving by roadrunner
                if ((Math.abs(correctionHeading) > 1.0) || (movingDistance > 1.0)) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.localizer.getPose())
                                    .strafeToLinearHeading(shootPosNear, shootHeading)
                                    .build()
                    );
                }
            }

            if (gpButtons.resetShootPos) {
                if (farShoot) {
                    shootPosFar = drive.localizer.getPose();
                }
                else
                {
                    shootPosNear = drive.localizer.getPose().position;
                    shootHeading = drive.localizer.getPose().heading.toDouble();
                }

                // Move the robot a little bit to tell driver that the shoot Position has been updated.
                Pose2d locP = drive.localizer.getPose();
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeTo(new Vector2d(locP.position.x + 1.0, locP.position.y + 1.0))
                                .build()
                );
            }

            // TODO : not implemented correctly yet
//            if(gpButtons.autoLaunchPos) {
//                // move angle to launch using limelight return position (which returns degrees x and degrees y of the pattern)
//                // telemetry
//                double[] patternPos = patternDetector.returnPosition();
//                if (patternPos.length >= 2) {
//                    double angleToTurn = patternPos[0];
////                    Pose3D botpose = patternDetector.returnPositionbeta();
////                    Actions.runBlocking(
////                            drive.actionBuilder(new Pose2d(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw()))
////                                    .strafeToLinearHeading(new Vector2d(1*Params.HALF_MAT, 1*Params.HALF_MAT), Math.toRadians(135))
////                                    .build()
////                    );
//                    if (angleToTurn > 0) {
//                        Actions.runBlocking(
//                                drive.actionBuilder(new Pose2d(0,0,0))
//                                        .strafeToLinearHeading(new Vector2d(0,0.00000000000000000000000001), 3.141592653589 * angleToTurn / 180)
//                                        .build()
//                        );
//                    } else if (angleToTurn < 0) {
//                        Actions.runBlocking(
//                                drive.actionBuilder(new Pose2d(0,0,0))
//                                        .strafeToLinearHeading(new Vector2d(0,0.00000000000000000000000001), -3.141592653589 * Math.abs(angleToTurn) / 180)
//                                        .build()
//                        );
//                    }
//                }
//            }

            if (debugFlag) {

                // display trigger servo position for testing purpose.
                telemetry.addData("trigger servo", "position = %.3f", motors.getTriggerPosition());
                telemetry.addData("launcher motor", "power = %.3f", motors.getLauncherPower());
                telemetry.addData("launcher motor", "velocity = %.3f", motors.getLaunchVelocity());

                patternPos = patternDetector.returnPosition();
                if (patternPos.length >= 2) {
                    for (int i = 0; i < patternPos.length; i++) {
                        telemetry.addData("Pattern ", " Pos[%d] = %.5f", i, patternPos[i]);
                    }
                }

                telemetry.addData("heading", " %.3f", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
                telemetry.addData("location", " %s", drive.localizer.getPose().position.toString());
                telemetry.addData(" --- ", " --- ");
                telemetry.update(); // update message at the end of while loop

            }

            // save the current position in a static variable.
            Params.currentPose = drive.localizer.getPose();
        }

        // The motor stop on their own but power is still applied. Turn off motor.
    }

    public void shootArtifacts(double launchV) {
        int waitTimeForTriggerClose = 1000;
        int waitTimeForTriggerOpen = 600;
        int rampUpTime = 800;
        double launchVelocity = launchV;

        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < launchVelocity * 0.96) {
            motors.setLauncherVelocity(launchVelocity);
        }

        // correct heading according to limelight feedback
        if (launchV == motors.launchSpeedNear) {
            //  it is near launching
            nearAprilTagTracking();
        }
        else {
            //  it is far launching
            farAprilTagTracking();
        }

        // waiting motor speed to ramp up
        reachTargetVelocity(launchVelocity, rampUpTime);
        motors.triggerOpen(); // shoot first
        checkingVelocityRampDown(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        // start shooting 2nd one
        launchVelocity -= 6; // reduce a little bit.
        sleepWithDriving(200);
        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        reachTargetVelocity(launchVelocity, waitTimeForTriggerOpen);// waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        checkingVelocityRampDown(waitTimeForTriggerClose);
        motors.triggerClose();

        // start shooting 3rd one
        launchVelocity -= 4; // reduce a little bit.
        reachTargetVelocity(launchVelocity, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen();  // shoot third
        checkingVelocityRampDown(waitTimeForTriggerClose);

        motors.triggerClose();
        motors.stopIntake();
        motors.stopLauncher();
    }

    // sleep for other motors than driving motors
    private void sleepWithDriving(int msecond) {
        double startTime = runtime.milliseconds();
        while ((runtime.milliseconds() - startTime) < msecond) {
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * Params.POWER_LOW / 2.0,
                            -gpButtons.robotStrafe * Params.POWER_LOW / 2.0
                    ),
                    -gpButtons.robotTurn * Params.POWER_LOW / 2.0
            ));
            drive.updatePoseEstimate();
        }
    }

    private void checkingVelocityRampDown(int msecond) {
        double startTime = runtime.milliseconds();

        boolean artifactReached = false;
        double stableVelocity;
        stableVelocity = motors.launcherAverageVelocity(20);

        while (!artifactReached && ((runtime.milliseconds() - startTime) < msecond)) {
            double currentVel = motors.launcherAverageVelocity(20);
            artifactReached = (currentVel < stableVelocity * 0.87); // when speed reduced to 87%

            gpButtons.checkGamepadButtons(gamepad1, gamepad2);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * Params.POWER_LOW / 1.5,
                            -gpButtons.robotStrafe * Params.POWER_LOW / 1.5
                    ),
                    -gpButtons.robotTurn * Params.POWER_LOW / 1.5
            ));
            drive.updatePoseEstimate();
        }
    }

    private void reachTargetVelocity(double targetVel, int msec) {
        double startTime = runtime.milliseconds();
        boolean rampedUp = false;
        motors.setLauncherVelocity(targetVel); // update target velocity

        while (!rampedUp && ((runtime.milliseconds() - startTime) < msec)) {
            double currentVel = motors.launcherAverageVelocity(20);
            rampedUp = (currentVel > targetVel * 0.90); // when speed reduced to 85%
            Logging.log("launcher motor average velocity : %.1f.", currentVel);
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * Params.POWER_LOW / 1.5,
                            -gpButtons.robotStrafe * Params.POWER_LOW / 1.5
                    ),
                    -gpButtons.robotTurn * Params.POWER_LOW / 1.5
            ));
            drive.updatePoseEstimate();
        }
        Logging.log("Total waiting duration = %.2f", runtime.milliseconds() - startTime);
    }

    private void detectPattern() {
        patternPos = patternDetector.returnPosition();
        for (int i = 0; i < 30; i++) { // check for 30 cycles (~30 milliseconds) to detect pattern
            patternPos = patternDetector.returnPosition();
            Logging.log("pattern  = %f", patternPos[0]);
            //telemetry.addData("limelight", "Detected Pattern = %f", detectedPattern);
            //telemetry.update();
            if ((patternPos.length > 1) && (Math.abs(patternPos[0]) > 0)) {
                Logging.log("pattern  = %f", patternPos[0]);
                return; // return when detected.
            }
            sleep(1);
        }
    }

    private void nearAprilTagTracking() {
        detectPattern();
        // make sure the april Tag has been detected and need to turn. Bigger than 1 degree
        // moving according to area if pattern has detected.
        if ((patternPos.length > 2) && (Math.abs(patternPos[2]) > 0.001)) {
            double distance = 3000 * (0.013 - patternPos[2]);
            Vector2d locP1 = drive.localizer.getPose().position;
            double headingAng = drive.localizer.getPose().heading.toDouble();
            double newPosX = locP1.x - distance * Math.cos(headingAng);
            double newPosY = locP1.y - distance * Math.sin(headingAng);

            // moving robot when there is significant difference.
            if ((Math.abs(distance) > 2.0 /* inch */) || (Math.abs(patternPos[0]) > 2.0 /* degree*/)) {
                // slow down for more accurate position
                VelConstraint correctSpeed = (robotPose, _path, _disp) -> 30.0;

                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(
                                        new Vector2d(newPosX, newPosY),
                                        headingAng + Math.toRadians(-patternPos[0]), correctSpeed)
                                .build()
                );
            }
        }
    }

    private void farAprilTagTracking() {
        detectPattern();
        // make sure the april Tag has been detected and need to turn. Bigger than 1 degree
        // moving according to area if pattern has detected.
        if ((patternPos.length > 2) && (Math.abs(patternPos[0]) > 0.01)) {

            double correctAng = patternPos[0] - leftOrRight * 1.0; // 1.0 degree to left.
            // moving robot when there is significant difference.
            if (Math.abs(correctAng) > 0.5 /* degree*/)
            {
                TurnConstraints tc = new TurnConstraints(Math.PI/2, -Math.PI/2, Math.PI/2);
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .turn(-Math.toRadians(correctAng), tc)
                                .build()
                );
            }
        }

        // correct two times for more accurate.
        detectPattern();
        // run angle adjustment a second time
        // make sure the april Tag has been detected and need to turn. Bigger than 1 degree
        // moving according to area if pattern has detected.
        if ((patternPos.length > 2) && (Math.abs(patternPos[0]) > 0.01)) {

            double correctAng = patternPos[0] - leftOrRight * 1.0; // 1.0 degree to left.
            // moving robot when there is significant difference.
            if (Math.abs(correctAng) > 0.5 /* degree*/)
            {
                TurnConstraints tc = new TurnConstraints(Math.PI/3, -Math.PI/3, Math.PI/3);
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .turn(-Math.toRadians(correctAng), tc)
                                .build()
                );
            }
        }
    }
}
