/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * This is NOT an opmode.
 *
 * This class defines the parameters related with game field elements.
 */
public class Params {
    // road runner
    static int blueOrRed = 1;
    static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(180.0));

    static boolean armCalibrated = false;

    //distances for dist sensor
    static double HIGH_CHAMBER_DIST = 6.2; // back distance sensor to chamber when hanging specimen
    static double SPECIMEN_PICKUP_DIST = 14.3; //14.5; // front distance sensor to wall when pickup specimen

    static int NO_CATION = 999999;
    //game field parameters
    static final double HALF_MAT = 23.75/2.0;  // 60 cm. 11.8inch

    // robot size
    static final double CHASSIS_HALF_WIDTH = 15.0 / 2.0;
    static final double CHASSIS_LENGTH = 15.5;
    static final double CHASSIS_HALF_LENGTH = CHASSIS_LENGTH / 2.0;
    static final double TELEOP_DISTANCE_TO_TAG = 7.0;

    // chassis power factors
    static final double POWER_LOW = 0.3;
    static final double POWER_NORMAL = 0.80;
    static final double POWER_HIGH = 1.0;

    // X positions for pickup and hang specimen
    static double pickupSpecimenLineupX = - 3.85 * HALF_MAT;
    static double pickupSpecimenX = pickupSpecimenLineupX - 0.2 * HALF_MAT;
    static double hangingSpecimenX = -3.2 * HALF_MAT; // it will be updated during autonomous

    static boolean imuReseted = false;
}