/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Autonomous  for only vision detection using OpenCV VisionPortal and park
 */
@Autonomous(name = "Auton-Right", group = "00-Autonomous", preselectTeleOp = "TeleOpPS5")
public class AUTON_RIGHT extends LinearOpMode {

    public static String TEAM_NAME = "Tx-Rx"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 21386; //TODO: Enter team Number

    private Servo specimen;
    private DcMotor Lift;
    private int SPECIMEN_LIFT = 2000;
    private double OPEN_SPECIMEN_CLAW = 0.5;
    private double CLOSE_SPECIMEN_CLAW = 0.8;
    private double liftPow = 0.5;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        //Servo: Specimen claw
        specimen = hardwareMap.get(Servo.class, "specimen");

        //Lift: Initialize lift
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotor.Direction.REVERSE);

        //Key Pay inputs to selecting Starting Position of robot

        telemetry.addData("Selected Starting Position", startPosition);
        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        //For Robot-centric ts
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d specimenDropPose = new Pose2d(0,0,0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        //initPose -> midwayPose 1 -> specimenDropPose -> midwayPose2 -> parkPose
        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        midwayPose1 = new Pose2d(20, 5, Math.toRadians(0));
        specimenDropPose = new Pose2d(30,10,0); //changed from 28 to 30
        midwayPose2 = new Pose2d(10, -18, Math.toRadians(45));
        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
        parkPose = new Pose2d(5, -50, Math.toRadians(0));  //changed from 90 to 0 to face forward

        drive = new MecanumDrive(hardwareMap, initPose);

        //Start with the Specimen claw closed
        specimen.setPosition(CLOSE_SPECIMEN_CLAW);

        //Move to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());
        
        safeWaitSeconds((0.5));

        //TODO : Code to raise slide and drop specimen
        Lift.setTargetPosition(2000);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        while (Lift.isBusy()) {
            telemetry.addData("Current Position", Lift.getCurrentPosition());
            telemetry.addData("Target Position", Lift.getTargetPosition());
            telemetry.update();
        }
        safeWaitSeconds(0.5);
        //Lift.setPower(0);

        //Move to specimenDropPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenDropPose.position, specimenDropPose.heading)
                        .build());

        safeWaitSeconds(0.5);

        //Lower lift
        Lift.setTargetPosition(1800);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.1);
        while (Lift.isBusy()) {
            telemetry.addData("Current Position", Lift.getCurrentPosition());
            telemetry.addData("Target Position", Lift.getTargetPosition());
            telemetry.update();
        }
        safeWaitSeconds(0.5);

        //Lower lift
        Lift.setTargetPosition(1500);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.1);
        while (Lift.isBusy()) {
            telemetry.addData("Current Position", Lift.getCurrentPosition());
            telemetry.addData("Target Position", Lift.getTargetPosition());
            telemetry.update();
        }
        safeWaitSeconds(0.5);

        specimen.setPosition(OPEN_SPECIMEN_CLAW);

        safeWaitSeconds(0.5);


        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .build());


        safeWaitSeconds(0.5);

        //Bring the lift down
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        while (Lift.isBusy()) {
            telemetry.addData("Current Position", Lift.getCurrentPosition());
            telemetry.addData("Target Position", Lift.getTargetPosition());
            telemetry.update();
        }
        safeWaitSeconds(0.5);
        //close claw - or leave it open
        //specimen.setPosition(CLOSE_SPECIMEN_CLAW);

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //TODO: after specimen go here
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());


    } // runAutonoumousMode

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

  }