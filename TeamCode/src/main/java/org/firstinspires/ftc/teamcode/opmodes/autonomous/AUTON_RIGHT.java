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
@Autonomous(name = "Auton-Right Meet2", group = "00-Autonomous", preselectTeleOp = "TeleOpPS5")
public class AUTON_RIGHT extends LinearOpMode {

    public static String TEAM_NAME = "Tx-Rx"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 21386; //TODO: Enter team Number

    private Servo specimen;
    private DcMotor Lift;
    private DcMotor Lift2;
    private Servo Rotation;
    private Servo sample;
    private Servo Wrist;
    private int SPECIMEN_LIFT = 2000;
    private double OPEN_SPECIMEN_CLAW = 0.5;
    private double CLOSE_SPECIMEN_CLAW = 0.77;
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

        //extension initialization
        sample = hardwareMap.get(Servo.class, "sample");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        Wrist = hardwareMap.get(Servo.class, "wrist");

        //Lift: Initialize lift
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotor.Direction.REVERSE);
        //Lift2: Initialize
        Lift2 = hardwareMap.get(DcMotor.class, "lift2");
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setDirection(DcMotor.Direction.FORWARD);

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
        Pose2d pickSamplePose = new Pose2d(0,0,0);
        Pose2d pickSpecimen = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        //initPose -> midwayPose 1 -> specimenDropPose -> midwayPose2 -> parkPose
        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        midwayPose1 = new Pose2d(20, 5, Math.toRadians(0));
        specimenDropPose = new Pose2d(31,10,0); //changed from 28 to 30
        midwayPose2 = new Pose2d(10, -12, Math.toRadians(0));
        pickSamplePose = new Pose2d(55,-35, Math.toRadians(180));
        Pose2d pickSamplePose2 = new Pose2d(55,-38.5, Math.toRadians(180));
        pickSpecimen = new Pose2d(4,-45.5 , Math.toRadians(180));
        Pose2d pickSpecimenPose2 = new Pose2d(8,-45.5 , Math.toRadians(180));
        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
        parkPose = new Pose2d(5, -50, Math.toRadians(0));  //changed from 90 to 0 to face forward
        Pose2d specimenDropPose2 = new Pose2d(24.5, 15, 0);
        drive = new MecanumDrive(hardwareMap, initPose);
        Pose2d specimenDropPose3 = new Pose2d(29.5, 15, 0);

        //Start with the Specimen claw closed and Rotation set
        specimen.setPosition(CLOSE_SPECIMEN_CLAW);
        Rotation.setPosition(0);

        safeWaitSeconds(0.5);
        dropSpecimen();
        //Move to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //Lift.setPower(0);


        //Move to specimenDropPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenDropPose.position, specimenDropPose.heading)
                        .build());
        lowerLift();
        safeWaitSeconds(2);

        specimen.setPosition(OPEN_SPECIMEN_CLAW);


        bringliftDown();

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickSamplePose.position, pickSamplePose.heading)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickSamplePose2.position, pickSamplePose2.heading)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickSpecimen.position, pickSpecimen.heading)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickSpecimenPose2.position, pickSpecimenPose2.heading)
                        .build());
        safeWaitSeconds(1);

        specimen.setPosition(CLOSE_SPECIMEN_CLAW);
        dropSpecimen();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //TODO: DROP SPECIMEN
                        .strafeToLinearHeading(specimenDropPose2.position, specimenDropPose2.heading)
                        .build());


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //TODO: DROP SPECIMEN
                        .strafeToLinearHeading(specimenDropPose3.position, specimenDropPose3.heading)
                        .build());

        safeWaitSeconds(0.25);
        lowerLift();
        specimen.setPosition(OPEN_SPECIMEN_CLAW);

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //TODO: after specimen go here
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .build());

        bringliftDown();

        specimen.setPosition(CLOSE_SPECIMEN_CLAW);

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
    //lift functions
    public void dropSpecimen(){
        //TODO : Code to raise slide and drop specimen
        Lift.setTargetPosition(2000);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(2000);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(-1 * liftPow);
        //while (Lift.isBusy()) {
        telemetry.addData("Current Position", Lift.getCurrentPosition());
        telemetry.addData("Target Position", Lift.getTargetPosition());
        telemetry.addData("Current Position", Lift2.getCurrentPosition());
        telemetry.addData("Target Position", Lift2.getTargetPosition());
        telemetry.update();
        //}
    }
    public void lowerLift(){
        //Lower lift
        Lift.setTargetPosition(1150);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.5);
        Lift.setTargetPosition(1150);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(-0.5);
        //while (Lift.isBusy()) {
        telemetry.addData("Current Position", Lift.getCurrentPosition());
        telemetry.addData("Target Position", Lift.getTargetPosition());
        telemetry.addData("Current Position", Lift2.getCurrentPosition());
        telemetry.addData("Target Position", Lift2.getTargetPosition());
        telemetry.update();
        //}
    }
    public void bringliftDown(){
        //Bring the lift down
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(0);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(-1 * liftPow);
        //while (Lift.isBusy()) {
        telemetry.addData("Current Position", Lift.getCurrentPosition());
        telemetry.addData("Target Position", Lift.getTargetPosition());
        telemetry.addData("Current Position", Lift2.getCurrentPosition());
        telemetry.addData("Target Position", Lift2.getTargetPosition());
        telemetry.update();
        //}
    }
  }