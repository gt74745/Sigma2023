package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ChangeArmTargetAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawSyncAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ColorWaypointJumpAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.DriveToFreightAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.OpenClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.OpenClawWideAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.OverridePositionAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.WaitForActionsAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.WaypointBuilder;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.WaypointManager;

/*
The Instructions class contains a map of waypoints and actions. It outlines all of the
tasks that the robot carries out during the autonomous phase. This class provides a method of
isolating these instructions to a separate class, rather than clogging up the AutonCore class.

This class also provides a method of disposing resources.
 */
public class Instructions {
    public static double initialX, initialY, initialTheta;

    public Navigation navigation;
    public Actions actions;
    public ObjectDetector objectDetector;
    public final WaypointManager waypointManager;
    private final LinearOpMode opMode;
    public ArmPositionAction armPositionAction;

    public Instructions(Hardware hardware, Localization localization, LinearOpMode opMode, double _initialX, double _initialY, double _initialTheta, ObjectDetector objectDetector)
    {
        this.objectDetector = objectDetector;
        initialX = _initialX;
        initialY = _initialY;
        initialTheta = _initialTheta;
        objectDetector.calculateState();
        this.waypointManager = new WaypointManager();
        this.opMode = opMode;
        navigation = new Navigation(hardware, localization);
        armPositionAction = new ArmPositionAction(hardware, this);
        registerNav(initialX, initialY, initialTheta, hardware);
    }

    //Enter initial navigation waypoints here.
    private void registerNav(double initialX, double initialY, double initialTheta, Hardware hardware)
    {
        actions = new Actions();
        WaypointBuilder waypointBuilder = new WaypointBuilder(waypointManager, actions, hardware, this);
        double targetArmPos = 0;
        switch (objectDetector.getTeamElementLocation()) {
            case LEFT:
                targetArmPos = -1200;
                break;
            case CENTER:
                targetArmPos = -2800;
                break;
            case RIGHT:
                targetArmPos = -4500;
                break;
        }
        if (!Constants.IS_LEFT_OPMODE) {
//            waypointBuilder
//                    .move(new Position(initialX, initialY, 0))
//                        .run(new OpenClawWideAction())
//                        .run(new DriveToFreightAction(objectDetector, true))
//                        .run(new CloseClawSyncAction());
            double shippingHubX = Constants.IS_BLUE_TEAM ? 900 : 840;//870;
            if (objectDetector.getTeamElementLocation() == ObjectDetector.TeamElementLocation.CENTER) {
                shippingHubX += 20;
            } else if (objectDetector.getTeamElementLocation() == ObjectDetector.TeamElementLocation.RIGHT) {
                shippingHubX += 60;
            }
            double shippingHubY = 1540;
            waypointBuilder
                    .runContinuously(armPositionAction)
                    .move(new Position(initialX, initialY, 0))
                        .run(new CloseClawAction())
                        .run(new ChangeArmTargetAction(targetArmPos))
                    .move(new Position(initialX + 0.8 * (shippingHubX - initialX), initialY + 0.8 * (shippingHubY - initialY)))
                        .run(new WaitForActionsAction(actions))
                    .move(new Position(shippingHubX, shippingHubY, 0))
                        .run(new OpenClawAction())
                    .move(new Position(shippingHubX, shippingHubY), new Position(664, 1250), new Position(600, 1040), new Position(575, 664, 0))
                        .run(new ChangeArmTargetAction(-500))
                    .move(new Position(Constants.IS_BLUE_TEAM ? 575 : 425, 70, 0))
//                        .run(new ChangeArmTargetAction(0))
                        .run(new WaitForActionsAction(actions))
//                        .run(new OpenClawWideAction())
                        .run(new SpinCarouselAction())
                        .run(new OverridePositionAction(305d, 305d))
//                    .move(new Position(1500, 545, -Math.PI))
//                        .run(new DriveToFreightAction(objectDetector, true))
//                        .run(new CloseClawSyncAction())
//                        .run(new ColorWaypointJumpAction(7))
//                        .run(new ChangeArmTargetAction(-3800))
//                    .move(new Position(1300, 1600, Math.PI/4))
//                        .run(new WaitForActionsAction(actions))
//                        .run(new OpenClawAction())
//                        .run(new ChangeArmTargetAction(0))
                    .move(new Position(1055, 100, 0))
                        .run(new CloseClawSyncAction())
                        .run(new ChangeArmTargetAction(0))
                        .run(new WaitForActionsAction(actions));
        } else {
//            waypointBuilder
//                    .move(new Position(initialX, initialY, Math.PI / 2))
//                        .run(new OpenClawWideAction())
//                        .run(new DriveToFreightAction(objectDetector, false))
//                        .run(new CloseClawSyncAction());
            double shippingHubX = 920;//1020;
            if (objectDetector.getTeamElementLocation() == ObjectDetector.TeamElementLocation.CENTER) {
                shippingHubX += 20;
            } else if (objectDetector.getTeamElementLocation() == ObjectDetector.TeamElementLocation.RIGHT) {
                shippingHubX += 60;
            }
            double shippingHubY = 1370;
            double initialXSpinOffset = 20;
            double warehouseAngle = Constants.IS_BLUE_TEAM ? -3 * Math.PI / 2 : Math.PI / 2;
            waypointBuilder
                    .runContinuously(armPositionAction)
                    .move(new Position(initialX, initialY, 0))
                        .run(new CloseClawAction())
                        .run(new ChangeArmTargetAction(targetArmPos))
                    .move(new Position(initialX + 0.8 * (shippingHubX - initialX), initialY + 0.8 * (shippingHubY - initialY), 0))
                        .run(new WaitForActionsAction(actions))
                    .move(new Position(shippingHubX, shippingHubY, 0))
                        .run(new OpenClawAction())
                    .move(new Position(773, 1780, warehouseAngle))
                        .run(new ChangeArmTargetAction(-500))
                    .move(new Position(initialX - initialXSpinOffset, 1780, warehouseAngle))
                        .run(new ChangeArmTargetAction(0))
                        .run(new OpenClawWideAction())
                    .move(new Position(initialX - initialXSpinOffset, 2400,  warehouseAngle))
                        .run(new DriveToFreightAction(objectDetector, false))
                        .run(new CloseClawSyncAction())
//                        .run(new OverridePositionAction(initialX - initialXSpinOffset, null))
//                        .run(new ColorWaypointJumpAction(10))
//                        .run(new ChangeArmTargetAction(-4500))
//                    .move(new Position(initialX - initialXSpinOffset - 300, 1780, warehouseAngle))
//                        .run(new OverridePositionAction(initialX - initialXSpinOffset, null))
//                    .move(new Position(shippingHubX, shippingHubY, 0))
//                        .run(new OpenClawAction())
//                    .move(new Position(initialX - initialXSpinOffset, 1780, warehouseAngle))
//                        .run(new ChangeArmTargetAction(-100))
//                        .run(new CloseClawAction())
//                    .move(new Position(initialX - initialXSpinOffset, 2400, warehouseAngle))
//                        .run(new ChangeArmTargetAction(0))
                    .move(new Position(initialX + 100, 3000, 0))
                        .run(new WaitForActionsAction(actions))
                    .move(new Position(initialX + 100, 3000, 0));
        }
    }

    public void runTasks()
    {
        actions.initialize();
        while (waypointManager.nextWaypoint()) {
            Waypoint waypoint = waypointManager.getCurrentWaypoint();
            if (opMode.isStopRequested()) {
                break;
            }

            double time = System.currentTimeMillis();
            navigation.clear();
            while (System.currentTimeMillis() - time < 500)
            {
                //nothing
            }

            loopToWaypoint(waypoint, false);

            navigation.clear();

            if (opMode.isStopRequested())
                break;

            actions.executeActions(waypointManager.getIndex());
        }
    }

    public void loopToWaypoint(Waypoint waypoint, boolean isForDuck) {
        while (!navigation.isTargetReached(waypoint) && !opMode.isStopRequested()) {
            actions.executeContinuousActions();
            if (waypoint.isSpline) {
                navigation.driveToTarget(waypoint.startingPos, waypoint.splinePos1, waypoint.splinePos2, waypoint.targetPos, true, waypoint.onlyRotate, isForDuck);
            } else {
                navigation.driveToTarget(waypoint.targetPos, false, waypoint.onlyRotate, isForDuck);
            }
        }
    }


    public void reset() {
        navigation.reset();
        actions.reset();
    }
}
