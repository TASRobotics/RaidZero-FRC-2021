package raidzero.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import raidzero.robot.auto.AutoRunner;
import raidzero.robot.auto.DetermineGSCPath;
import raidzero.robot.auto.sequences.EmptySequence;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.teleop.Teleop;
import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.submodules.Conveyor;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Led;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Spindexer;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.Superstructure;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private static final Teleop teleop = Teleop.getInstance();
    private static final Swerve swerve = Swerve.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Conveyor conveyor = Conveyor.getInstance();
    private static final Spindexer spindexer = Spindexer.getInstance();
    private static final AdjustableHood hood = AdjustableHood.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Turret turret = Turret.getInstance();
    private static final Led led = Led.getInstance();

    private static final Superstructure superstructure = Superstructure.getInstance();

    private AutoRunner autoRunner;

    // private NetworkTableEntry shouldCheckGSCEntry =
    //     Shuffleboard.getTab(Tab.SELECTION).add("Scan Path", 0).withWidget(BuiltInWidgets.kBooleanBox)
    //             .withSize(1, 1).withPosition(0, 0).getEntry();
    // private NetworkTableEntry foundPathEntry =
    //     Shuffleboard.getTab(Tab.SELECTION).add("Chosen Path", 0).withWidget(BuiltInWidgets.kTextView)
    //             .withSize(2, 1).withPosition(0, 1).getEntry();

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        // Register all submodules here
        submoduleManager.setSubmodules(
            // swerve,
            superstructure,
            swerve,
            intake,
            conveyor,
            spindexer, 
            hood,
            shooter,
            turret,
            led
        );
        submoduleManager.onInit();

        autoRunner = new AutoRunner();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // Stop autonomous
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {
        submoduleManager.onStart(Timer.getFPGATimestamp());
        
        // if (shouldCheckGSCEntry.getBoolean(false)) {
        if (false) {
            var result = DetermineGSCPath.lookForPath();
            System.out.println("Found path: " + result.name());
            // autoRunner.selectSequence(new EmptySequence());
            switch (result) {
            case PATH_A_RED:
                autoRunner.selectSequence("Search Path A Sequence Red");
                break;
            case PATH_A_BLUE:
                autoRunner.selectSequence("Search Path A Sequence Blue");
                break;
            case PATH_B_RED:
                autoRunner.selectSequence("Search Path B Sequence Red");
                break;
            case PATH_B_BLUE:
                autoRunner.selectSequence("Search Path B Sequence Blue");
                break;
            default:
                autoRunner.selectSequence(new EmptySequence());
                break;
            }
        } else {
            autoRunner.readSendableSequence();
        }
        autoRunner.start();
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        // System.out.println("tx full: " + RobotController.getCANStatus().txFullCount);
        autoRunner.onLoop(timestamp);
        submoduleManager.onLoop(timestamp);
    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
        // Stop the autonomous
        autoRunner.stop();

        // Start the teleop handler
        teleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during teleop (50 Hz).
     */
    @Override
    public void teleopPeriodic() {
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }

}
