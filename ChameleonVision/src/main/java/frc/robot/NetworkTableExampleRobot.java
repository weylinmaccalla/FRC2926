package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;

public class NetworkTableExampleRobot extends TimedRobot {
    private final Servo XAxis = new Servo(0);
    private final Servo YAxis = new Servo(1);
    public NetworkTableEntry yaw;
    public NetworkTableEntry pitch;
    NetworkTable table;
    public void robotInit() {
        
        
        table=NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("USB Camera-B4.09.24.1");
        // Gets the MyCamName table under the chamelon-vision table
        
        // Gets the yaw to the target from the cameraTable
        yaw=table.getEntry("targetYaw");
        pitch=table.getEntry("targetPitch");
        
        // Gets the driveMode boolean from the cameraTable
        
    }
        //yaw is horizontal (Max 30 +/-)
        //pitch is vertical (Max 20 +/-)
    public void teleopPeriodic() {
        // Prints the yaw to the target
        XAxis.set(.5 + yaw.getDouble(0.0)/60);
        YAxis.set(.5 + pitch.getDouble(0.0)/40);
        System.out.println(yaw.getDouble(0.0));
        System.out.println(pitch.getDouble(0.0)); 
        // Sets driver mode to true if the A button is pressed
    
    }
}