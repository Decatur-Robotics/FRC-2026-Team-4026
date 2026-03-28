package frc.robot;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
    private Subscriber subcX;
    private Subscriber subcY;
    private Subscriber subRotationOfGamePiece; 

    public NetworkTables(){

    //Gets the default network table. I think this is always called no matter what
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //gets the specific table from the network tables.
    NetworkTable table = inst.getTable("datatable");
    //gets the topics (essentially subfiles) from the table
    DoubleTopic cY = table.getDoubleTopic("cY");
    DoubleTopic cX = table.getDoubleTopic("cX");

    DoubleTopic rotationOfGamePiece = table.getDoubleTopic("rotationOfGamePiece");
//Makes a subscriber for eahc of the networktable topics that gets transmitted
    subcX = new Subscriber(cX);
    subcY = new Subscriber(cY);
    subRotationOfGamePiece = new Subscriber(rotationOfGamePiece);
    }
    public double getCX(){
        //this is the method created in subscriber that returns the value of the topic that the subscriber is subscribed to
        return subcX.getValue();
    }
    public double getCY(){
        return subcY.getValue();
    }
    public double getFuelRotation(){
        return subRotationOfGamePiece.getValue();
    }

    
}
