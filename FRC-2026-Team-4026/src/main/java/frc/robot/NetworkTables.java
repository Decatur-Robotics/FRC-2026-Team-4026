package frc.robot;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
    private Subscriber subcX;
    private Subscriber subcY;
    private Subscriber subcount; 
    //Its not datatable i forget what it was it was something similar though. its in vison
    public NetworkTables(){

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("dataTable");
    DoubleTopic cY = table.getDoubleTopic("cY");
    DoubleTopic cX = table.getDoubleTopic("cX");
    DoubleTopic count = table.getDoubleTopic("count");

    subcX = new Subscriber(cX);
    subcY = new Subscriber(cY);
    subcount = new Subscriber(count);
    }
    public double getCX(){
        return subcX.getValue();
    }
    public double getCY(){
        return subcY.getValue();
    }
    public double getCount(){
        return subcount.getValue();
    }

    
}
