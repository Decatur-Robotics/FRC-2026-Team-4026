package frc.robot;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;

public class Subscriber {
    private double value;
    final DoubleSubscriber dblSub;
    public Subscriber(DoubleTopic dblTopic){
        dblSub = dblTopic.subscribe(0);


    }
    public void periodic(){
        value = dblSub.get();

    }
    public double getValue(){
        return value;
    }

    

}
