package frc.robot;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;

public class Subscriber {
    private double value;
    final DoubleSubscriber dblSub;
    //You need to make a subscriber object so that the subscriber lasts for the entire program
    public Subscriber(DoubleTopic dblTopic){
        //subscribes to the networktable topic which connects it 
        //and allows you to get the value of the subscriber/the network table topic.
        dblSub = dblTopic.subscribe(0);


    }
    //This updates the value of the networktable topic
    public void periodic(){
        value = dblSub.get();

    }

    public double getValue(){
        return value;
    }

    

}
