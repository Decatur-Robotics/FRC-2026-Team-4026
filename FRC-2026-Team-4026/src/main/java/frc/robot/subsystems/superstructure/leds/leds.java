package frc.robot.subsystems.superstructure.leds;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.util.TeamColor;
import edu.wpi.first.math.*;
public class leds extends SubsystemBase {

    // an led strip
    private AddressableLED led;
    // how many leds on the strip
    private int length;
    // a buffer with data on the led strip states
    private AddressableLEDBuffer buffer;
    // the current desired color of the leds
    private TeamColor currentColor;
    // the color when the leds are off
    private final TeamColor offColor;
    //number of remaining led flashes
    private int numFlashes;
    //timer for flashing leds
    private int periodsPassed;

    private boolean on;
    //array list for all the leds to add for pulsing
    ArrayList<TeamColor> ledsToAdd = new ArrayList<TeamColor>();
    // different modes for leds.  1 = normal flashing   2 = pulsing
    private int mode;

    private int red = 0;
    private int green = 0;
    private int blue = 0;
    
    private DoubleSupplier shiftProgressBar;
    
    public leds(){
        this.led = new AddressableLED(Ports.ADDRESSABLE_LED);
        this.length = ledsConstants.LENGTH;
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();

  

        numFlashes = 0;
        periodsPassed = 0;

        on = true;
        offColor = ledsConstants.OFF_COLOR;
        currentColor = offColor;

    }

    public void updateData(){
        led.setData(buffer);
    }
   
    public Command runPattern(LEDPattern pattern){
        return run(()->pattern.applyTo(buffer));
    }


    public Command setAllLEDS(Color color){
        return run(()->runPattern(LEDPattern.solid(color)));
    }

    public Command  progressBar(Color color){
        return run(()-> runPattern(LEDPattern.progressMaskLayer(shiftProgressBar)));
    }

    public Command rainbowLEDS(){
        return run(()-> runPattern(LEDPattern.rainbow(255, 128)));
    }



    @Override
    public void periodic(){
        updateData();
    }


    public int getLength(){
        return length;
    }

    public TeamColor getCurrentColor(){
        return currentColor;
    }
}
