package frc.robot.subsystems.superstructure.leds;
import javax.sound.sampled.Port;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Ports;
import frc.robot.util.TeamColor;
public class leds implements Subsystem {

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

    public leds(){

        led = new AddressableLED(Ports.ADDRESSABLE_LED);

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

    public void setAllPixels(TeamColor color){
        currentColor = color;

        for (int i = 0; i < length; i++){
            buffer.setRGB(i, color.r, color.g, color.b);
        }
        this.updateData();
    }

    public void flashAllPixels(TeamColor color, int numFlashes){
        setAllPixels(color);
        this.numFlashes = numFlashes;
    }
    

    @Override
    public void periodic(){
        periodsPassed ++;

        if (periodsPassed == 5 && numFlashes >0){

            if(on){

                for (int i = 0; i < length;i++){
                    buffer.setRGB(i, offColor.r, offColor.g, offColor.b);
                }

                this.updateData();
                on = false;
            }
            else{

                for(int i = 0; i< length;i++){
                    buffer.setRGB(i, currentColor.r, currentColor.g, currentColor.b);
                }

                this.updateData();
                on = true;
                numFlashes --;
            }
        }


        if (periodsPassed > 5) {
			periodsPassed = 0;
		}

    }


    public int getLength(){
        return length;
    }

    public TeamColor getCurrentColor(){
        return currentColor;
    }
}
