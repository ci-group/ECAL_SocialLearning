package simulation.robot.sensors;

import simulation.Simulator;
import simulation.robot.Robot;
//import simulation.robot.actuators.PreyPickerActuator;
import simulation.util.Arguments;

public class PreyCarriedSensor extends Sensor{
    
    boolean hasprey = false;
	
	//protected PreyPickerActuator actuator;

	public PreyCarriedSensor(Simulator simulator,int id, Robot robot, Arguments args) {
		super(simulator, id, robot, args);
	}
	
	public boolean preyCarried() {
        
        return hasprey;
        /*
		if(actuator == null) {
			actuator = (PreyPickerActuator) robot.getActuatorByType(PreyPickerActuator.class);
		}
		return actuator.isCarryingPrey(); */
	}
	
	@Override
	public double getSensorReading(int sensorNumber) {
		return preyCarried() ? 1 : 0;
	}

	@Override
	public String toString() {
		return "PreyCarriedSensor ["+preyCarried()+"]";
	}
    
    public void havePrey(){
        hasprey = true;
    }
    
    public void noPrey(){
        hasprey = false;
    }
}