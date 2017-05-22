package evolutionaryrobotics.evolution.odneat.controlsystem;

import java.util.ArrayList;
import java.util.Random;

import evolutionaryrobotics.neuralnetworks.ERNEATNetwork;

//import org.encog.engine.network.activation.ActivationFunction;
//import org.encog.neural.neat.NEATLink;
//import org.encog.neural.neat.NEATNetwork;

import controllers.Controller;
import evolutionaryrobotics.evaluationfunctions.EvaluationFunction;
import evolutionaryrobotics.neuralnetworks.NeuralNetwork;
import evolutionaryrobotics.evaluationfunctions.ODNEATEvaluationFunction;
import evolutionaryrobotics.evolution.odneat.evolutionaryalgorithm.ODNEAT;
import evolutionaryrobotics.evolution.odneat.evolutionaryalgorithm.OnlineEA;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.ODNEATGenome;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.operators.ALGDescriptor;
import evolutionaryrobotics.evolution.odneat.genotypePhenotypeMapping.GPMapping;
import simulation.Simulator;
import simulation.robot.Robot;
import simulation.util.Arguments;

public class ANNODNEATController extends Controller implements OnlineController<ODNEATGenome>{

	private static final long serialVersionUID = 1L;
	protected ERNEATNetwork neuralNetwork;
	//protected GPMapping<ODNEATGenome,NEATNetwork> map;
	protected ODNEAT instance;
	
	@SuppressWarnings("unchecked")
	public ANNODNEATController(Simulator simulator, Robot robot, Arguments args) {
		super(simulator, robot, args);
		
		neuralNetwork = (ERNEATNetwork) NeuralNetwork.getNeuralNetwork(simulator, robot, 
				new Arguments(args.getArgumentAsString("network")));
		
		//map = GPMapping.getGPMapping(new Arguments(args.getArgumentAsString("mapping")));
	}

	public void setNetwork(NeuralNetwork network) {
//		NeuralNetwork copyNetwork = createCopyNetwork(network);
//		this.neuralNetwork.setNEATNetwork(copyNetwork);
	}
	
	public static synchronized NeuralNetwork createCopyNetwork(NeuralNetwork neatNetwork) {
//		//get
//		NEATLinkGene[] originalLinks = neatNetwork.getLinks();
//		ActivationFunction[] originalFunctions = ((NeuralNetwork) neatNetwork).getActivationFunctions();
//		
//		//initialise
//		int inputs = neatNetwork.getInputCount(), outputs = neatNetwork.getOutputCount();
//		ArrayList<NEATLink> links = new ArrayList<NEATLink>(originalLinks.length);
//		ActivationFunction[] functions = new ActivationFunction[originalFunctions.length];
//		
//		//copy/clone
//		for(int i = 0; i < originalLinks.length; i++){
//			NEATLink original = originalLinks[i];
//			links.add(new NEATLink(original.getFromNeuron(), original.getToNeuron(), original.getWeight()));
//		}
//		
//		for(int i = 0; i < originalFunctions.length; i++){
//			ActivationFunction original = originalFunctions[i];
//			functions[i] = original.clone();
//		}
		
	//	return new NEATNetwork(inputs, outputs, links, functions);
		return null;
	}
		
	public OnlineEA<ODNEATGenome> getEAInstance(){
		return instance;
	}
	
	@Override
	public void initialise(Random random, int robotId,
			ALGDescriptor descriptor, Robot r, EvaluationFunction eval, Arguments genomeArguments) {
		int inputs = this.neuralNetwork.getNumberOfInputNeurons(), 
				outputs = this.neuralNetwork.getNumberOfOutputNeurons();
		System.out.println("I: " + inputs + "; O: " + outputs + "\n");
		ODNEATEvaluationFunction func = (ODNEATEvaluationFunction) eval;
		this.instance = new ODNEAT(random, robotId, descriptor, r, func, inputs, outputs, genomeArguments);
		this.updateStructure(instance.getActiveGenome());
	}
	
	public void updateStructure(Object genome){
		//this.neuralNetwork.setNEATNetwork(map.decode((ODNEATGenome) genome));
	}
	
	@Override
	public void begin() {
	}

	@Override
	public void controlStep(double time) {
		neuralNetwork.controlStep(time);
	}

	@Override
	public void end() {
	}

	@Override
	public void reset() {
		super.reset();
		neuralNetwork.reset();
	}

	@Override
	public void setOnlineEAInstance(OnlineEA<?> instance) {
		this.instance = (ODNEAT) instance;
	}
}

