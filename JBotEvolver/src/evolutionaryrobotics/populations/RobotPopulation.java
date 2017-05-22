package evolutionaryrobotics.populations;


public interface RobotPopulation<E> {

	public int getCurrentPopulationSize();
	public int getMaxPopulationSize();
	
	public E reproduce();
	
	public E getFittestIndividual();
}

