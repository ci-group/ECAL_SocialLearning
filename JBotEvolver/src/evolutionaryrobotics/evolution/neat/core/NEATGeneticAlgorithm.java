/*
 * Created on 20-Jun-2005
 *
 */
package evolutionaryrobotics.evolution.neat.core;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import evolutionaryrobotics.evolution.neat.core.mutators.NEATMutator;
import evolutionaryrobotics.evolution.neat.ga.core.Chromosome;
import evolutionaryrobotics.evolution.neat.ga.core.ChromosomeSet;
import evolutionaryrobotics.evolution.neat.ga.core.CrossOver;
import evolutionaryrobotics.evolution.neat.ga.core.FitnessFunction;
import evolutionaryrobotics.evolution.neat.ga.core.GADescriptor;
import evolutionaryrobotics.evolution.neat.ga.core.GeneticAlgorithm;
import evolutionaryrobotics.evolution.neat.ga.core.Mutator;
import evolutionaryrobotics.evolution.neat.ga.core.ParentSelector;
import evolutionaryrobotics.evolution.neat.ga.core.Population;
import evolutionaryrobotics.evolution.neat.ga.core.Specie;
import evolutionaryrobotics.evolution.neat.ga.core.Species;

/**
 *
 * @author MSimmerson
 *
 */
public class NEATGeneticAlgorithm implements GeneticAlgorithm {

    private static final long serialVersionUID = 1L;
    private NEATGADescriptor descriptor;
    private NEATMutator mut;
    private FitnessFunction func;
    private ParentSelector selector;
    private CrossOver xOver;
    private Population pop;
    private Chromosome discoveredBest;
    private Chromosome genBest;
    private Species specieList;
    private int specieIdIdx = 1;
    private int eleCount = 0;
    public InnovationDatabase innov;
    public Chromosome[] currentGen;

    /**
     * Creates a NEAT GA with behaviour defined by the descriptor
     *
     * @param descriptor
     */
    public NEATGeneticAlgorithm(NEATGADescriptor descriptor) {
        this(descriptor, new InnovationDatabase());
    }

    public NEATGeneticAlgorithm(NEATGADescriptor descriptor, InnovationDatabase db) {
        this.descriptor = descriptor;
        this.specieList = new Species();
        this.innov = db;
    }

    public InnovationDatabase innovationDatabase() {
        return innov;
    }
    
    public void setInnovationDatabase(InnovationDatabase db) {
    	this.innov=db;
  
    }

    public FitnessFunction gaEvaluator() {
        return (this.func);
    }

    public NEATMutator gaMutator() {
        return mut;
    }

    public CrossOver gaCrossover() {
        return xOver;
    }

    @Override
	public GADescriptor descriptor() {
        return (this.descriptor);
    }

    public ParentSelector gaSelector() {
        return selector;
    }

    /**
     * Creates the initial population
     */
    @Override
	public void createPopulation() {
        int popSize = this.descriptor.gaPopulationSize();
        int initialChromoSize = this.func.requiredChromosomeSize() + this.descriptor.getExtraFeatureCount();
        this.pop = new NEATPopulation4J(popSize, initialChromoSize, this.descriptor.getInputNodes(), this.descriptor.getOutputNodes(), this.descriptor.featureSelectionEnabled(), this.descriptor.getExtraFeatureCount());
        ((NEATPopulation4J) pop).setInnovationDatabase(innov);
        this.pop.createPopulation();
    }
    
    // changed: a given db
    public void loadPopulation(NEATPopulation4J pop, InnovationDatabase db) {
    	this.pop = pop;
    	this.innov = db;
    	setSpecies(pop.getSpecies());
    }

    /**
     * Finds the best performing chromosome from a give evaluation cycle
     *
     * @param genoTypes - population
     * @return
     */
    private Chromosome findBestChromosome(Chromosome[] genoTypes) {
        Chromosome best = genoTypes[0];
        int i;

        for (i = 1; i < genoTypes.length; i++) {
            if (this.descriptor.isNaturalOrder()) {
                if (genoTypes[i].fitness() < best.fitness()) {
                    best = genoTypes[i];
                }
            } else {
                if (genoTypes[i].fitness() > best.fitness()) {
                    best = genoTypes[i];
                }
            }
        }

        return (best);
    }

    protected void evaluatePopulation(Chromosome[] genoTypes) {
        int i;
        double eval;
        int evalSecond;

        for (i = 0; i < genoTypes.length; i++) {
            eval = this.func.evaluate(genoTypes[i]);
            //evalSecond = this.func.evaluateSecond(genoTypes[i]);
            genoTypes[i].updateFitness(eval);
            System.out.println(eval);
            //genoTypes[i].updateSecondFitness(evalSecond);
        }
    }

    public Chromosome cloneChromosome(Chromosome best) {
        Chromosome cloneBest = new NEATChromosome(best.genes());
        cloneBest.updateFitness(best.fitness());
        cloneBest.updateSecondFitness(best.secondFitness());
        ((NEATChromosome) cloneBest).setSpecieId(((NEATChromosome) best).getSpecieId());

        return (cloneBest);
    }

    private void speciatePopulation(Chromosome[] currentGen) {
        int i;
        int j;
        boolean memberAssigned = false;
        ArrayList currentSpecieList;
        Specie specie;

        this.specieList.resetSpecies(this.descriptor.getThreshold());

        //cat.info("Compat threshold:" + this.descriptor.getThreshold());
        for (i = 0; i < currentGen.length; i++) {
            if (!memberAssigned) {
                currentSpecieList = this.specieList.specieList();
                j = 0;
                while (!memberAssigned && j < currentSpecieList.size()) {
                    specie = (Specie) currentSpecieList.get(j);
                    if (specie.addSpecieMember(currentGen[i])) {
                        memberAssigned = true;
						//((NEATChromosome)currentGen[i]).setSpecieId(specie.id());
                        //cat.info("Member assigned to specie " + specie.id());
                    } else {
                        j++;
                    }
                }

                if (!memberAssigned) {
                    specie = this.createNewSpecie(currentGen[i]);
                    this.specieList.addSpecie(specie);
					((NEATChromosome)currentGen[i]).setSpecieId(specie.id());
                    //cat.info("Created new specie, member assigned to specie " + specie.id());
                }
            }
            memberAssigned = false;
        }

        /*if (cat.isDebugEnabled()) {
         for (i = 0; i < this.specieList.specieList().size(); i++) {
         specie = (Specie)this.specieList.specieList().get(i);
         if (!specie.isExtinct() && specie.specieMembers().size() > 0) {
         cat.debug("Specie:" + specie.id() + ":size:" + specie.specieMembers().size() + ":age:" + ((NEATSpecie)specie).specieAge() + ":fAge:" + + specie.getCurrentFitnessAge() + ":best:" + specie.findBestMember().fitness());
         }
         }
         }*/
    }

    /**
     * Runs an evaluation and evolution cycle
     */
    
	public void runEpoch() {
        
		
		currentGen = this.pop.genoTypes();
        this.setChromosomeNO(currentGen);
        //cat.debug("Evaluating pop");
        this.evaluatePopulation(currentGen);
        //this is where everything is stored
        
        //this.runEvolutionCycle(currentGen);
        
    }
    
    public Chromosome[] givePopulation(){
    	return currentGen;
    }

    /**
     * Runs an evolution cycle on the given population
     *
     * @param currentGen
     */
    public void runEvolutionCycle(Chromosome[] currentGen) {
        NEATChromosome champ;
        ArrayList validSpecieList;

        // ELE? boolean is false so this function does not run
        this.runEle(currentGen);
        // speciate the remaining population
        this.speciatePopulation(currentGen);
        this.genBest = this.findBestChromosome(currentGen);
        if ((this.discoveredBest == null) || (this.genBest.fitness() >= this.discoveredBest.fitness() && !this.descriptor.isNaturalOrder()) || (this.genBest.fitness() <= this.discoveredBest.fitness() && this.descriptor.isNaturalOrder())) {
            // copy best
            this.discoveredBest = this.cloneChromosome(this.genBest);
        }
		//cat.info("Best Ever Raw:" + (this.discoveredBest.fitness()) + ":from specie:" + ((NEATChromosome)this.discoveredBest).getSpecieId());		
        //cat.debug("Best of Generation is:" + (this.genBest.fitness()) + " specie " + ((NEATChromosome)this.genBest).getSpecieId());
        // kill any extinct species
        if (this.descriptor.keepBestEver()) {
            champ = (NEATChromosome) this.discoveredBest;
        } else {
            champ = (NEATChromosome) this.genBest;
        }
        this.specieList.removeExtinctSpecies(champ);
		//cat.debug("Creating New Gen");
        // spawn new pop	
        this.pop.updatePopulation(this.spawn());
        // Display specie stats
        validSpecieList = this.specieList.validSpecieList(champ.getSpecieId());
        //cat.debug("Num species:" + validSpecieList.size());
        if (this.descriptor.getCompatabilityChange() > 0) {
            if (validSpecieList.size() > this.descriptor.getSpecieCount()) {
                this.descriptor.setThreshold(this.descriptor.getThreshold() + this.descriptor.getCompatabilityChange());
            } else if (validSpecieList.size() < this.descriptor.getSpecieCount() && (this.descriptor.getThreshold() > this.descriptor.getCompatabilityChange())) {
                this.descriptor.setThreshold(this.descriptor.getThreshold() - this.descriptor.getCompatabilityChange());
            }
        }
        this.eleCount++;
    }

    private void runEle(Chromosome[] currentGen) {
        if (this.descriptor.isEleEvents()) {
            if ((this.eleCount % this.descriptor.getEleEventTime()) == 0 && this.eleCount != 0) {
                //cat.info("Runnig ELE");
                this.descriptor.setThreshold(this.descriptor.getThreshold() * 5);
            } else if ((this.eleCount % this.descriptor.getEleEventTime()) == 1 && this.eleCount != 1) {
                this.descriptor.setThreshold(this.descriptor.getThreshold() / 5.0);
            }
        }
    }

    private void setChromosomeNO(Chromosome[] gen) {
        int i;
        if(gen!=null){
        for (i = 0; i < gen.length; i++) {
        	//System.out.println("gen length" + i);
        	//System.out.println("descriptor" + this.descriptor.isNaturalOrder());
        	//System.out.println("gen fitness" + ((NEATChromosome) gen[i]).fitness());
            ((NEATChromosome) gen[i]).setNaturalOrder(this.descriptor.isNaturalOrder());
        }
        }
    }

    private Chromosome[] spawn() {
        Chromosome[] currentGen = this.pop.genoTypes();
        Chromosome[] newGen = new Chromosome[currentGen.length];
        Specie specie = null;
        ChromosomeSet offspring = null;
        int offSpringCount;
        int newGenIdx = 0;
        int i;
        int j = 0;
        double totalAvFitness;
		//int offSp = 0;

        // update species by sharing fitness.
        this.specieList.shareFitness();
        totalAvFitness = this.specieList.totalAvSpeciesFitness();
        // mate within valid species to produce new population
        ArrayList species = this.specieList.validSpecieList(((NEATChromosome) this.discoveredBest).getSpecieId());
        for (i = 0; i < species.size(); i++) {
            specie = (Specie) species.get(i);
            if (specie.specieMembers().size() == 0) {
                System.err.println("spawn produced error:");
            }
            // ensure we have enough for next gen
            if (i == (species.size() - 1)) {
                offSpringCount = newGen.length - newGenIdx;
                //offSp = this.specieList.calcSpecieOffspringCount(specie, this.descriptor.gaPopulationSize(), totalAvFitness);
            } else {
                offSpringCount = this.specieList.calcSpecieOffspringCount(specie, this.descriptor.gaPopulationSize(), totalAvFitness);
            }

            //cat.debug("Sp[" + specie.id() + "] Age:" + ((NEATSpecie)specie).specieAge() + ":Offspring Sz:" + offSpringCount + ":AvF:" + specie.getAverageFitness() + ":FAge:" + specie.getCurrentFitnessAge() + ":BestF:" + specie.findBestMember().fitness());
            if (offSpringCount > 0) {
                offspring = specie.specieOffspring(offSpringCount, this.mut, this.selector, this.xOver);
                for (j = 0; j < offspring.size(); j++) {
                    if (newGenIdx < newGen.length) {
                        // if population not full
                        newGen[newGenIdx++] = offspring.nextChromosome();
                    }
                }
            } else {
                //cat.debug("Specie " + specie.id() + ":size:" + specie.specieMembers().size() + " produced no offspring.  Average fitness was " + specie.averageFitness() + " out of a total fitness of " + this.specieList.totalAvSpeciesFitness());
                specie.setExtinct();
            }
        }

        return (newGen);
    }

    private Specie createNewSpecie(Chromosome member) {
        double excessCoeff = this.descriptor.getExcessCoeff();
        double disjointCoeff = this.descriptor.getDisjointCoeff();
        double weightCoeff = this.descriptor.getWeightCoeff();
        double threshold = this.descriptor.getThreshold();
        boolean copyBest = this.descriptor.getCopyBest();

        NEATSpecie specie = new NEATSpecie(threshold, excessCoeff, disjointCoeff, weightCoeff, specieIdIdx++, copyBest);
        specie.setMaxFitnessAge(this.descriptor.getMaxSpecieAge());
        specie.setAgePenalty(this.descriptor.getAgePenalty());
        specie.setAgeThreshold(this.descriptor.getSpecieAgeThreshold());
        specie.setYouthBoost(this.descriptor.getYouthBoost());
        specie.setYouthThreshold(this.descriptor.getSpecieYouthThreshold());
        specie.setSurvivalThreshold(this.descriptor.getSurvivalThreshold());
        specie.addSpecieMember(member);

        return (specie);
    }

    @Override
	public Chromosome discoverdBestMember() {
        return (this.discoveredBest);
    }

    /**
     * @see
     * org.neat4j.ailibrary.ga.core.GeneticAlgorithm#pluginMutator(org.neat4j.ailibrary.ga.core.Mutator)
     */
    @Override
	public void pluginMutator(Mutator mut) {
        this.mut = (NEATMutator) mut;
        this.mut.setPAddLink(this.descriptor.getPAddLink());
        this.mut.setPAddNode(this.descriptor.getPAddNode());
        this.mut.setPPerturb(this.descriptor.getPMutation());
        this.mut.setPToggle(this.descriptor.getPToggleLink());
        this.mut.setPWeightReplaced(this.descriptor.getPWeightReplaced());
        this.mut.setFeatureSelection(this.descriptor.featureSelectionEnabled());
        this.mut.setRecurrencyAllowed(this.descriptor.isRecurrencyAllowed());
        this.mut.setPMutateBias(this.descriptor.getPMutateBias());
        this.mut.setBiasPerturb(this.descriptor.getMaxBiasPerturb());
        this.mut.setPerturb(this.descriptor.getMaxPerturb());
        this.mut.setInnovationDatabase(innov);
    }

    /**
     * @see
     * org.neat4j.ailibrary.ga.core.GeneticAlgorithm#pluginFitnessFunction(org.neat4j.ailibrary.ga.core.Function)
     */
    @Override
	public void pluginFitnessFunction(FitnessFunction func) {
        this.func = func;
    }

    /**
     * @see
     * org.neat4j.ailibrary.ga.core.GeneticAlgorithm#pluginParentSelector(org.neat4j.ailibrary.ga.core.ParentSelector)
     */
    @Override
	public void pluginParentSelector(ParentSelector selector) {
        this.selector = selector;
        this.selector.setOrderStrategy(this.descriptor.isNaturalOrder());
    }

    /**
     * @see
     * org.neat4j.ailibrary.ga.core.GeneticAlgorithm#pluginCrossOver(org.neat4j.ailibrary.ga.core.CrossOver)
     */
    @Override
	public void pluginCrossOver(CrossOver xOver) {
        this.xOver = xOver;
        this.xOver.setProbability(this.descriptor.getPXover());
    }

    /**
     * Saves the entire population. Especially useful for long running evolution
     * processes
     */
    @Override
	public void savePopulationState(String fileName) {
        FileOutputStream out = null;
        ObjectOutputStream s = null;
        try {
            if (fileName != null) {
                //cat.debug("Saving Population " + fileName);
                out = new FileOutputStream(fileName);
                s = new ObjectOutputStream(out);
                s.writeObject(this.pop);
                s.flush();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                s.close();
                out.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        }

        //cat.debug("Saving Population...Done");
    }

    @Override
	public Population population() {
        return (this.pop);
    }

    public Chromosome generationBest() {
        return (this.genBest);
    }

    public Species getSpecies() {
        return specieList;
    }
    
    public void setSpecies(Species specieList) {
		this.specieList = specieList;
	}
}
