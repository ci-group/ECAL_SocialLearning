package evolutionaryrobotics.evolution.odneat.geneticcomponents.operators;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import evolutionaryrobotics.evolution.odneat.geneticcomponents.MacroGenome;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.ODNEATGenome;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.ODNEATLinkGene;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.ODNEATMacroNodeGene;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.ODNEATNodeGene;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.ODNEATTemplateLinkGene;
import evolutionaryrobotics.evolution.odneat.geneticcomponents.StandardGenome;

public class MacroCrossover extends Crossover<ODNEATGenome>{

	protected Random random;

	public MacroCrossover(Random random){
		this.random = random;
	}

	@Override
	/**
	 * NOTE: method receives a copy of the parents.
	 */
	public ODNEATGenome applyCrossover(ODNEATGenome[] parentsCopy, String childId) {
		ODNEATGenome best, worst;
		//get the best
		if(parentsCopy[0].getFitness() == parentsCopy[1].getFitness())
			best = parentsCopy[0].getNumberOfLinkGenes(true) > parentsCopy[1].getNumberOfLinkGenes(false) ? parentsCopy[1] : parentsCopy[0];
			else
				best = parentsCopy[0].getFitness() > parentsCopy[1].getFitness() ? parentsCopy[0] : parentsCopy[1];
				//the worst is the other one.
				worst = best == parentsCopy[0] ? parentsCopy[1] : parentsCopy[0];
				//get the **active** link genes
				ArrayList <ODNEATLinkGene> bestGenes = (ArrayList<ODNEATLinkGene>) best.getLinkGenes(false), 
						worstGenes = (ArrayList<ODNEATLinkGene>) worst.getLinkGenes(false);

				ArrayList<ODNEATLinkGene>[] matchingGenes = getMatchingGenes(bestGenes, worstGenes);
				ArrayList<ODNEATLinkGene> nonMatchingGenes = getNonMatchingGenes(bestGenes, worstGenes);

				//list of node genes, link genes of child
				ArrayList<ODNEATLinkGene> childLinks = new ArrayList<ODNEATLinkGene>();
				ArrayList<ODNEATNodeGene> childNodes = new ArrayList<ODNEATNodeGene>();
				ArrayList<Long> nodesAdded = new ArrayList<Long>();
				addStructure(childLinks, childNodes, nodesAdded, nonMatchingGenes, best);
				addStructure(childLinks, childNodes, nodesAdded, matchingGenes[0], best);
				addStructure(childLinks, childNodes, nodesAdded, matchingGenes[1], worst);

				return (parentsCopy[0] instanceof MacroGenome) ? new MacroGenome(childId, childLinks, childNodes) 
				: new StandardGenome(childId, childLinks, childNodes);
	}

	private void addStructure(ArrayList<ODNEATLinkGene> childLinks,
			ArrayList<ODNEATNodeGene> childNodes, ArrayList<Long> nodesAdded,
			ArrayList<ODNEATLinkGene> linksToAdd, ODNEATGenome genome) {
		for(ODNEATLinkGene link : linksToAdd){
			ODNEATNodeGene[] connectionNodes = null;
			try {
				connectionNodes = getConnectionNodes(link, genome);

			} catch (Exception e) {
				System.out.println("this should not happen...");
				e.printStackTrace();
			}
			ODNEATNodeGene from = connectionNodes[0], to = connectionNodes[1];
			//because of the macro-neurons removal of structure....
			if(from != null && to != null){
				if(!nodesAdded.contains(from.getInnovationNumber())){
					nodesAdded.add(from.getInnovationNumber());
					childNodes.add(from);
				}
				if(!nodesAdded.contains(to.getInnovationNumber())){
					nodesAdded.add(to.getInnovationNumber());
					childNodes.add(to);
				}
				childLinks.add(link);
			}
		}	
	}

	private ODNEATNodeGene[] getConnectionNodes(ODNEATLinkGene link,
			ODNEATGenome genome) throws Exception {
		ODNEATNodeGene from = null, to = null;

		//not input or output of a macro-neuron
		if(!(link instanceof ODNEATTemplateLinkGene)){
			from = genome.getNodeGeneWithInnovationNumber(link.getFromId());
			to = genome.getNodeGeneWithInnovationNumber(link.getToId());
		}
		else {
			ODNEATTemplateLinkGene templateLink = (ODNEATTemplateLinkGene) link;
			//should never happen because the inputs are maintained by the macro-neuron itself.
			if(templateLink.isInput()){
				throw new Exception("Invalid link: " + templateLink.getFromId() + " => " + templateLink.getToId());
			}
			else {
				to = genome.getNodeGeneWithInnovationNumber(link.getToId());
				MacroGenome macrogenome = (MacroGenome) genome;
				from = getMacroNodeGene(link.getFromId(), macrogenome.getMacroNodeGenes());
			}
		}
		//System.out.println((from == null ? ("from null " + link.getFromId()) 
				//: from.getInnovationNumber()) + " " + (to == null ? ("to null " + link.getToId()): to.getInnovationNumber()));
		return new ODNEATNodeGene[]{from, to};
	}

	/**
	 * get the macro node with the "inner node id".
	 */
	private ODNEATNodeGene getMacroNodeGene(long innerNodeId,
			HashMap<Long, ODNEATMacroNodeGene> macroNodeGenes) {
		if(macroNodeGenes.containsKey(innerNodeId)){
			return macroNodeGenes.get(innerNodeId);
		}
		//the node belongs to a macro neuron, search for it.
		for(long key : macroNodeGenes.keySet()){
			ODNEATMacroNodeGene macronode = macroNodeGenes.get(key);
			if(macronode.getBehaviourGene().containsSubNodeWithId(innerNodeId)){
				return macronode;
			}
		}
		return null;
	}

	private ArrayList<ODNEATLinkGene> getNonMatchingGenes(
			ArrayList<ODNEATLinkGene> bestGenes,
			ArrayList<ODNEATLinkGene> worstGenes) {
		ArrayList<ODNEATLinkGene> nonMatchingGenes = new ArrayList<ODNEATLinkGene>();
		for(ODNEATLinkGene bestLink : bestGenes){
			if(!worstGenes.contains(bestLink)){
				nonMatchingGenes.add(bestLink);
			}
		}
		return nonMatchingGenes;
	}

	//matching genes from any parent randomly
	@SuppressWarnings("unchecked")
	private ArrayList<ODNEATLinkGene>[] getMatchingGenes(
			ArrayList<ODNEATLinkGene> bestGenes,
			ArrayList<ODNEATLinkGene> worstGenes) {
		//matching genes taken from the best parent
		ArrayList<ODNEATLinkGene> bestMatching = new ArrayList<ODNEATLinkGene>(), worstMatching = new ArrayList<ODNEATLinkGene>();
		for(ODNEATLinkGene bestLink : bestGenes){
			//matching link gene
			if(worstGenes.contains(bestLink)){
				if(random.nextBoolean()){
					bestMatching.add(bestLink);
					if(bestMatching.get(bestMatching.size() - 1) == null){
						System.out.println("matched best null");
						System.exit(0);
					}
				}
				else {
					int index = worstGenes.indexOf(bestLink);
					worstMatching.add(worstGenes.get(index));
					if(worstMatching.get(worstMatching.size() - 1) == null){
						System.out.println("matched worst null");
						System.exit(0);
					}
				}
			}
		}

		return new ArrayList[]{bestMatching, worstMatching};
	}

}

