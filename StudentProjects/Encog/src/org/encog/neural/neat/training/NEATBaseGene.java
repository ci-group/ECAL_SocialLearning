/*
 * Encog(tm) Core v3.2 - Java Version
 * http://www.heatonresearch.com/encog/
 * https://github.com/encog/encog-java-core
 
 * Copyright 2008-2013 Heaton Research, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *   
 * For more information on Heaton Research copyrights, licenses 
 * and trademarks visit:
 * http://www.heatonresearch.com/copyright
 */
package org.encog.neural.neat.training;

import java.io.Serializable;

/**
 * Defines a base class for NEAT genes. A neat gene holds instructions on how to
 * create either a neuron or a link. The NEATLinkGene and NEATLinkNeuron classes
 * extend NEATBaseGene to provide this specific functionality.
 * 
 * -----------------------------------------------------------------------------
 * http://www.cs.ucf.edu/~kstanley/ Encog's NEAT implementation was drawn from
 * the following three Journal Articles. For more complete BibTeX sources, see
 * NEATNetwork.java.
 * 
 * Evolving Neural Networks Through Augmenting Topologies
 * 
 * Generating Large-Scale Neural Networks Through Discovering Geometric
 * Regularities
 * 
 * Automatic feature selection in neuroevolution
 */
public class NEATBaseGene implements Comparable<NEATBaseGene>, Serializable {
	/**
	 * Serial id.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * ID of this gene, -1 for unassigned.
	 */
	private long id = -1;

	/**
	 * Innovation ID, -1 for unassigned.
	 */
	private long innovationId = -1;

	/**
	 * {@inheritDoc}
	 */
	@Override
	public int compareTo(final NEATBaseGene o) {
		return ((int) (getInnovationId() - o.getInnovationId()));
	}

	/**
	 * @return The id of this gene.
	 */
	public long getId() {
		return id;
	}

	/**
	 * @return The innovation id of this gene.
	 */
	public long getInnovationId() {
		return innovationId;
	}

	/**
	 * Set the id for this gene.
	 * 
	 * @param i
	 *            The id for this gene.
	 */
	public void setId(final long i) {
		this.id = i;
	}

	/**
	 * Set the innovation id for this gene.
	 * 
	 * @param theInnovationID
	 *            The innovation id for this gene.
	 */
	public void setInnovationId(final long theInnovationID) {
		innovationId = theInnovationID;
	}
}
