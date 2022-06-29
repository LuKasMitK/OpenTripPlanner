package org.opentripplanner.routing.graph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.opentripplanner.routing.graph.TransferPattern.TPNode;
import org.opentripplanner.routing.graph.TransferPattern.TPTravel;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Iterables;
import com.google.common.collect.Sets;

/**
 * Makes sure that all transfer patterns are acyclic.
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPatternGraphCleaner {
	private static final Logger LOG = LoggerFactory.getLogger(TransferPatternGraphCleaner.class);

	/**
	 * Removes all cycles from given transfer pattern graph
	 * 
	 * @param graph
	 *            OTP graph that the tp was created with
	 * @param tpGraph
	 *            transfer pattern graph
	 */
	public static void clean(Graph graph, TransferPatternGraph tpGraph) {
		Set<TransitStationStop> allStops = Sets.newHashSet();
		for (TransitStationStop s : Iterables.filter(graph.getVertices(), TransitStationStop.class)) {
			allStops.add(s);
		}

		for (TransferPattern tp : tpGraph.getTransferPatterns().values()) {
			for (TransitStationStop target : allStops) {
				TPNode targetNode = tp.getTransferPattern(target);
				if (targetNode == null)
					continue;

				visit(targetNode, null, new HashSet<>(6));
			}
		}
	}

	/**
	 * Recursively removes cycles in TP graphs
	 * 
	 * @param node
	 *            the current node to work on
	 * @param lastNode
	 *            the node that linked to the current node
	 * @param visited
	 *            nodes that have been visited before
	 */
	private static void visit(TPNode node, TPNode lastNode, Set<TPNode> visited) {
		if (visited.contains(node)) {
			LOG.info("Removing cycle!");

			// cycle detected, remove it
			Iterator<TPTravel> itTravel = lastNode.listIterator();

			while (itTravel.hasNext()) {
				TPNode current = itTravel.next().getNode();
				if (current == node) {
					itTravel.remove();
				}
			}
		} else {
			visited.add(node);

			// clone predecessors here, since list might be changed in subsequent recursive calls
			List<TPTravel> clonedPreds = new ArrayList<>(node.getPredecessors());

			for (TPTravel pred : clonedPreds) {
				// clone here, since nodes can be reached through more than one path, just not cyclic
				HashSet<TPNode> clonedVisited = new HashSet<>(visited);
				visit(pred.getNode(), node, clonedVisited);
			}
		}
	}
}
