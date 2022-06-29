package org.opentripplanner.routing.graph;

import java.util.Calendar;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import javax.annotation.Nonnull;

import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.TransferPattern.DelayClassification;
import org.opentripplanner.routing.graph.TransferPattern.TPNode;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.opentripplanner.routing.vertextype.TransitStopDepart;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Class that helps recording transfer patterns by holding temporary information
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPatternEditor {
	private static final Logger LOG = LoggerFactory.getLogger(TransferPatternEditor.class);

	private TransitStationStop root;
	private HashMap<TransitStationStop, TPNode> targets;

	/**
	 * saves hash map of intermediate nodes for specific key target stop
	 */
	private final HashMap<TransitStationStop, HashMap<TransitStationStop, TPNode>> allIntermediateNodes;
	private final HashMap<TripPattern, Integer> possibleDelays;

	private final Set<TransitStationStop> allStops;

	/**
	 * @param originStop
	 *            stop that the transfer pattern originates from
	 * @param allTargetStops
	 *            all target stops
	 */
	public TransferPatternEditor(TransitStationStop originStop, Set<TransitStationStop> allTargetStops) {
		allStops = allTargetStops;
		root = originStop;

		targets = new HashMap<>(allTargetStops.size());
		allIntermediateNodes = new HashMap<>(allTargetStops.size());

		possibleDelays = new HashMap<>();
	}

	/**
	 * Adds paths from the shortestPathTree to this transfer pattern. All added
	 * links are provided with given delay class.
	 * 
	 * @param spt
	 *            tree holding shortest paths to target stops
	 * @param delayClass
	 *            delay scenario that was used creating the spt
	 */
	public void add(@Nonnull ShortestPathTree spt, DelayClassification delayClass) {
		for (TransitStationStop target : allStops) {
			// skip trip to itself
			if (target == root)
				continue;

			List<State> targetStates = spt.getStates(target);
			// get optimal paths to current stop as states
			if (targetStates == null)
				// no route found to this target
				continue;

			addPatterns(target, targetStates, delayClass);
		}
	}

	/**
	 * Adds patterns from source to given target stop to the TP. If delayClass
	 * is null, used trips and their maximal preceding waiting time are
	 * recorded.
	 * 
	 * @param target
	 *            target stop to consider
	 * @param targetStates
	 *            list of states that represent paths to the target stop
	 * @param delayClass
	 *            delay scenario that was used creating the shortest paths
	 */
	private void addPatterns(TransitStationStop target, List<State> targetStates, DelayClassification delayClass) {
		HashMap<TransitStationStop, TPNode> intermediateNodes = allIntermediateNodes.get(target);
		if (intermediateNodes == null) {
			intermediateNodes = new HashMap<>(6);
			allIntermediateNodes.put(target, intermediateNodes);
		}

		TPNode targetNode = intermediateNodes.get(target);
		if (targetNode == null) {
			targetNode = new TPNode(target);
			intermediateNodes.put(target, targetNode);
		}

		for (State state : targetStates) {
			TPNode beforeNode = null;
			boolean wasWalking = false;

			long lastTime = -1;

			while (state != null) {
				if (state.getBackState() == null || state.getBackMode() == TraverseMode.LEG_SWITCH
						|| state.getBackMode() == TraverseMode.WALK) {
					Vertex v = state.getVertex();

					if (v instanceof TransitStationStop) {
						TransitStationStop stop = (TransitStationStop) v;

						TPNode currentNode = intermediateNodes.get(stop);
						if (currentNode == null) {
							currentNode = new TPNode(stop);
							intermediateNodes.put(stop, currentNode);
						}

						if (beforeNode != null) {
							// if this is not the first node we visit...
							if (currentNode != targetNode) {
								// ... and node is not target node, preventing cycles
								if (!beforeNode.hasPredecessor(currentNode, wasWalking)) {
									// ... and node link does not exist already
									beforeNode.addPredecessor(currentNode, wasWalking, delayClass);
									if (delayClass != null) {
										Calendar cal = Calendar.getInstance();
										cal.setTimeInMillis(state.getTimeInMillis());

										LOG.info("\t\t\tTP to " + target.getName() + ": new trip from "
												+ currentNode.getStop().getName() + " -> "
												+ beforeNode.getStop().getName() + (wasWalking ? " (walking)" : "")
												+ " at " + cal.get(Calendar.HOUR_OF_DAY) + ":"
												+ cal.get(Calendar.MINUTE) + " replacing delayed "
												+ delayClass.toString());
									}
								}
							}
						}

						beforeNode = currentNode;

						wasWalking = state.getBackMode() == TraverseMode.WALK;
					} else if (delayClass == null && v instanceof TransitStopDepart && lastTime > -1) {
						int waitTime = (int) ((lastTime - state.getTimeInMillis()) / 1000);
						if (waitTime > 0 && state.getNumBoardings() > 0) {
							// the wait time actually happened between two trips
							TripPattern lastPattern = state.getLastPattern();

							Integer currentDelay = possibleDelays.get(lastPattern);

							if (currentDelay == null || currentDelay < waitTime) {
								possibleDelays.put(lastPattern, waitTime);
							}
						}
					}
				}

				lastTime = state.getTimeInMillis();
				state = state.getBackState();
			}
		}

		// make sure we found a path (there's at least 2 nodes then)
		if (intermediateNodes.size() > 1)
			targets.put(target, targetNode);
	}

	/**
	 * @return all delays recorded during static TP creation
	 */
	public HashMap<TripPattern, Integer> getPossibleDelays() {
		return possibleDelays;
	}

	/**
	 * @return a transfer pattern with data recorded so far
	 */
	public TransferPattern create() {
		return new TransferPattern(root, targets);
	}
}
