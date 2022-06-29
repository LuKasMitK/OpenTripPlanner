package org.opentripplanner.routing.graph;

import java.io.Serializable;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import javax.annotation.Nullable;

import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.TransferPattern.TPNode;
import org.opentripplanner.routing.graph.TransferPattern.TPTravel;
import org.opentripplanner.routing.impl.TPLeg.TransitConnection;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Multimap;

/**
 * Contains TP graph data and provides source-to-target patterns
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPatternGraph implements Serializable {
	private static final long serialVersionUID = 2L;
	private static final Logger LOG = LoggerFactory.getLogger(TransferPatternGraph.class);

	private Multimap<TransitStationStop, TripPatternWithPosition> stopsToRoute;
	private Map<TransitStationStop, TransferPattern> transferPatterns;

	/**
	 * Creates new TP graph with given direct connection table and transfer
	 * patterns
	 * 
	 * @param stopsToRoute
	 *            direct connection information
	 * @param transferPatterns
	 *            transfer patterns
	 */
	public TransferPatternGraph(Multimap<TransitStationStop, TripPatternWithPosition> stopsToRoute,
			Map<TransitStationStop, TransferPattern> transferPatterns) {
		this.stopsToRoute = stopsToRoute;
		this.transferPatterns = transferPatterns;
	}

	/**
	 * Returns target node for given source-target combination
	 * 
	 * @param source
	 *            source of route
	 * @param target
	 *            target of the route
	 * @return target node
	 */
	public @Nullable TPNode getTransferPattern(TransitStationStop source, TransitStationStop target) {
		TransferPattern tp = transferPatterns.get(source);
		if (tp == null)
			return null;
		return tp.getTransferPattern(target);
	}

	/**
	 * @return all transfer patterns
	 */
	public Map<TransitStationStop, TransferPattern> getTransferPatterns() {
		return transferPatterns;
	}

	/**
	 * @return all direct connections
	 */
	public Multimap<TransitStationStop, TripPatternWithPosition> getDirectConnections() {
		return stopsToRoute;
	}

	/**
	 * Retrieves list of transit connections for given pairs of stops
	 * 
	 * @param from
	 *            stop that the trip starts with
	 * @param to
	 *            stop that the trip ends with
	 * @return direct connections
	 */
	public @Nullable List<TransitConnection> getDirectConnections(TransitStationStop from, TransitStationStop to) {
		Collection<TripPatternWithPosition> tripsFrom = stopsToRoute.get(from);
		Collection<TripPatternWithPosition> tripsTo = stopsToRoute.get(to);

		List<TransitConnection> directConnections = new LinkedList<>();

		for (TripPatternWithPosition tripFrom : tripsFrom) {
			for (TripPatternWithPosition tripTo : tripsTo) {
				if (tripFrom.tripPattern == tripTo.tripPattern && tripFrom.position < tripTo.position) {
					TransitConnection conn = new TransitConnection(tripFrom.tripPattern, tripFrom.position,
							tripTo.position);
					directConnections.add(conn);
				}
			}
		}

		return directConnections;
	}

	/**
	 * Computes and prints statistics of this transfer pattern. Watch out: after
	 * calling this method, the graph is unusable
	 */
	public void countDynamicPatterns() {
		LOG.info("Counting TP arcs...");

		CounterStats stats = new CounterStats();

		for (TransferPattern tp : transferPatterns.values()) {
			for (TPNode target : tp.getTargetNodes()) {
				countDynamicPatterns(target, stats);
			}

			stats.totalStops++;
		}

		LOG.info(stats.totalStops + " stops, " + stats.totalArcs + " arcs in total, of which " + stats.walkingArcs
				+ " walking arcs and " + stats.dynamicArcs + " dynamic arcs of which " + stats.dynamicWalkingArcs
				+ " walking");
	}

	private static void countDynamicPatterns(TPNode current, CounterStats stats) {
		for (TPTravel travel : current) {
			stats.totalArcs++;
			if (travel.isWalking())
				stats.walkingArcs++;
			if (travel.getDelayClass() != null) {
				stats.dynamicArcs++;
				if (travel.isWalking())
					stats.dynamicWalkingArcs++;
			}

			TPNode before = travel.getNode();
			countDynamicPatterns(before, stats);
		}

		// remove all arcs, so we don't count double
		current.getPredecessors().clear();
	}

	/**
	 * Data class for creating statistics
	 */
	private static class CounterStats {
		public int dynamicArcs = 0;
		public int dynamicWalkingArcs = 0;
		public int walkingArcs = 0;
		public int totalArcs = 0;

		public int totalStops = 0;
	}

	/**
	 * Data class containing a line and a position in the line
	 */
	public static class TripPatternWithPosition implements Serializable {
		private static final long serialVersionUID = 1L;

		public TripPattern tripPattern;
		public int position;

		public TripPatternWithPosition(TripPattern tripPattern, int position) {
			this.tripPattern = tripPattern;
			this.position = position;
		}
	}
}
