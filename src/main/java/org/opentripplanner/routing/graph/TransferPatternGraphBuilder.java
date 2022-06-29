package org.opentripplanner.routing.graph;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.GregorianCalendar;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Set;

import org.onebusaway.gtfs.model.calendar.ServiceDate;
import org.opentripplanner.routing.algorithm.AStar;
import org.opentripplanner.routing.algorithm.strategies.InterleavedBidirectionalHeuristic;
import org.opentripplanner.routing.algorithm.strategies.MultiTargetTerminationStrategy;
import org.opentripplanner.routing.algorithm.strategies.SearchTerminationStrategy;
import org.opentripplanner.routing.core.RoutingContext;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseModeSet;
import org.opentripplanner.routing.edgetype.TimetableSnapshot;
import org.opentripplanner.routing.edgetype.TransitBoardAlight;
import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.TransferPattern.DelayClassification;
import org.opentripplanner.routing.graph.TransferPatternDelayBuilder.SimpleDelayBuilder;
import org.opentripplanner.routing.graph.TransferPatternGraph.TripPatternWithPosition;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.routing.trippattern.TripTimes;
import org.opentripplanner.routing.vertextype.PatternArriveVertex;
import org.opentripplanner.routing.vertextype.PatternDepartVertex;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.opentripplanner.routing.vertextype.TransitStop;
import org.opentripplanner.updater.stoptime.TimetableSnapshotSource;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

/**
 * Builds transfer pattern graphs given an OTP graph
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPatternGraphBuilder {
	private static final Logger LOG = LoggerFactory.getLogger(TransferPatternGraphBuilder.class);

	/**
	 * Delay times filter in seconds
	 */
	private static final int DEPART_TIMES_FILTER = 60 * 30;

	private static final TransferPatternDelayBuilder DELAY_BUILDER = new SimpleDelayBuilder();

	/**
	 * Builds transfer patterns for all stops
	 * 
	 * @param graph
	 *            graph to use as a basis
	 * @return transfer patterns
	 */
	public static TransferPatternGraph buildGraph(Graph graph) {
		return buildPartialGraph(graph, 1, 1);
	}

	/**
	 * Builds the nth chunk of transfer patterns of m chunks in total
	 * 
	 * @param graph
	 *            graph to use as a basis
	 * @param n
	 *            number of chunk, starting with 1
	 * @param m
	 *            total number of chunks
	 * @return transfer patterns
	 */
	public static TransferPatternGraph buildPartialGraph(Graph graph, int n, int m) {
		long startTime = System.currentTimeMillis();

		Set<TripPattern> tripPatterns = Sets.newHashSet();
		for (PatternArriveVertex pav : Iterables.filter(graph.getVertices(), PatternArriveVertex.class)) {
			tripPatterns.add(pav.getTripPattern());
		}

		List<TransitStationStop> allStopsList = new ArrayList<>();
		for (TransitStationStop s : Iterables.filter(graph.getVertices(), TransitStationStop.class)) {
			allStopsList.add(s);
		}
		Collections.sort(allStopsList, new Comparator<TransitStationStop>() {
			@Override
			public int compare(TransitStationStop o1, TransitStationStop o2) {
				return o1.getIndex() - o2.getIndex();
			}
		});

		// take subset from n-1/m to n/m
		final int total = allStopsList.size();
		int startIndex = total * (n - 1) / m;
		int endBeforeIndex = total * n / m;

		Set<TransitStationStop> subsetStops = Sets.newHashSet(allStopsList.subList(startIndex, endBeforeIndex));
		Set<TransitStationStop> allStops = Sets.newHashSet(allStopsList);

		Multimap<TransitStationStop, TripPatternWithPosition> stopsToRoute = HashMultimap.create(allStopsList.size(),
				10);
		if (n == 1) {
			LOG.info("\tBuilding direct connection table...");
			computeDirectConnections(tripPatterns, stopsToRoute);
		}

		LOG.info("Building " + subsetStops.size() + " transfer patterns " + startIndex + " to " + endBeforeIndex
				+ " ...");
		Map<TransitStationStop, TransferPattern> transferPatterns = new HashMap<>(subsetStops.size());
		computeTransferPatterns(graph, subsetStops, allStops, transferPatterns);

		long endTime = System.currentTimeMillis();
		LOG.info(String.format("Done. Building transfer patterns took %.1f minutes.",
				(endTime - startTime) / 1000 / 60.0));

		return new TransferPatternGraph(stopsToRoute, transferPatterns);
	}

	/**
	 * Computes direct connections using given trip patterns
	 * 
	 * @param tripPatterns
	 *            all trip patterns
	 * @param stopsToRoute
	 *            map to store direct connections in
	 */
	private static void computeDirectConnections(Set<TripPattern> tripPatterns,
			Multimap<TransitStationStop, TripPatternWithPosition> stopsToRoute) {
		for (TripPattern tripPattern : tripPatterns) {
			for (int i = 0; i < tripPattern.stopVertices.length; i++) {
				TransitStop tstop = tripPattern.stopVertices[i];

				TripPatternWithPosition tpp = new TripPatternWithPosition(tripPattern, i);
				stopsToRoute.put(tstop, tpp);
			}
		}
	}

	/**
	 * Computes transfer patterns starting in given source stops
	 * 
	 * @param graph
	 *            graph to use as a basis
	 * @param sourceStops
	 *            set of stops that the transfer patterns originate in
	 * @param allStops
	 *            all stops of graph
	 * @param transferPatterns
	 *            map to store transfer patterns in
	 */
	private static void computeTransferPatterns(Graph graph, Set<TransitStationStop> sourceStops,
			Set<TransitStationStop> allStops, Map<TransitStationStop, TransferPattern> transferPatterns) {
		AStar aStar = new AStar();

		int i = 1;

		Calendar cal = Calendar.getInstance();
		cal.set(Calendar.DAY_OF_WEEK, Calendar.MONDAY);
		cal.set(Calendar.HOUR_OF_DAY, 0);
		cal.set(Calendar.MINUTE, 0);
		cal.set(Calendar.SECOND, 0);
		cal.set(Calendar.MILLISECOND, 0);

		// midnight, Monday this week, in seconds
		final long midnight = cal.getTimeInMillis() / 1000;

		LOG.info("\t\tUsing delay builder " + DELAY_BUILDER.getClass().getSimpleName());

		for (TransitStationStop root : sourceStops) {
			if (!(root instanceof TransitStop)) {
				LOG.warn("Stop " + root.getName() + " is not of type TransitStop and thus ignored!");
				continue;
			}

			TransferPatternEditor editor = new TransferPatternEditor(root, allStops);

			LOG.info("\t\t" + i++ + ": " + root.getName());

			List<Integer> departTimes = getDepartTimes((TransitStop) root, DEPART_TIMES_FILTER);
			// clearing delays
			graph.timetableSnapshotSource = null;

			for (int depart : departTimes) {
				RoutingRequest req = createTPRoutingRequest(root, graph, false);

				req.dateTime = midnight + depart;

				SearchTerminationStrategy terminationStrategy = new MultiTargetTerminationStrategy(
						new HashSet<>(allStops));
				ShortestPathTree spt = aStar.getShortestPathTree(req, -1, terminationStrategy);

				if (spt != null) {
					editor.add(spt, null);
				}
			}

			// Delays
			{
				final HashMap<TripPattern, Integer> possibleDelays = editor.getPossibleDelays();
				final List<DelayClassification> delayClasses = DELAY_BUILDER.getDelays(possibleDelays);

				LOG.info("\t\t Building delayed patterns with " + delayClasses.size() + " delay classes.");

				long midDay = midnight + 60 * 60 * 12;
				Calendar calendar = GregorianCalendar.getInstance();
				calendar.setTimeInMillis(midDay * 1000);
				final ServiceDate serviceDate = new ServiceDate(calendar);

				for (DelayClassification delayClass : delayClasses) {
					TimetableSnapshot snapshot = delayClass.toTimeTableSnapshot(serviceDate);
					graph.timetableSnapshotSource = new SimpleSnapshotSource(graph, snapshot.commit());

					for (int depart : departTimes) {
						RoutingRequest req = createTPRoutingRequest(root, graph, true);
						req.dateTime = depart;

						SearchTerminationStrategy terminationStrategy = new MultiTargetTerminationStrategy(
								new HashSet<>(allStops));
						ShortestPathTree spt = aStar.getShortestPathTree(req, -1, terminationStrategy);

						if (spt != null) {
							editor.add(spt, delayClass);
						}
					}
				}
			}
			// END Delays

			transferPatterns.put(root, editor.create());
		}
	}

	/**
	 * Creates a fresh routing request for a one-to-all query starting in given
	 * source stop
	 * 
	 * @param source
	 *            stop that the search originates in
	 * @param graph
	 *            graph to traverse
	 * @param delay
	 *            whether dynamic patterns are computed
	 * @return new routing request
	 */
	private static RoutingRequest createTPRoutingRequest(TransitStationStop source, Graph graph, boolean delay) {
		RoutingRequest req = new RoutingRequest();
		req.maxTransfers = 2;
		req.modes = new TraverseModeSet(TraverseMode.TRANSIT, TraverseMode.WALK);
		req.maxWalkDistance = 500;
		req.softWalkLimiting = false;

		// set for one-to-all search
		req.numItineraries = 1;
		req.batch = true;
		// real-time data only used in dynamic transfer patterns
		req.ignoreRealtimeUpdates = !delay;
		req.useTraffic = delay;

		req.rctx = new RoutingContext(req, graph, source, null);
		req.rctx.remainingWeightHeuristic = new InterleavedBidirectionalHeuristic();

		return req;
	}

	/**
	 * Computes all departure times for a given stop that are at least minDelta
	 * seconds apart
	 * 
	 * @param stop
	 *            stop to compute departure times for
	 * @param minDelta
	 *            minimal time that departures need to be apart
	 * @return list of departure times in seconds since midnight
	 */
	private static List<Integer> getDepartTimes(TransitStop stop, int minDelta) {
		List<Integer> departTimes = new LinkedList<Integer>();

		Collection<Edge> outEdges = stop.departVertex.getOutgoing();
		for (Edge out : outEdges) {
			if (!(out instanceof TransitBoardAlight))
				continue;

			final int stopIndex = ((TransitBoardAlight) out).getStopIndex();

			Vertex to = out.getToVertex();
			if (!(to instanceof PatternDepartVertex))
				continue;

			List<TripTimes> tripTimesList = ((PatternDepartVertex) to).getTripPattern().scheduledTimetable.tripTimes;

			for (TripTimes tripTimes : tripTimesList)
				departTimes.add(tripTimes.getScheduledDepartureTime(stopIndex));
		}

		Collections.sort(departTimes);

		int lastTime = -minDelta;

		ListIterator<Integer> it = departTimes.listIterator();
		while (it.hasNext()) {
			int time = it.next();

			if (time - lastTime < minDelta)
				it.remove();
			else
				lastTime = time;
		}

		return departTimes;
	}

	/**
	 * Simple source for artificial delay data
	 */
	public static class SimpleSnapshotSource extends TimetableSnapshotSource {
		private final TimetableSnapshot snapshot;

		public SimpleSnapshotSource(Graph graph, TimetableSnapshot snapshot) {
			super(graph);
			this.snapshot = snapshot;
		}

		@Override
		public TimetableSnapshot getTimetableSnapshot() {
			return snapshot;
		}
	}
}