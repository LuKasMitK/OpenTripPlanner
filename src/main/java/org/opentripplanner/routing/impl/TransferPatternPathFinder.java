package org.opentripplanner.routing.impl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;
import java.util.TimeZone;

import javax.annotation.Nullable;

import org.onebusaway.gtfs.model.calendar.ServiceDate;
import org.opentripplanner.common.geometry.SphericalDistanceLibrary;
import org.opentripplanner.common.model.T2;
import org.opentripplanner.routing.algorithm.AStar;
import org.opentripplanner.routing.algorithm.strategies.EuclideanRemainingWeightHeuristic;
import org.opentripplanner.routing.core.RoutingContext;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.ServiceDay;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseModeSet;
import org.opentripplanner.routing.edgetype.Timetable;
import org.opentripplanner.routing.edgetype.TimetableSnapshot;
import org.opentripplanner.routing.edgetype.TransitBoardAlight;
import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.error.PathNotFoundException;
import org.opentripplanner.routing.error.VertexNotFoundException;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.TransferPattern.DelayClassification;
import org.opentripplanner.routing.graph.TransferPattern.TPNode;
import org.opentripplanner.routing.graph.TransferPattern.TPTravel;
import org.opentripplanner.routing.graph.TransferPatternGraph;
import org.opentripplanner.routing.graph.TransferPatternGraphBuilder.SimpleSnapshotSource;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.TPLeg.TransitConnection;
import org.opentripplanner.routing.impl.TPLeg.WalkingConnection;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.trippattern.TripTimes;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.opentripplanner.routing.vertextype.TransitStop;
import org.opentripplanner.standalone.Router;

import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;

import jersey.repackaged.com.google.common.collect.Sets;

/**
 * Finds routes using transfer patterns
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPatternPathFinder {
	private final Graph graph;
	private final TransferPatternGraph tpGraph;

	private final HashMap<T2<Vertex, Vertex>, WalkingPathBucket> walkingPathCache;

	/**
	 * Creates a new path finder with given router
	 * 
	 * @param router
	 *            router to use
	 */
	public TransferPatternPathFinder(Router router) {
		graph = router.graph;
		this.tpGraph = router.graph.tp;

		walkingPathCache = new HashMap<>();
	}

	/**
	 * Computes and returns journeys for given routing request
	 * 
	 * @param req
	 *            routing request to compute journeys for
	 * @return list of journeys
	 */
	public List<TPJourney> findJourneys(RoutingRequest req) {
		if (req.rctx == null) {
			req.setRoutingContext(graph);
		}

		Vertex sourceVertex;
		Vertex targetVertex;
		if (req.rctx.origin != null && req.rctx.target != null) {
			sourceVertex = req.rctx.origin;
			targetVertex = req.rctx.target;
		} else {
			sourceVertex = graph.streetIndex.getVertexForLocation(req.from, req, false);
			targetVertex = graph.streetIndex.getVertexForLocation(req.to, req, true);
		}

		Coordinate sourceCoord = sourceVertex.getCoordinate();
		Coordinate targetCoord = targetVertex.getCoordinate();

		if (sourceCoord == null || targetCoord == null) {
			List<String> list = new ArrayList<String>();
			if (sourceCoord == null)
				list.add(req.from.place);
			if (targetCoord == null)
				list.add(req.to.place);
			throw new VertexNotFoundException(list);
		}

		List<TransitStop> sourceStations = findClosestStops(graph, sourceVertex, req.maxWalkDistance);
		List<TransitStop> targetStations = findClosestStops(graph, targetVertex, req.maxWalkDistance);

		if (sourceStations.isEmpty() || targetStations.isEmpty())
			throw new PathNotFoundException();

		walkingPathCache.clear();

		TimeZone timeZone = graph.getTimeZone();
		final ServiceDay serviceDay = new ServiceDay(graph, req.dateTime, graph.getCalendarService(), timeZone);
		final int startSecs = serviceDay.secondsSinceMidnight(req.dateTime);

		final List<TPJourney> journeys = new LinkedList<>();

		for (TransitStop source : sourceStations) {
			for (TransitStop target : targetStations) {
				double startWalkLength = SphericalDistanceLibrary.distance(sourceVertex.getCoordinate(),
						source.getCoordinate());
				double endWalkLength = SphericalDistanceLibrary.distance(target.getCoordinate(),
						targetVertex.getCoordinate());

				// if walking distance in the beginning and end is already greater than max walk distance, skip
				if (startWalkLength + endWalkLength > req.maxWalkDistance)
					continue;

				TPNode targetNode = tpGraph.getTransferPattern(source, target);
				if (targetNode == null)
					continue;

				LinkedList<TPJourney> newJourneys = unfoldTransferPattern(targetNode);
				addWalkingBeginEnd(newJourneys, sourceVertex, targetVertex);
				fillDirectConnections(newJourneys, req, startSecs);

				journeys.addAll(newJourneys);
			}
		}

		List<TPJourney> result = filterConnections(journeys);
		Collections.sort(result, JourneySortComparator.getInstance());

		//journeys = journeys.subList(0, Math.min(journeys.size(), 3));

		return result;
	}

	/**
	 * Adds an artificial delay to given line for demonstration purposes
	 * 
	 * @param tripPatternName
	 *            line to delay
	 * @param graph
	 *            current graph
	 * @param serviceDay
	 *            day to delay the line on
	 * @param delay
	 *            amount of delay in seconds
	 */
	private static void addDemoDelay(String tripPatternName, Graph graph, ServiceDay serviceDay, int delay) {
		if (tripPatternName == null) {
			graph.timetableSnapshotSource = null;
			return;
		}

		List<TransitStop> allStopsList = new ArrayList<>();
		for (TransitStop s : Iterables.filter(graph.getVertices(), TransitStop.class)) {
			allStopsList.add(s);
		}

		// add delay such that next trip is missed
		delay += 2;

		TimetableSnapshot snapshot = new TimetableSnapshot();
		for (TransitStop stop : allStopsList) {
			for (Edge edge : stop.departVertex.getOutgoing()) {
				if (!(edge instanceof TransitBoardAlight))
					continue;
				TripPattern tripPattern = ((TransitBoardAlight) edge).getPattern();

				if (tripPattern.name.startsWith(tripPatternName)) {
					Timetable timetable = tripPattern.getUpdatedTimetable(null, null);

					for (TripTimes tripTimes : timetable.tripTimes) {
						TripTimes cloneTt = tripTimes.clone();
						for (int i = 0; i < tripTimes.getNumStops(); i++) {
							cloneTt.updateArrivalTime(i, tripTimes.getArrivalTime(i) + delay);
							cloneTt.updateDepartureTime(i, tripTimes.getDepartureTime(i) + delay);
						}

						snapshot.update(null, tripPattern, cloneTt, serviceDay.getServiceDate());
					}
				}
			}
		}

		graph.timetableSnapshotSource = new SimpleSnapshotSource(graph, snapshot.commit());
	}

	/**
	 * Finds closest public transport stops for given vertex
	 * 
	 * @param graph
	 *            graph to use
	 * @param point
	 *            vertex to search surrounding stops for
	 * @param radius
	 *            radius to search in
	 * @return list of stops
	 */
	protected static List<TransitStop> findClosestStops(Graph graph, Vertex point, double radius) {
		List<TransitStop> stops;
		if (point instanceof TransitStop) {
			stops = new ArrayList<>(1);
			stops.add((TransitStop) point);
		} else {
			// first search with 30m radius for stop in immediate proximity, then use regular radius, then add 50% to radius
			double[] radiuses = new double[] { 30, radius, radius * 1.5 };

			stops = new ArrayList<>();
			int i = 0;
			while (stops.isEmpty() && i < radiuses.length) {
				stops = graph.streetIndex.getNearbyTransitStops(point.getCoordinate(), radiuses[i]);
				i++;
			}
		}
		return stops;
	}

	/**
	 * Unfolds a given target node into journeys
	 * 
	 * @param targetNode
	 *            target node
	 * @return journeys
	 */
	private static LinkedList<TPJourney> unfoldTransferPattern(TPNode targetNode) {
		TPJourney journey = new TPJourney();
		return unfoldTransferPattern(journey, targetNode);
	}

	/**
	 * Recursive method that unfolds transfer patterns
	 * 
	 * @param journey
	 *            current journey
	 * @param current
	 *            current node
	 * @return list of journeys
	 */
	private static LinkedList<TPJourney> unfoldTransferPattern(TPJourney journey, TPNode current) {
		LinkedList<TPJourney> result = new LinkedList<>();

		if (current.getPredecessorCount() == 0) {
			// reached source, return
			result.add(journey);
		} else {
			for (TPTravel travel : current) {
				TPNode before = travel.getNode();

				TPLeg leg = new TPLeg(before.getStop(), current.getStop(), travel.isWalking(), travel.getDelayClass());

				TPJourney clonedJourney = new TPJourney(journey);
				clonedJourney.legs.addFirst(leg);

				result.addAll(unfoldTransferPattern(clonedJourney, before));
			}
		}

		return result;
	}

	/**
	 * Adds footpaths to journeys at beginning and end
	 * 
	 * @param journeys
	 *            journeys to attach footpaths to
	 * @param requestStart
	 *            requested source vertex
	 * @param requestEnd
	 *            requested target vertex
	 */
	private static void addWalkingBeginEnd(List<TPJourney> journeys, Vertex requestStart, Vertex requestEnd) {
		for (TPJourney journey : journeys) {
			final TPLeg firstLeg = journey.legs.getFirst();
			if (!firstLeg.getFromVertex().equals(requestStart)) {
				// if first leg is already a footpath, only change source
				if (firstLeg.isWalking()) {
					firstLeg.setFromVertex(requestStart);
				} else {
					TPLeg newFirstLeg = new TPLeg(requestStart, firstLeg.getFromVertex(), true, null);
					journey.legs.addFirst(newFirstLeg);
				}
			}

			final TPLeg lastLeg = journey.legs.getLast();
			if (!journey.legs.getLast().getToVertex().equals(requestEnd)) {
				// if last leg is already a footpath, only change target
				if (lastLeg.isWalking()) {
					lastLeg.setToVertex(requestEnd);
				} else {
					TPLeg newLastLeg = new TPLeg(lastLeg.getToVertex(), requestEnd, true, null);
					journey.legs.addLast(newLastLeg);
				}
			}
		}
	}

	/**
	 * Fills in concrete connection information for feasible routes and removes
	 * infeasible routes
	 * 
	 * @param journeys
	 *            journeys to fill direct connection info in
	 * @param req
	 *            routing request
	 * @param startTime
	 *            seconds from midnight
	 */
	private void fillDirectConnections(List<TPJourney> journeys, RoutingRequest req, int startTime) {
		ListIterator<TPJourney> itJourneys = journeys.listIterator();

		while (itJourneys.hasNext()) {
			TPJourney journey = itJourneys.next();
			try {
				fillDirectConnection(journey, req, startTime);
			} catch (Exception e) {
				itJourneys.remove();
			}
		}
	}

	/**
	 * Fills in concrete connection information for feasible routes and removes
	 * infeasible routes
	 * 
	 * @param journey
	 *            journey to fill direct connection info in
	 * @param req
	 *            routing request
	 * @param startTime
	 *            seconds from midnight
	 */
	private void fillDirectConnection(TPJourney journey, RoutingRequest req, int startTime) throws Exception {
		Iterator<TPLeg> itLegs = journey.legs.iterator();

		int lastTime = startTime;

		while (itLegs.hasNext()) {
			TPLeg leg = itLegs.next();
			Vertex one = leg.getFromVertex();
			Vertex two = leg.getToVertex();

			if (leg.isWalking()) {
				ServiceDay serviceDay = new ServiceDay(graph, req.dateTime, graph.getCalendarService(),
						graph.getTimeZone());

				WalkingPathBucket bucket = getWalkingPathBucket(one, two);
				GraphPath graphPath = bucket.getWalkingPath(graph, serviceDay.time(lastTime));

				if (graphPath == null) {
					// remove this journey if no path found
					throw new Exception();
				}

				leg.setWalkingConnection(new WalkingConnection(graphPath, serviceDay));
				lastTime += graphPath.getDuration();
			} else {
				// assume that the two stops are transit stops here
				TransitStationStop oneStop = (TransitStationStop) one;
				TransitStationStop twoStop = (TransitStationStop) two;

				if (leg.getDelayClass() != null) {
					ServiceDay serviceDay = new ServiceDay(graph, req.dateTime, graph.getCalendarService(),
							graph.getTimeZone());

					if (!isDelayClassApplicable(leg.getDelayClass(), req.rctx.timetableSnapshot,
							serviceDay.getServiceDate())) {
						throw new Exception();
					}
				}

				List<TransitConnection> connList = tpGraph.getDirectConnections(oneStop, twoStop);
				// already includes boarding time
				TransitConnection bestConnection = pickBestConnection(connList, req, lastTime);

				if (bestConnection == null) {
					// remove this journey if no trip found
					throw new Exception();
				}

				leg.setTransitConnection(bestConnection);

				int alightTime = req.getAlightTime(bestConnection.tripPattern.mode);
				lastTime = bestConnection.getArrivalSinceMidnight() + alightTime;
			}
		}

		// adjust the first walk now since it started too early
		Iterator<TPLeg> itConn = journey.legs.iterator();
		TPLeg first = itConn.next();
		if (first.isWalking() && itConn.hasNext()) {
			TPLeg second = itConn.next();
			long walkArrivalCurrent = first.getWalkingPath().getEndTime();
			long walkArrivalActual = second.getDeparture();
			long offset = walkArrivalActual - walkArrivalCurrent;

			first.setWalkingPath(new GraphPath(first.getWalkingPath(), offset));
		}
	}

	/**
	 * Checks whether given delay classification is fulfilled in current traffic
	 * situation
	 * 
	 * @param delayClass
	 *            classification to test
	 * @param snapshot
	 *            current delay data
	 * @param serviceDate
	 *            current date
	 */
	private static boolean isDelayClassApplicable(DelayClassification delayClass, TimetableSnapshot snapshot,
			ServiceDate serviceDate) {

		Iterator<TripPattern> itTrip = delayClass.getTripPatterns().iterator();
		Iterator<Integer> itDelay = delayClass.getMinDelay().iterator();

		while (itTrip.hasNext()) {
			TripPattern tripPattern = itTrip.next();
			Integer classDelay = itDelay.next();

			Timetable updatedTimetable = snapshot.resolve(tripPattern, serviceDate);

			int maxDelay = 0;

			if (tripPattern.scheduledTimetable != updatedTimetable) {
				// there are updates here
				for (TripTimes tripTimes : updatedTimetable.tripTimes) {
					for (int i = 0; i < tripTimes.getNumStops(); i++) {
						final int newDelay = tripTimes.getArrivalDelay(i);
						maxDelay = Math.max(maxDelay, newDelay);
					}
				}
			}

			if (maxDelay < classDelay)
				return false;
		}

		return true;
	}

	/**
	 * Computes Pareto set of given journeys using departure time, arrival time
	 * and number of transfers
	 * 
	 * @param journeys
	 *            journeys to filter
	 * @return Pareto set of journeys
	 */
	private static List<TPJourney> filterConnections(List<TPJourney> journeys) {
		ListIterator<TPJourney> itJourneys = journeys.listIterator();

		final Comparator<TPJourney> comparator = JourneyParetoComparator.getInstance();
		List<TPJourney> paretoSet = new LinkedList<>();

		if (!itJourneys.hasNext())
			return paretoSet;

		paretoSet.add(itJourneys.next());

		while (itJourneys.hasNext()) {
			TPJourney potential = itJourneys.next();

			ListIterator<TPJourney> itCurrent = paretoSet.listIterator();
			boolean add = true;

			while (itCurrent.hasNext()) {
				TPJourney current = itCurrent.next();
				final int comp = comparator.compare(potential, current);

				if (comp < 0)
					add = false;
				else if (comp > 0 || current.equals(potential))
					itCurrent.remove();
			}

			if (add)
				paretoSet.add(potential);
		}

		return paretoSet;
	}

	/**
	 * Picks best connection (earliest possible) from list
	 * 
	 * @param connList
	 *            list of connections
	 * @param req
	 *            routing request
	 * @param startTime
	 *            seconds from midnight
	 */
	private TransitConnection pickBestConnection(List<TransitConnection> connList, RoutingRequest req, int startTime) {
		TransitConnection best = null;

		for (TransitConnection conn : connList) {
			ServiceDay serviceDay = new ServiceDay(graph, req.dateTime, graph.getCalendarService(),
					conn.tripPattern.route.getAgency().getId());
			Timetable timeTable = conn.tripPattern.getUpdatedTimetable(req, serviceDay);
			TripTimes tripTimes = timeTable.getNextTrip(req, startTime, serviceDay, conn.fromPos, true);

			if (tripTimes == null)
				continue;

			conn.setConcreteConnection(tripTimes, serviceDay);

			if (best == null || conn.getDepartureSinceMidnight() < best.getDepartureSinceMidnight())
				best = conn;
		}

		return best;
	}

	/**
	 * Gets a walking path cache bucket for given vertices
	 * 
	 * @param from
	 *            footpath source
	 * @param to
	 *            footpath target
	 * @return walking path bucket (either cached or new one)
	 */
	private WalkingPathBucket getWalkingPathBucket(Vertex from, Vertex to) {
		T2<Vertex, Vertex> key = new T2<>(from, to);
		WalkingPathBucket bucket = walkingPathCache.get(key);

		if (bucket == null) {
			bucket = new WalkingPathBucket(from, to);
			walkingPathCache.put(key, bucket);
		}

		return bucket;
	}

	/**
	 * Cache class for walking path bucket. Lazily returns actual paths
	 */
	private static class WalkingPathBucket {
		private static final int timeOffset = 12 * 60 * 60;

		private Vertex from;
		private Vertex to;

		private GraphPath walkingPath = null;

		/**
		 * Creates new bucket with given vertices
		 * 
		 * @param from
		 *            source of walking path
		 * @param to
		 *            target of walking path
		 */
		public WalkingPathBucket(Vertex from, Vertex to) {
			this.from = from;
			this.to = to;
		}

		/**
		 * Returns walking path starting at given time
		 * 
		 * @param graph
		 *            graph to use
		 * @param time
		 *            time in seconds since epoch
		 * @return footpath
		 */
		public GraphPath getWalkingPath(Graph graph, long time) {
			if (walkingPath == null) {
				walkingPath = getWalkingRoute(graph, from, to, timeOffset);
			}

			if (walkingPath != null) {
				return new GraphPath(walkingPath, time - timeOffset);
			}
			return null;
		}

		/**
		 * Actually computes walking path
		 * 
		 * @param graph
		 *            grapht to use
		 * @param from
		 *            source vertex
		 * @param to
		 *            target vertex
		 * @param dateTime
		 *            time in seconds since epoch
		 * @return footpath or null on failure
		 */
		private static @Nullable GraphPath getWalkingRoute(Graph graph, Vertex from, Vertex to, long dateTime) {
			RoutingRequest req = new RoutingRequest();
			req.modes = new TraverseModeSet(TraverseMode.WALK);
			req.dateTime = dateTime;

			req.ignoreRealtimeUpdates = true;
			req.useTraffic = false;

			req.rctx = new RoutingContext(req, graph, from, to);
			req.rctx.remainingWeightHeuristic = new EuclideanRemainingWeightHeuristic();

			AStar aStar = new AStar();
			// start search
			aStar.getShortestPathTree(req);
			// get paths
			List<GraphPath> paths = aStar.getPathsToTarget();

			if (paths.size() > 0)
				return paths.get(0);
			else
				return null;
		}
	}

	/**
	 * Compares journeys in terms of pareto equality
	 */
	private static class JourneyParetoComparator implements Comparator<TPJourney> {
		private static JourneyParetoComparator instance;

		public static JourneyParetoComparator getInstance() {
			if (instance == null)
				instance = new JourneyParetoComparator();

			return instance;
		}

		private JourneyParetoComparator() {}

		@Override
		public int compare(TPJourney o1, TPJourney o2) {
			if (!o1.hasLegs())
				return !o2.hasLegs() ? 0 : -1;
			if (!o2.hasLegs())
				return 1;

			// compute differences in arrival time, departure time and number of transfers and transform into usable values
			int valueArr = cut(o2.getArrival() - o1.getArrival());
			int valueDep = cut(o1.getDeparture() - o2.getDeparture());
			int valueTransfer = cut(o2.legsCount() - o1.legsCount());

			// create set of compare values and sum of all values
			Set<Integer> values = Sets.newHashSet(valueArr, valueDep, valueTransfer);
			final int sum = valueArr + valueDep + valueTransfer;

			// determine Pareto answer
			if (sum > 0) {
				if (values.contains(-1))
					return 0;
				return 1;
			} else if (sum < 0) {
				if (values.contains(1))
					return 0;
				return -1;
			}
			return 0;
		}

		private static int cut(long difference) {
			if (difference > 0)
				return 1;
			if (difference < 0)
				return -1;
			return 0;
		}

	}

	/**
	 * Sorts journeys by departure and arrival time for proper display
	 */
	private static class JourneySortComparator implements Comparator<TPJourney> {
		private static JourneySortComparator instance;

		public static JourneySortComparator getInstance() {
			if (instance == null)
				instance = new JourneySortComparator();

			return instance;
		}

		private JourneySortComparator() {}

		@Override
		public int compare(TPJourney o1, TPJourney o2) {
			if (o1.legs.isEmpty())
				return o2.legs.isEmpty() ? 0 : 1;
			if (o2.legs.isEmpty())
				return -1;

			int deltaArr = (int) (o1.legs.getLast().getArrival() - o2.legs.getLast().getArrival());

			if (deltaArr != 0)
				return deltaArr;

			return (int) (o1.getDuration() - o2.getDuration());
		}

	}
}
