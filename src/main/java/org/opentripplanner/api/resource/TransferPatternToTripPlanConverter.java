package org.opentripplanner.api.resource;

import java.util.Calendar;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.TimeZone;

import org.onebusaway.gtfs.model.Agency;
import org.onebusaway.gtfs.model.Route;
import org.onebusaway.gtfs.model.Stop;
import org.onebusaway.gtfs.model.Trip;
import org.opentripplanner.api.model.Itinerary;
import org.opentripplanner.api.model.Leg;
import org.opentripplanner.api.model.Place;
import org.opentripplanner.api.model.TripPlan;
import org.opentripplanner.api.model.VertexType;
import org.opentripplanner.api.model.WalkStep;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.ServiceDay;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.PatternHop;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.TPJourney;
import org.opentripplanner.routing.impl.TPLeg;
import org.opentripplanner.routing.impl.TPLeg.TransitConnection;
import org.opentripplanner.routing.location.TemporaryStreetLocation;
import org.opentripplanner.routing.trippattern.TripTimes;
import org.opentripplanner.routing.vertextype.BikeParkVertex;
import org.opentripplanner.routing.vertextype.BikeRentalStationVertex;
import org.opentripplanner.routing.vertextype.StreetVertex;
import org.opentripplanner.routing.vertextype.TransitVertex;
import org.opentripplanner.util.PolylineEncoder;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;

/**
 * Converts TP journeys into trip plan that can be used by OTP client. Uses
 * several methods of {@link GraphPathToTripPlanConverter}
 * 
 * @author Sebastian Peter
 *
 */
public abstract class TransferPatternToTripPlanConverter {

	/**
	 * Creates trip plan for given journeys
	 * 
	 * @param journeys
	 *            journeys to create trip plans for
	 * @param request
	 *            original routing request
	 * @return trip plan
	 */
	public static TripPlan generatePlan(List<TPJourney> journeys, RoutingRequest request) {
		if (journeys.isEmpty()) {
			return new TripPlan();
		}

		Locale requestedLocale = request.locale;

		TPJourney exemplar = journeys.get(0);
		Vertex origin = exemplar.legs.getFirst().getFromVertex();
		Vertex target = exemplar.legs.getLast().getToVertex();

		String startName = origin.getName(requestedLocale);
		String endName = target.getName(requestedLocale);

		// Use vertex labels if they don't have names
		if (startName == null)
			startName = origin.getLabel();

		if (endName == null)
			endName = target.getLabel();

		Place from = new Place(origin.getX(), origin.getY(), startName);
		Place to = new Place(target.getX(), target.getY(), endName);

		from.orig = request.from.name;
		to.orig = request.to.name;

		TripPlan plan = new TripPlan(from, to, request.getDateTime());

		for (TPJourney journey : journeys) {
			Itinerary itinerary = generateItinerary(journey, request, request.showIntermediateStops, requestedLocale);
			itinerary = adjustItinerary(request, itinerary);
			plan.addItinerary(itinerary);
		}
		for (Itinerary i : plan.itinerary) {
			Leg firstLeg = i.legs.get(0);
			firstLeg.from.orig = plan.from.orig;
			Leg lastLeg = i.legs.get(i.legs.size() - 1);
			lastLeg.to.orig = plan.to.orig;
		}

		return plan;
	}

	/**
	 * Generate an itinerary from a {@link TPJourney}
	 *
	 * @param journey
	 *            journey to generate itinerary for
	 * @param request
	 *            original routing request
	 * @param showIntermediateStops
	 *            Whether to include intermediate stops in the itinerary or not
	 * @param requestedLocale
	 *            locale
	 * @return The generated itinerary
	 */
	private static Itinerary generateItinerary(TPJourney journey, RoutingRequest request, boolean showIntermediateStops,
			Locale requestedLocale) {
		Itinerary itinerary = new Itinerary();

		Iterator<TPLeg> itConn = journey.legs.iterator();

		ServiceDay serviceDay = null;

		while (itConn.hasNext()) {
			TPLeg connection = itConn.next();

			Leg leg = generateLeg(connection, request, showIntermediateStops, requestedLocale);
			itinerary.addLeg(leg);

			if (serviceDay == null)
				serviceDay = connection.getServiceDay();
		}

		TPLeg firstConn = journey.legs.getFirst();
		TPLeg lastConn = journey.legs.getLast();

		itinerary.startTime = makeCalendar(request.rctx.graph.getTimeZone(), firstConn.getDeparture() * 1000);
		itinerary.endTime = makeCalendar(request.rctx.graph.getTimeZone(), lastConn.getArrival() * 1000);
		itinerary.duration = lastConn.getArrival() - firstConn.getDeparture();

		// TODO fare
		itinerary.fare = null;

		int startSecs = -1;
		if (serviceDay != null)
			startSecs = serviceDay.secondsSinceMidnight(request.dateTime);
		calculateTimes(itinerary, journey, startSecs);

		return itinerary;
	}

	/**
	 * Check whether itinerary needs adjustments based on the request.
	 * 
	 * @param itinerary
	 *            is the itinerary
	 * @param request
	 *            is the request containing the original trip planning options
	 * @return the (adjusted) itinerary
	 */
	private static Itinerary adjustItinerary(RoutingRequest request, Itinerary itinerary) {
		// Check walk limit distance
		if (itinerary.walkDistance > request.maxWalkDistance) {
			itinerary.walkLimitExceeded = true;
		}

		return itinerary;
	}

	/**
	 * Calculate the walkTime, transitTime and waitingTime of an
	 * {@link Itinerary}.
	 *
	 * @param itinerary
	 *            The itinerary to calculate the times for
	 * @param startSecs
	 */
	private static void calculateTimes(Itinerary itinerary, TPJourney journey, int startSecs) {
		int lastTime = startSecs;

		for (TPLeg leg : journey.legs) {
			if (leg.isWalking()) {
				State lastState = leg.getWalkingPath().states.getLast();

				itinerary.walkDistance += lastState.getWalkDistance();
				itinerary.walkTime += lastState.getTimeDeltaSeconds();
			} else {
				itinerary.transitTime += leg.getArrivalSinceMidnight() - leg.getDepartureSinceMidnight();
				if (lastTime > -1)
					itinerary.waitingTime += leg.getDepartureSinceMidnight() - lastTime;
				itinerary.transfers++;
			}

			lastTime = leg.getArrivalSinceMidnight();
		}
	}

	/**
	 * Generate one leg of an itinerary from a {@link State} array.
	 *
	 * @param showIntermediateStops
	 *            Whether to include intermediate stops in the leg or not
	 * @return The generated leg
	 */
	private static Leg generateLeg(TPLeg tpLeg, RoutingRequest request, boolean showIntermediateStops,
			Locale requestedLocale) {
		Graph graph = request.rctx.graph;

		Leg leg = new Leg();

		if (tpLeg.isWalking()) {
			State[] states = tpLeg.getWalkingPath().states.toArray(new State[tpLeg.getWalkingPath().states.size()]);

			leg = GraphPathToTripPlanConverter.generateLeg(graph, states, showIntermediateStops, requestedLocale);

			// add walking trip
			List<WalkStep> walkSteps = GraphPathToTripPlanConverter.generateWalkSteps(graph, states, null,
					requestedLocale);
			leg.walkSteps = walkSteps;
			leg.mode = TraverseMode.WALK.toString();
		} else {
			// add transit trip
			long depTime = tpLeg.getDeparture();
			long arrTime = tpLeg.getArrival();

			leg.startTime = makeCalendar(tpLeg.getAgency(), depTime * 1000);
			leg.endTime = makeCalendar(tpLeg.getAgency(), arrTime * 1000);

			leg.distance = calculateDistance(tpLeg);

			TimeZone timeZone = leg.startTime.getTimeZone();
			leg.agencyTimeZoneOffset = timeZone.getOffset(leg.startTime.getTimeInMillis());

			addTripFields(leg, tpLeg, requestedLocale);

			addPlaces(leg, tpLeg, showIntermediateStops, requestedLocale);

			CoordinateArrayListSequence coordinates = makeCoordinates(tpLeg.getTransitConnection());
			Geometry geometry = GeometryUtils.getGeometryFactory().createLineString(coordinates);
			leg.legGeometry = PolylineEncoder.createEncodings(geometry);

			// TODO interlining
			leg.interlineWithPreviousLeg = false;
			leg.rentedBike = false;

			// TODO alerts
			leg.mode = tpLeg.getTransitConnection().tripPattern.mode.toString();

			if (leg.isTransitLeg())
				addRealTimeData(leg, tpLeg);
		}

		return leg;
	}

	private static double calculateDistance(TPLeg leg) {
		double distance = 0.0;
		if (leg.isWalking()) {
			for (Edge edge : leg.getWalkingPath().edges) {
				distance += edge.getDistance();
			}
		} else {
			for (int i = leg.getTransitConnection().fromPos; i < leg.getTransitConnection().toPos; i++) {
				PatternHop edge = leg.getTransitConnection().tripPattern.hopEdges[i];
				distance += edge.getDistance();
			}
		}

		return distance;
	}

	/**
	 * Generate a {@link CoordinateArrayListSequence} based on an {@link Edge}
	 * array.
	 *
	 * @param edges
	 *            The array of input edges
	 * @return The coordinates of the points on the edges
	 */
	private static CoordinateArrayListSequence makeCoordinates(TransitConnection transitConnection) {
		CoordinateArrayListSequence coordinates = new CoordinateArrayListSequence();

		if (transitConnection.tripPattern == null)
			return coordinates;

		for (int i = transitConnection.fromPos; i < transitConnection.toPos; i++) {
			PatternHop edge = transitConnection.tripPattern.hopEdges[i];
			LineString geometry = edge.getGeometry();

			if (geometry != null) {
				if (coordinates.size() == 0) {
					coordinates.extend(geometry.getCoordinates());
				} else {
					coordinates.extend(geometry.getCoordinates(), 1); // Avoid duplications
				}
			}
		}

		return coordinates;
	}

	/**
	 * Add trip-related fields to a {@link Leg}.
	 *
	 * @param leg
	 *            The leg to add the trip-related fields to
	 * @param states
	 *            The states that go with the leg
	 */
	private static void addTripFields(Leg leg, TPLeg connection, Locale requestedLocale) {
		if (connection.isWalking())
			return;

		Trip trip = connection.getTripTimes().trip;

		Route route = trip.getRoute();
		Agency agency = route.getAgency();
		// TODO ServiceDay serviceDay = states[states.length - 1].getServiceDay();

		leg.agencyId = agency.getId();
		leg.agencyName = agency.getName();
		leg.agencyUrl = agency.getUrl();
		leg.headsign = trip.getTripHeadsign(); // FIXME ?
		leg.route = route.getLongName(); // FIXME ?
		leg.routeColor = route.getColor();
		leg.routeId = route.getId();
		leg.routeLongName = route.getLongName();
		leg.routeShortName = route.getShortName();
		leg.routeTextColor = route.getTextColor();
		leg.routeType = route.getType();
		leg.tripId = trip.getId();
		leg.tripShortName = trip.getTripShortName();
		leg.tripBlockId = trip.getBlockId();

		if (connection.getServiceDay() != null) {
			leg.serviceDate = connection.getServiceDay().getServiceDate().getAsString();
		}

		if (leg.headsign == null) {
			leg.headsign = trip.getTripHeadsign();
		}

	}

	/**
	 * Add {@link Place} fields to a {@link Leg}. There is some code duplication
	 * because of subtle differences between departure, arrival and intermediate
	 * stops.
	 *
	 * @param leg
	 *            The leg to add the places to
	 * @param states
	 *            The states that go with the leg
	 * @param edges
	 *            The edges that go with the leg
	 * @param showIntermediateStops
	 *            Whether to include intermediate stops in the leg or not
	 */
	private static void addPlaces(Leg leg, TPLeg tpLeg, boolean showIntermediateStops, Locale requestedLocale) {
		Vertex firstVertex = tpLeg.getFromVertex();
		Vertex lastVertex = tpLeg.getToVertex();

		Stop firstStop = firstVertex instanceof TransitVertex ? ((TransitVertex) firstVertex).getStop() : null;
		Stop lastStop = lastVertex instanceof TransitVertex ? ((TransitVertex) lastVertex).getStop() : null;

		leg.from = makePlace(firstVertex, firstStop, tpLeg, tpLeg.getTransitConnection().fromPos, requestedLocale);
		leg.from.arrival = null;
		leg.to = makePlace(lastVertex, lastStop, tpLeg, tpLeg.getTransitConnection().toPos, requestedLocale);
		leg.to.departure = null;

		if (showIntermediateStops) {
			// TODO
		}
	}

	/**
	 * Make a {@link Place} to add to a {@link Leg}.
	 *
	 * @param state
	 *            The {@link State} that the {@link Place} pertains to.
	 * @param vertex
	 *            The {@link Vertex} at the {@link State}.
	 * @param edge
	 *            The {@link Edge} leading out of the {@link Vertex}.
	 * @param stop
	 *            The {@link Stop} associated with the {@link Vertex}.
	 * @param tripTimes
	 *            The {@link TripTimes} associated with the {@link Leg}.
	 * @return The resulting {@link Place} object.
	 */
	private static Place makePlace(Vertex vertex, Stop stop, TPLeg connection, int stopIndex, Locale requestedLocale) {
		TripTimes tripTimes = connection.getTripTimes();
		Agency agency = connection.getAgency();

		String name = vertex.getName(requestedLocale);

		// This gets nicer names instead of osm:node:id when changing mode of
		// transport
		// Names are generated from all the streets in a corner, same as names
		// in origin and destination
		// We use name in TemporaryStreetLocation since this name generation
		// already happened when temporary location was generated
		if (vertex instanceof StreetVertex && !(vertex instanceof TemporaryStreetLocation)) {
			name = ((StreetVertex) vertex).getIntersectionName(requestedLocale).toString(requestedLocale);
		}

		int depTimeSecs = tripTimes.getDepartureTime(stopIndex);
		int arrTimeSecs = tripTimes.getArrivalTime(stopIndex);

		long depTime = connection.getServiceDay().time(depTimeSecs) * 1000;
		long arrTime = connection.getServiceDay().time(arrTimeSecs) * 1000;

		Place place = new Place(vertex.getX(), vertex.getY(), name, makeCalendar(agency, depTime),
				makeCalendar(agency, arrTime));

		if (vertex instanceof TransitVertex && !connection.isWalking()) {
			place.stopId = stop.getId();
			place.stopCode = stop.getCode();
			place.platformCode = stop.getPlatformCode();
			place.zoneId = stop.getZoneId();
			place.stopIndex = stopIndex;
			if (tripTimes != null)
				place.stopSequence = tripTimes.getStopSequence(place.stopIndex);

			place.vertexType = VertexType.TRANSIT;
		} else if (vertex instanceof BikeRentalStationVertex) {
			place.vertexType = VertexType.BIKESHARE;
		} else if (vertex instanceof BikeParkVertex) {
			place.vertexType = VertexType.BIKEPARK;
		} else {
			place.vertexType = VertexType.NORMAL;
		}

		return place;
	}

	/**
	 * Add information about real-time data to a {@link Leg}.
	 *
	 * @param leg
	 *            The leg to add the real-time information to
	 * @param states
	 *            The states that go with the leg
	 */
	private static void addRealTimeData(Leg leg, TPLeg connection) {
		TripTimes tripTimes = connection.getTripTimes();

		if (tripTimes != null && !tripTimes.isScheduled()) {
			leg.realTime = true;
			if (leg.from.stopIndex != null) {
				leg.departureDelay = tripTimes.getDepartureDelay(leg.from.stopIndex);
			}
			leg.arrivalDelay = tripTimes.getArrivalDelay(leg.to.stopIndex);
		}
	}

	public static Calendar makeCalendar(Agency agency, long time) {
		TimeZone timeZone = TimeZone.getTimeZone(agency.getTimezone());
		return makeCalendar(timeZone, time);
	}

	public static Calendar makeCalendar(TimeZone timeZone, long time) {
		Calendar calendar = Calendar.getInstance(timeZone);
		calendar.setTimeInMillis(time);
		return calendar;
	}
}
