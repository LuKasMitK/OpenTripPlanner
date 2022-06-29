package org.opentripplanner.routing.impl;

import org.onebusaway.gtfs.model.Agency;
import org.opentripplanner.routing.core.ServiceDay;
import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.TransferPattern.DelayClassification;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.trippattern.TripTimes;
import java.util.Objects;

/**
 * a connection between two vertexes, either walking or transit.
 * 
 * @author Sebastian Peter
 *
 */
public class TPLeg {
	private Vertex fromVertex, toVertex;
	private final boolean walking;
	private final DelayClassification delayClass;

	private TransitConnection transitConnection;
	private WalkingConnection walkingConnection;

	public TPLeg(Vertex from, Vertex to, boolean walking, DelayClassification delayClass) {
		fromVertex = from;
		toVertex = to;
		this.walking = walking;
		this.delayClass = delayClass;
	}

	public TPLeg(TPLeg leg) {
		// vertices are not cloned
		fromVertex = leg.fromVertex;
		toVertex = leg.toVertex;
		
		walking = leg.walking;
		delayClass = leg.delayClass;
		
		// walking paths and transits neither
		walkingConnection = leg.walkingConnection;
		transitConnection = leg.transitConnection;
	}

	public void setWalkingConnection(WalkingConnection walkingConnection) {
		this.walkingConnection = walkingConnection;
	}

	public WalkingConnection getWalkingConnection() {
		return walkingConnection;
	}

	public void setTransitConnection(TransitConnection transitConnection) {
		this.transitConnection = transitConnection;
	}

	public TransitConnection getTransitConnection() {
		return transitConnection;
	}

	public void setWalkingPath(GraphPath walkingPath) {
		this.walkingConnection.path = walkingPath;
	}

	/**
	 * @return the walkingPath
	 */
	public GraphPath getWalkingPath() {
		if (walkingConnection == null)
			return null;
		return walkingConnection.path;
	}

	/**
	 * @return the serviceDay
	 */
	public ServiceDay getServiceDay() {
		if (walkingConnection != null)
			return walkingConnection.serviceDay;
		else if (transitConnection != null)
			return transitConnection.serviceDay;

		return null;
	}

	/**
	 * @return the tripTimes
	 */
	public TripTimes getTripTimes() {
		if (transitConnection == null)
			return null;
		return transitConnection.tripTimes;
	}

	/**
	 * @return departure time in seconds since midnight
	 */
	public int getDepartureSinceMidnight() {
		if (walkingConnection != null)
			return walkingConnection.getDepartureSinceMidnight();
		else if(transitConnection != null)
			return transitConnection.getDepartureSinceMidnight();

		return -1;
	}

	/**
	 * @return arrival time in seconds since midnight
	 */
	public int getArrivalSinceMidnight() {
		if (walkingConnection != null)
			return walkingConnection.getArrivalSinceMidnight();
		else if(transitConnection != null)
			return transitConnection.getArrivalSinceMidnight();

		return -1;
	}

	/**
	 * @return departure time in seconds since epoch
	 */
	public long getDeparture() {
		if (walkingConnection != null)
			return walkingConnection.getDeparture();
		else if(transitConnection != null)
			return transitConnection.getDeparture();

		return -1;
	}

	/**
	 * @return arrival time in seconds since epoch
	 */
	public long getArrival() {
		if (walkingConnection != null)
			return walkingConnection.getArrival();
		else if(transitConnection != null)
			return transitConnection.getArrival();

		return -1;
	}

	public boolean hasConcreteConnection() {
		return transitConnection != null || walkingConnection != null;
	}

	public Agency getAgency() {
		if (transitConnection == null)
			return null;

		return transitConnection.tripPattern.route.getAgency();
	}

	public boolean isWalking() {
		return walking;
	}

	/**
	 * @param fromVertex the fromVertex to set
	 */
	public void setFromVertex(Vertex fromVertex) {
		this.fromVertex = fromVertex;
	}

	/**
	 * @return the fromVertex
	 */
	public Vertex getFromVertex() {
		return fromVertex;
	}

	/**
	 * @param toVertex the toVertex to set
	 */
	public void setToVertex(Vertex toVertex) {
		this.toVertex = toVertex;
	}

	/**
	 * @return the toVertex
	 */
	public Vertex getToVertex() {
		return toVertex;
	}

	/**
	 * @return the delayClass
	 */
	public DelayClassification getDelayClass() {
		return delayClass;
	}

	public static class TransitConnection {
		public TripPattern tripPattern;
		public int fromPos, toPos;

		private TripTimes tripTimes;
		private ServiceDay serviceDay;

		public TransitConnection(TripPattern tripPattern, int fromPos, int toPos) {
			this.tripPattern = tripPattern;
			this.fromPos = fromPos;
			this.toPos = toPos;
		}

		public void setConcreteConnection(TripTimes tripTimes, ServiceDay serviceDay) {
			this.tripTimes = tripTimes;
			this.serviceDay = serviceDay;
		}

		/**
		 * @return departure time in seconds since midnight
		 */
		public int getDepartureSinceMidnight() {
			return tripTimes.getDepartureTime(fromPos);
		}

		/**
		 * @return arrival time in seconds since midnight
		 */
		public int getArrivalSinceMidnight() {
			return tripTimes.getArrivalTime(toPos);
		}

		/**
		 * @return departure time in seconds since epoch
		 */
		public long getDeparture() {
			int secs = tripTimes.getDepartureTime(fromPos);
			return serviceDay.time(secs);
		}

		/**
		 * @return arrival time in seconds since epoch
		 */
		public long getArrival() {
			int secs = tripTimes.getArrivalTime(toPos);
			return serviceDay.time(secs);
		}
	}

	public static class WalkingConnection {
		public ServiceDay serviceDay;

		public GraphPath path;

		public WalkingConnection(GraphPath path, ServiceDay serviceDay) {
			this.path = path;
			this.serviceDay = serviceDay;
		}

		/**
		 * @return departure time in seconds since midnight
		 */
		public int getDepartureSinceMidnight() {
			return serviceDay.secondsSinceMidnight(path.getStartTime());
		}

		/**
		 * @return arrival time in seconds since midnight
		 */
		public int getArrivalSinceMidnight() {
			return serviceDay.secondsSinceMidnight(path.getEndTime());
		}

		/**
		 * @return departure time in seconds since epoch
		 */
		public long getDeparture() {
			return path.getStartTime();
		}

		/**
		 * @return arrival time in seconds since epoch
		 */
		public long getArrival() {
			return path.getEndTime();
		}
	}
}