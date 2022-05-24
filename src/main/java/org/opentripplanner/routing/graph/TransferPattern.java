package org.opentripplanner.routing.graph;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.onebusaway.gtfs.model.calendar.ServiceDate;
import org.opentripplanner.routing.edgetype.Timetable;
import org.opentripplanner.routing.edgetype.TimetableSnapshot;
import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.trippattern.TripTimes;
import org.opentripplanner.routing.vertextype.TransitStationStop;

/**
 * 
 * Data class that holds patterns from one source stop to all remaining target
 * stops
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPattern implements Serializable {
	private static final long serialVersionUID = 1L;

	private TransitStationStop source;
	private HashMap<TransitStationStop, TPNode> targets;

	/**
	 * Creates a transfer pattern with given source stop and targets nodes
	 * 
	 * @param sourceStop
	 *            source stop of this pattern
	 * @param targets
	 *            map with stop -> target node, which link through to source
	 *            stop
	 */
	public TransferPattern(TransitStationStop sourceStop, HashMap<TransitStationStop, TPNode> targets) {
		this.source = sourceStop;
		this.targets = targets;
	}

	/**
	 * @return the source stop of this
	 */
	public TransitStationStop getSourceStop() {
		return source;
	}

	/**
	 * @param sourceStop
	 *            the source stop to set
	 */
	public void setSourceStop(TransitStationStop sourceStop) {
		source = sourceStop;
	}

	/**
	 * @param targetStop
	 *            the target stop to get the node for
	 * @return target node, or null if no path exists from source to target
	 */
	public @Nullable TPNode getTransferPattern(TransitStationStop targetStop) {
		return targets.get(targetStop);
	}

	/**
	 * @param newTargets
	 *            new map with stop -> target node, which link through to source
	 *            stop
	 */
	public void setMap(HashMap<TransitStationStop, TPNode> newTargets) {
		targets = newTargets;
	}

	/**
	 * @return all target nodes
	 */
	public Collection<TPNode> getTargetNodes() {
		return targets.values();
	}

	/**
	 * Encodes a single stop inside a transfer pattern. Links to other stops
	 * through {@link TPTravel}
	 * 
	 * @author Sebastian Peter
	 *
	 */
	public static class TPNode implements Iterable<TPTravel>, Serializable {
		private static final long serialVersionUID = 1L;

		private LinkedList<TPTravel> predecessors = new LinkedList<>();
		private TransitStationStop stop;

		/**
		 * @param stop
		 *            stop of this node
		 */
		public TPNode(TransitStationStop stop) {
			this.stop = stop;
		}

		/**
		 * @return the stop of this node
		 */
		public TransitStationStop getStop() {
			return stop;
		}

		/**
		 * @param stop
		 *            the stop of this node to set
		 */
		public void setStop(TransitStationStop stop) {
			this.stop = stop;
		}

		/**
		 * Adds link to given node with given properties
		 * 
		 * @param predecessor
		 *            node to link to
		 * @param walking
		 *            whether trip is footpath or not
		 * @param delayClass
		 *            classification of lines that were delayed in order for
		 *            this trip to exist.
		 */
		public void addPredecessor(TPNode predecessor, boolean walking, @Nullable DelayClassification delayClass) {
			predecessors.add(new TPTravel(predecessor, walking, delayClass));
		}

		/**
		 * Checks whether a precedent node with given properties exists
		 * 
		 * @param predecessor
		 *            preceding node to check for
		 * @param walking
		 *            whether link is walking
		 * @return whether such a link exists or not
		 */
		public boolean hasPredecessor(TPNode predecessor, boolean walking) {
			for (TPTravel travel : predecessors) {
				if (travel.isWalking == walking && travel.node.equals(predecessor))
					return true;
			}
			return false;
		}

		/**
		 * @return all preceding links
		 */
		public List<TPTravel> getPredecessors() {
			return predecessors;
		}

		/**
		 * @return number of preceding links
		 */
		public int getPredecessorCount() {
			return predecessors.size();
		}

		@Override
		public Iterator<TPTravel> iterator() {
			return predecessors.iterator();
		}

		public ListIterator<TPTravel> listIterator() {
			return predecessors.listIterator();
		}

		@Override
		public boolean equals(Object obj) {
			if (obj instanceof TPNode && obj != null) {
				TPNode second = (TPNode) obj;
				return stop.equals(second.stop);
			}
			return false;
		}
	}

	/**
	 * Encodes a link (or "travel") to another node. Traveling properties such
	 * as delay classes and whether travel is footpath are referenced here
	 * 
	 * @author Sebastian Peter
	 *
	 */
	public static class TPTravel implements Serializable {
		private static final long serialVersionUID = 1L;

		private TPNode node;
		private boolean isWalking = false;

		private DelayClassification delayClass = null;

		/**
		 * Creates a new travel link to given node
		 * 
		 * @param node
		 *            node that is traveled to
		 * @param isWalking
		 *            whether travel is footpath or not
		 * @param delayClass
		 *            delay classification that was utilized when creating this
		 *            link
		 */
		public TPTravel(TPNode node, boolean isWalking, @Nullable DelayClassification delayClass) {
			this.node = node;
			this.isWalking = isWalking;
			this.delayClass = delayClass;
		}

		/**
		 * @return the node that this links to
		 */
		public TPNode getNode() {
			return node;
		}

		/**
		 * @return whether this link is footpath or not
		 */
		public boolean isWalking() {
			return isWalking;
		}

		/**
		 * @return delay classification of this link
		 */
		public @Nullable DelayClassification getDelayClass() {
			return delayClass;
		}

		@Override
		public boolean equals(Object obj) {
			if (obj instanceof TPTravel && obj != null) {
				TPTravel second = (TPTravel) obj;
				return second.isWalking == isWalking && node.equals(second.node);
			}
			return false;
		}
	}

	/**
	 * Encodes delay scenario that led to the creation of a new travel link
	 * 
	 * @author Sebastian Peter
	 *
	 */
	public static class DelayClassification implements Serializable {
		private static final long serialVersionUID = 2L;

		private ArrayList<TripPattern> tripPatterns = new ArrayList<>();

		/**
		 * minimum delay in seconds
		 */
		private ArrayList<Integer> minDelays = new ArrayList<>();

		/**
		 * Adds line with given delay
		 * 
		 * @param tripPattern
		 *            line that was delayed
		 * @param minDelay
		 *            amount of delay in seconds
		 */
		public void add(TripPattern tripPattern, int minDelay) {
			this.tripPatterns.add(tripPattern);
			this.minDelays.add(minDelay);
		}

		/**
		 * @return delayed lines
		 */
		public List<TripPattern> getTripPatterns() {
			return tripPatterns;
		}

		/**
		 * @return the minimal delays in seconds for this classification to become active
		 */
		public List<Integer> getMinDelay() {
			return minDelays;
		}

		@Override
		public int hashCode() {
			return tripPatterns.hashCode();
		}

		/**
		 * Creates a timetable snapshot with all trips of all lines delayed by
		 * their respective amount
		 * 
		 * @param sd
		 *            service date when the delay should happen
		 * @return timetable snaptshot with all delays
		 */
		public TimetableSnapshot toTimeTableSnapshot(ServiceDate sd) {
			TimetableSnapshot snapshot = new TimetableSnapshot();

			Iterator<TripPattern> tripPatternIt = tripPatterns.iterator();
			Iterator<Integer> minDelayIt = minDelays.iterator();

			while (tripPatternIt.hasNext()) {
				TripPattern tp = tripPatternIt.next();
				Integer delay = minDelayIt.next();

				addPatternDelay(snapshot, tp, delay, sd);
			}

			return snapshot;
		}

		/**
		 * Helper method that adds a delay to a timetable snapshot
		 * 
		 * @param snapshot
		 *            the snapshot to add the delay to
		 * @param tripPattern
		 *            delayed line
		 * @param delaySecs
		 *            amount of delay
		 * @param serviceDate
		 *            date that the delay should apply
		 */
		private static void addPatternDelay(@Nonnull TimetableSnapshot snapshot, @Nonnull TripPattern tripPattern,
				int delaySecs, ServiceDate serviceDate) {
			Timetable timetable = tripPattern.getUpdatedTimetable(null, null);

			// add delay such that next trip is missed
			final int delay = delaySecs + 1;

			for (TripTimes tripTimes : timetable.tripTimes) {
				TripTimes cloneTt = tripTimes.clone();
				for (int i = 0; i < tripTimes.getNumStops(); i++) {
					cloneTt.updateArrivalTime(i, tripTimes.getArrivalTime(i) + delay);
					cloneTt.updateDepartureTime(i, tripTimes.getDepartureTime(i) + delay);
				}

				snapshot.update(null, tripPattern, cloneTt, serviceDate);
			}
		}

		@Override
		public String toString() {
			StringBuilder b = new StringBuilder();
			b.append('{');
			for (int i = 0; i < tripPatterns.size(); i++) {
				b.append(tripPatterns.get(i).name);
				b.append(" <");
				b.append(minDelays.get(i));
				b.append("s>");

				if (i != tripPatterns.size() - 1)
					b.append(", ");
			}
			b.append('}');
			return b.toString();
		}
	}
}
