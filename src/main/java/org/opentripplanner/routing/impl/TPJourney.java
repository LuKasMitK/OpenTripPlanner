package org.opentripplanner.routing.impl;

import java.util.Iterator;
import java.util.LinkedList;

/**
 * holds information about one journey, consisting of one or multiple
 * {@link TPLeg}
 * 
 * @author Sebastian Peter
 *
 */
public class TPJourney {
	public LinkedList<TPLeg> legs = new LinkedList<>();

	public TPJourney() {}

	public TPJourney(TPJourney clone) {
		for (TPLeg leg : clone.legs) {
			legs.add(new TPLeg(leg));
		}
	}

	/**
	 * @return departure time in seconds since epoch
	 */
	public long getDeparture() {
		if (hasLegs())
			return legs.getFirst().getDeparture();
		return -1;
	}

	/**
	 * @return arrival time in seconds since epoch
	 * 
	 */
	public long getArrival() {
		if (hasLegs())
			return legs.getLast().getArrival();
		return -1;
	}

	/**
	 * @return duration of journey in seconds
	 */
	public int getDuration() {
		if (hasLegs())
			return (int) (getArrival() - getDeparture());

		return -1;
	}

	/**
	 * @return whether journey has legs or not
	 */
	public boolean hasLegs() {
		return !legs.isEmpty();
	}

	/**
	 * @return number of legs of this journey
	 */
	public int legsCount() {
		return legs.size();
	}

	@Override
	public String toString() {
		StringBuilder b = new StringBuilder();

		Iterator<TPLeg> itLegs = legs.iterator();

		boolean first = true;

		while (itLegs.hasNext()) {
			TPLeg currentLeg = itLegs.next();

			if (first) {
				b.append(currentLeg.getFromVertex().getName());
				first = false;
			}

			if (currentLeg.hasConcreteConnection()) {
				// departure time in minutes
				int depTime = currentLeg.getDepartureSinceMidnight();
				b.append(' ');
				appendTime(b, depTime);
			}

			if (currentLeg.isWalking())
				b.append(" --(walking)--> ");
			else
				b.append(" --> ");

			if (currentLeg.hasConcreteConnection()) {
				// departure time in minutes
				int arrTime = currentLeg.getArrivalSinceMidnight();
				appendTime(b, arrTime);
				b.append(' ');
			}

			b.append(currentLeg.getToVertex().getName());
		}

		return b.toString();
	}

	/**
	 * Computes equality only considering stops and walking
	 */
	@Override
	public boolean equals(Object other) {
		if (other == null || !(other instanceof TPJourney))
			return false;

		TPJourney otherJourney = (TPJourney) other;

		if (legs.size() != otherJourney.legs.size())
			return false;

		Iterator<TPLeg> legs1 = legs.iterator();
		Iterator<TPLeg> legs2 = otherJourney.legs.iterator();

		while (legs1.hasNext()) {
			TPLeg leg1 = legs1.next();
			TPLeg leg2 = legs2.next();

			if (leg1.getFromVertex() != leg2.getFromVertex() || leg1.getToVertex() != leg2.getToVertex()
					|| leg1.isWalking() != leg2.isWalking())
				return false;
		}

		return true;
	}

	private static void appendTime(StringBuilder b, int depTime) {
		// convert to minutes
		depTime /= 60;
		b.append('(');
		b.append(depTime / 60);
		b.append(':');
		int min = depTime % 60;
		if (min < 10)
			b.append('0');
		b.append(min);
		b.append(')');
	}
}