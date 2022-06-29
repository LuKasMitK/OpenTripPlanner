package org.opentripplanner.routing.graph;

import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.TransferPattern.DelayClassification;

/**
 * Builds sets of delay classifications for transfer pattern creation
 * 
 * @author Sebastian Peter
 *
 */
public abstract class TransferPatternDelayBuilder {

	/**
	 * Computes sets of delays that can be used for TP creation
	 * 
	 * @param delays
	 *            single delays and their respective amount of delay in seconds
	 * @return list of delay classes that state delay scenarios
	 */
	public abstract List<DelayClassification> getDelays(Map<TripPattern, Integer> delays);

	/**
	 * Returns an empty set. Should be used when no dynamic patterns are to be
	 * built.
	 */
	public static class NoDelayBuilder extends TransferPatternDelayBuilder {
		@Override
		public List<DelayClassification> getDelays(Map<TripPattern, Integer> delays) {
			return Collections.emptyList();
		}
	}

	/**
	 * Builds simple set of delay classifications with a single delayed line
	 */
	public static class SimpleDelayBuilder extends TransferPatternDelayBuilder {
		@Override
		public List<DelayClassification> getDelays(Map<TripPattern, Integer> delays) {
			List<DelayClassification> delaySets = new LinkedList<>();

			for (Entry<TripPattern, Integer> entry : delays.entrySet()) {
				TripPattern tripPattern = entry.getKey();
				Integer delay = entry.getValue();

				DelayClassification dc = new DelayClassification();
				dc.add(tripPattern, delay);

				delaySets.add(dc);
			}

			return delaySets;
		}
	}

	/**
	 * Builds simple set of delay classifications with a single delayed line,
	 * but restricted to given number of classifications
	 */
	public static class RestrictedSimpleDelayBuilder extends SimpleDelayBuilder {
		private final int maxClasses;

		/**
		 * @param maxClasses
		 *            maximal number of classifications to be computed
		 */
		public RestrictedSimpleDelayBuilder(int maxClasses) {
			this.maxClasses = maxClasses;
		}

		@Override
		public List<DelayClassification> getDelays(Map<TripPattern, Integer> delays) {
			List<DelayClassification> delaySets = super.getDelays(delays);

			if (delaySets.size() > maxClasses) {
				Collections.shuffle(delaySets);
				return delaySets.subList(0, maxClasses);
			}

			return delaySets;
		}
	}

	/**
	 * Returns all possible delay classifications with combinations of delayed
	 * lines that include 1 to given number of delayed lines
	 */
	public static class PowerSetDelayBuilder extends TransferPatternDelayBuilder {

		private final int maxItems;

		/**
		 * @param maxItems
		 *            maximal number of delayed lines that a classification
		 *            should contain
		 */
		public PowerSetDelayBuilder(int maxItems) {
			this.maxItems = maxItems;
		}

		@Override
		public List<DelayClassification> getDelays(Map<TripPattern, Integer> delays) {
			Set<TripPattern> tripPattern = new HashSet<>();

			for (Entry<TripPattern, Integer> entry : delays.entrySet()) {
				tripPattern.add(entry.getKey());
			}

			// build power set between 1 and 2 items
			PowerSet<TripPattern> pSet = new PowerSet<>(tripPattern, 1, maxItems);

			List<DelayClassification> delaySets = new LinkedList<>();

			for (Iterable<TripPattern> set : pSet) {
				DelayClassification dc = new DelayClassification();
				for (TripPattern tp : set) {
					Integer delay = delays.get(tp);
					dc.add(tp, delay);
				}

				delaySets.add(dc);
			}

			return delaySets;
		}
	}
}