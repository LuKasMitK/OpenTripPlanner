package org.opentripplanner.transit.raptor.rangeraptor.standard.besttimes;

import org.opentripplanner.transit.raptor.rangeraptor.internalapi.RoundProvider;
import org.opentripplanner.transit.raptor.rangeraptor.standard.internalapi.BestNumberOfTransfers;
import org.opentripplanner.transit.raptor.util.IntUtils;

/**
 * The responsibility for this class is to keep track of the best (minimun) number of transfers for
 * all stops reached.
 */
public class SimpleBestNumberOfTransfers implements BestNumberOfTransfers {

  private final int[] bestNumOfTransfers;
  private final RoundProvider roundProvider;

  public SimpleBestNumberOfTransfers(int nStops, RoundProvider roundProvider) {
    this.bestNumOfTransfers = IntUtils.intArray(nStops, unreachedMinNumberOfTransfers());
    this.roundProvider = roundProvider;
  }

  @Override
  public int calculateMinNumberOfTransfers(int stop) {
    return bestNumOfTransfers[stop];
  }

  /**
   * Call this method to notify that the given stop is reached in the current round of Raptor.
   */
  void arriveAtStop(int stop) {
    final int numOfTransfers = roundProvider.round() - 1;
    if (numOfTransfers < bestNumOfTransfers[stop]) {
      bestNumOfTransfers[stop] = numOfTransfers;
    }
  }
}
