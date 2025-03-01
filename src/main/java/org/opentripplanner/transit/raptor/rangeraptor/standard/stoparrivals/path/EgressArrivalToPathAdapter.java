package org.opentripplanner.transit.raptor.rangeraptor.standard.stoparrivals.path;

import java.util.ArrayList;
import java.util.List;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransfer;
import org.opentripplanner.transit.raptor.api.transit.RaptorTripSchedule;
import org.opentripplanner.transit.raptor.api.view.ArrivalView;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.WorkerLifeCycle;
import org.opentripplanner.transit.raptor.rangeraptor.path.DestinationArrivalPaths;
import org.opentripplanner.transit.raptor.rangeraptor.standard.internalapi.ArrivedAtDestinationCheck;
import org.opentripplanner.transit.raptor.rangeraptor.standard.internalapi.DestinationArrivalListener;
import org.opentripplanner.transit.raptor.rangeraptor.standard.stoparrivals.view.StopsCursor;
import org.opentripplanner.transit.raptor.rangeraptor.transit.TransitCalculator;
import org.opentripplanner.util.lang.ToStringBuilder;
import org.opentripplanner.util.time.TimeUtils;

/**
 * The responsibility of this class is to listen for egress stop arrivals and forward these as
 * Destination arrivals to the {@link DestinationArrivalPaths}.
 * <p/>
 * Range Raptor requires paths to be collected at the end of each iteration. Following iterations
 * may overwrite the existing state; Hence invalidate trips explored in previous iterations. Because
 * adding new destination arrivals to the set of paths is expensive, this class optimize this by
 * only adding new destination arrivals at the end of each round.
 * <p/>
 *
 * @param <T> The TripSchedule type defined by the user of the raptor API.
 */
public class EgressArrivalToPathAdapter<T extends RaptorTripSchedule>
  implements ArrivedAtDestinationCheck, DestinationArrivalListener {

  private final DestinationArrivalPaths<T> paths;
  private final TransitCalculator<T> calculator;
  private final StopsCursor<T> cursor;
  private final List<DestinationArrivalEvent> rejectedArrivals;

  private int bestDestinationTime = -1;
  private DestinationArrivalEvent bestArrival = null;

  public EgressArrivalToPathAdapter(
    DestinationArrivalPaths<T> paths,
    TransitCalculator<T> calculator,
    StopsCursor<T> cursor,
    WorkerLifeCycle lifeCycle
  ) {
    this.paths = paths;
    this.calculator = calculator;
    this.cursor = cursor;
    this.rejectedArrivals = paths.isDebugOn() ? new ArrayList<>() : null;
    lifeCycle.onSetupIteration(ignore -> setupIteration());
    lifeCycle.onRoundComplete(ignore -> roundComplete());
  }

  @Override
  public void newDestinationArrival(
    int round,
    int fromStopArrivalTime,
    boolean stopReachedOnBoard,
    RaptorTransfer egressPath
  ) {
    int arrivalTime = calculator.plusDuration(fromStopArrivalTime, egressPath.durationInSeconds());

    if (calculator.isBefore(arrivalTime, bestDestinationTime)) {
      debugRejectCurrentBestArrival();
      bestDestinationTime = arrivalTime;
      bestArrival = new DestinationArrivalEvent(round, stopReachedOnBoard, egressPath);
    } else {
      debugRejectNew(round, stopReachedOnBoard, egressPath);
    }
  }

  @Override
  public boolean arrivedAtDestinationCurrentRound() {
    return newElementSet();
  }

  private boolean newElementSet() {
    return bestArrival != null;
  }

  private void setupIteration() {
    bestArrival = null;
    bestDestinationTime = calculator.unreachedTime();
  }

  private void roundComplete() {
    if (newElementSet()) {
      addNewElementToPath();
      logDebugRejectEvents();
      bestArrival = null;
    }
  }

  private void addNewElementToPath() {
    paths.add(bestArrival.toArrivalState(cursor), bestArrival.egressPath);
  }

  private void debugRejectNew(int round, boolean stopReachedOnBoard, RaptorTransfer egressPath) {
    if (paths.isDebugOn()) {
      rejectedArrivals.add(new DestinationArrivalEvent(round, stopReachedOnBoard, egressPath));
    }
  }

  private void debugRejectCurrentBestArrival() {
    if (paths.isDebugOn() && newElementSet()) {
      rejectedArrivals.add(bestArrival);
    }
  }

  private void logDebugRejectEvents() {
    if (paths.isDebugOn()) {
      String reason = "Arrival time > " + TimeUtils.timeToStrCompact(bestDestinationTime);

      for (DestinationArrivalEvent it : rejectedArrivals) {
        paths.debugReject(it.toArrivalState(cursor), it.egressPath, reason);
      }
    }
  }

  /** Used internally in this class to cache a destination arrival */
  private static class DestinationArrivalEvent {

    final int round;
    final boolean stopReachedOnBoard;
    final RaptorTransfer egressPath;

    private DestinationArrivalEvent(
      int round,
      boolean stopReachedOnBoard,
      RaptorTransfer egressPath
    ) {
      this.round = round;
      this.stopReachedOnBoard = stopReachedOnBoard;
      this.egressPath = egressPath;
    }

    @Override
    public String toString() {
      return ToStringBuilder
        .of(DestinationArrivalEvent.class)
        .addNum("round", round)
        .addBool("stopReachedOnBoard", stopReachedOnBoard)
        .addObj("egressPath", egressPath)
        .toString();
    }

    <T extends RaptorTripSchedule> ArrivalView<T> toArrivalState(StopsCursor<T> cursor) {
      return cursor.stop(round, egressPath.stop(), stopReachedOnBoard);
    }
  }
}
