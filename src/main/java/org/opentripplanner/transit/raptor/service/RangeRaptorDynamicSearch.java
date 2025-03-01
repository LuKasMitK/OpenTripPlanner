package org.opentripplanner.transit.raptor.service;

import static org.opentripplanner.transit.raptor.api.request.RaptorProfile.MULTI_CRITERIA;
import static org.opentripplanner.transit.raptor.api.transit.SearchDirection.FORWARD;
import static org.opentripplanner.transit.raptor.api.transit.SearchDirection.REVERSE;
import static org.opentripplanner.transit.raptor.service.HeuristicToRunResolver.resolveHeuristicToRunBasedOnOptimizationsAndSearchParameters;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.stream.Collectors;
import javax.annotation.Nullable;
import org.opentripplanner.transit.raptor.api.request.RaptorRequest;
import org.opentripplanner.transit.raptor.api.request.SearchParams;
import org.opentripplanner.transit.raptor.api.request.SearchParamsBuilder;
import org.opentripplanner.transit.raptor.api.response.RaptorResponse;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransitDataProvider;
import org.opentripplanner.transit.raptor.api.transit.RaptorTripSchedule;
import org.opentripplanner.transit.raptor.configure.RaptorConfig;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.Heuristics;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.Worker;
import org.opentripplanner.transit.raptor.rangeraptor.transit.RaptorSearchWindowCalculator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This search helps the {@link org.opentripplanner.transit.raptor.RaptorService} to configure
 * heuristics and set dynamic search parameters like EDT, LAT and raptor-search-window.
 * <p>
 * If possible the forward and reverse heuristics will be run in parallel.
 * <p>
 * Depending on which optimization is enabled and which search parameters are set a forward and/or a
 * reverse "single-iteration" raptor search is performed and heuristics are collected. This is used
 * to configure the "main" multi-iteration RangeRaptor search.
 */
public class RangeRaptorDynamicSearch<T extends RaptorTripSchedule> {

  private static final Logger LOG = LoggerFactory.getLogger(RangeRaptorDynamicSearch.class);

  private final RaptorConfig<T> config;
  private final RaptorTransitDataProvider<T> transitData;
  private final RaptorRequest<T> originalRequest;
  private final RaptorSearchWindowCalculator dynamicSearchParamsCalculator;

  private final HeuristicSearchTask<T> fwdHeuristics;
  private final HeuristicSearchTask<T> revHeuristics;

  public RangeRaptorDynamicSearch(
    RaptorConfig<T> config,
    RaptorTransitDataProvider<T> transitData,
    RaptorRequest<T> originalRequest
  ) {
    this.config = config;
    this.transitData = transitData;
    this.originalRequest = originalRequest;
    this.dynamicSearchParamsCalculator =
      config.searchWindowCalculator().withSearchParams(originalRequest.searchParams());

    this.fwdHeuristics = new HeuristicSearchTask<>(FORWARD, "Forward", config, transitData);
    this.revHeuristics = new HeuristicSearchTask<>(REVERSE, "Reverse", config, transitData);
  }

  public RaptorResponse<T> route() {
    try {
      enableHeuristicSearchBasedOnOptimizationsAndSearchParameters();

      // Run heuristics, if no destination is reached
      runHeuristics();

      // Set search-window and other dynamic calculated parameters
      RaptorRequest<T> dynamicRequest = originalRequest;
      dynamicRequest = requestWithDynamicSearchParams(dynamicRequest);

      return createAndRunDynamicRRWorker(dynamicRequest);
    } catch (DestinationNotReachedException e) {
      return new RaptorResponse<>(
        Collections.emptyList(),
        null,
        originalRequest,
        // If a trip exist(forward heuristics succeed), but is outside the calculated
        // search-window, then set the search-window params as if the request was
        // performed. This enable the client to page to the next window
        requestWithDynamicSearchParams(originalRequest)
      );
    }
  }

  /**
   * Only exposed for testing purposes
   */
  @Nullable
  public Heuristics getDestinationHeuristics() {
    if (!originalRequest.useDestinationPruning()) {
      return null;
    }
    LOG.debug("RangeRaptor - Destination pruning enabled.");
    return revHeuristics.result();
  }

  /**
   * Create and prepare heuristic search (both FORWARD and REVERSE) based on optimizations and input
   * search parameters. This is done for Standard and Multi-criteria profiles only.
   */
  private void enableHeuristicSearchBasedOnOptimizationsAndSearchParameters() {
    // We delegate this to a static method to be able to write unit test on this logic
    resolveHeuristicToRunBasedOnOptimizationsAndSearchParameters(
      originalRequest,
      fwdHeuristics::enable,
      revHeuristics::enable
    );
  }

  /**
   * Run standard "singe-iteration" raptor search to calculate heuristics - this should be really
   * fast to run compared with a (multi-criteria) range-raptor search.
   *
   * @throws DestinationNotReachedException if destination is not reached.
   */
  private void runHeuristics() {
    if (isItPossibleToRunHeuristicsInParallel()) {
      runHeuristicsInParallel();
    } else {
      runHeuristicsSequentially();
    }
    fwdHeuristics.debugCompareResult(revHeuristics);
  }

  private RaptorResponse<T> createAndRunDynamicRRWorker(RaptorRequest<T> request) {
    LOG.debug("Main request: " + request.toString());
    Worker<T> worker;

    // Create worker
    if (request.profile().is(MULTI_CRITERIA)) {
      worker = config.createMcWorker(transitData, request, getDestinationHeuristics());
    } else {
      worker = config.createStdWorker(transitData, request);
    }

    // Route
    worker.route();

    // create and return response
    return new RaptorResponse<>(worker.paths(), worker.stopArrivals(), originalRequest, request);
  }

  private boolean isItPossibleToRunHeuristicsInParallel() {
    SearchParams s = originalRequest.searchParams();
    return (
      config.isMultiThreaded() &&
      originalRequest.runInParallel() &&
      s.isEarliestDepartureTimeSet() &&
      s.isLatestArrivalTimeSet() &&
      fwdHeuristics.isEnabled() &&
      revHeuristics.isEnabled()
    );
  }

  /**
   * @throws DestinationNotReachedException if destination is not reached
   */
  private void runHeuristicsInParallel() {
    try {
      fwdHeuristics.withRequest(originalRequest);
      revHeuristics.withRequest(originalRequest);

      Future<?> f = config.threadPool().submit(fwdHeuristics::run);
      revHeuristics.run();
      f.get();
      LOG.debug(
        "Route using RangeRaptor - " + "REVERSE and FORWARD heuristic search performed in parallel."
      );
    } catch (ExecutionException | InterruptedException e) {
      if (e.getCause() instanceof DestinationNotReachedException) {
        throw new DestinationNotReachedException();
      }
      LOG.error(e.getMessage() + ". Request: " + originalRequest, e);
      throw new IllegalStateException(
        "Failed to run FORWARD/REVERSE heuristic search in parallel. Details: " + e.getMessage()
      );
    }
  }

  /**
   * @throws DestinationNotReachedException if destination is not reached
   */
  private void runHeuristicsSequentially() {
    List<HeuristicSearchTask<T>> tasks = listTasksInOrder();

    if (tasks.isEmpty()) {
      return;
    }

    // Run the first heuristic search
    HeuristicSearchTask<T> task;
    task = tasks.get(0);
    task.withRequest(originalRequest).run();
    calculateDynamicSearchParametersFromHeuristics(task.result());

    if (tasks.size() == 1) {
      return;
    }

    // Run the second heuristic search
    task = tasks.get(1);
    RaptorRequest<T> request = task.getDirection().isForward()
      ? requestForForwardHeurSearchWithDynamicSearchParams()
      : requestForReverseHeurSearchWithDynamicSearchParams();

    task.withRequest(request).run();
  }

  /**
   * If the earliest-departure-time(EDT) is set, the task order should be:
   * <ol>
   *     <li>{@code FORWARD}</li>
   *     <li>{@code REVERSE}</li>
   * </ol>
   * If not EDT is set, the latest-arrival-time is set, and the order should be the opposite,
   * with {@code REVERSE} first
   */
  private List<HeuristicSearchTask<T>> listTasksInOrder() {
    boolean performForwardFirst = originalRequest.searchParams().isEarliestDepartureTimeSet();

    List<HeuristicSearchTask<T>> list = performForwardFirst
      ? List.of(fwdHeuristics, revHeuristics)
      : List.of(revHeuristics, fwdHeuristics);

    return list.stream().filter(HeuristicSearchTask::isEnabled).collect(Collectors.toList());
  }

  private RaptorRequest<T> requestForForwardHeurSearchWithDynamicSearchParams() {
    if (originalRequest.searchParams().isEarliestDepartureTimeSet()) {
      return originalRequest;
    }
    return originalRequest
      .mutate()
      .searchParams()
      .earliestDepartureTime(transitData.getValidTransitDataStartTime())
      .build();
  }

  private RaptorRequest<T> requestForReverseHeurSearchWithDynamicSearchParams() {
    if (originalRequest.searchParams().isLatestArrivalTimeSet()) {
      return originalRequest;
    }
    return originalRequest
      .mutate()
      .searchParams()
      .latestArrivalTime(
        transitData.getValidTransitDataEndTime() +
        originalRequest.searchParams().getAccessEgressMaxDurationSeconds()
      )
      .build();
  }

  private RaptorRequest<T> requestWithDynamicSearchParams(RaptorRequest<T> request) {
    SearchParamsBuilder<T> builder = request.mutate().searchParams();

    if (!request.searchParams().isEarliestDepartureTimeSet()) {
      builder.earliestDepartureTime(dynamicSearchParamsCalculator.getEarliestDepartureTime());
    }
    if (!request.searchParams().isSearchWindowSet()) {
      builder.searchWindowInSeconds(dynamicSearchParamsCalculator.getSearchWindowSeconds());
    }
    // We do not set the latest-arrival-time, because we do not want to limit the forward
    // multi-criteria search, it does not have much effect on the performance - we only risk
    // loosing optimal results.
    return builder.build();
  }

  private void calculateDynamicSearchParametersFromHeuristics(@Nullable Heuristics heuristics) {
    if (heuristics != null) {
      dynamicSearchParamsCalculator
        .withHeuristics(
          heuristics.bestOverallJourneyTravelDuration(),
          heuristics.minWaitTimeForJourneysReachingDestination()
        )
        .calculate();
    }
  }
}
