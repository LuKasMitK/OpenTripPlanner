package org.opentripplanner.updater.stoptime;

import com.google.transit.realtime.GtfsRealtime.TripUpdate;
import java.util.List;
import org.opentripplanner.routing.RoutingService;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.transit.service.DefaultTransitService;
import org.opentripplanner.updater.GtfsRealtimeFuzzyTripMatcher;
import org.opentripplanner.updater.PollingGraphUpdater;
import org.opentripplanner.updater.WriteToGraphCallback;
import org.opentripplanner.util.lang.ToStringBuilder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Update OTP stop time tables from some (realtime) source
 * <p>
 * Usage example:
 *
 * <pre>
 * rt.type = stop-time-updater
 * rt.frequencySec = 60
 * rt.sourceType = gtfs-http
 * rt.url = http://host.tld/path
 * rt.feedId = TA
 * </pre>
 */
public class PollingStoptimeUpdater extends PollingGraphUpdater {

  private static final Logger LOG = LoggerFactory.getLogger(PollingStoptimeUpdater.class);
  /**
   * Update streamer
   */
  private final TripUpdateSource updateSource;
  /**
   * Property to set on the RealtimeDataSnapshotSource
   */
  private final Boolean purgeExpiredData;
  /**
   * Feed id that is used for the trip ids in the TripUpdates
   */
  private final String feedId;
  private final boolean fuzzyTripMatching;
  /**
   * Defines when delays are propagated to previous stops and if these stops are given
   * the NO_DATA flag.
   */
  private final BackwardsDelayPropagationType backwardsDelayPropagationType;
  /**
   * Parent update manager. Is used to execute graph writer runnables.
   */
  private WriteToGraphCallback saveResultOnGraph;
  /**
   * Property to set on the RealtimeDataSnapshotSource
   */
  private Integer logFrequency;
  /**
   * Property to set on the RealtimeDataSnapshotSource
   */
  private Integer maxSnapshotFrequency;
  /**
   * Set only if we should attempt to match the trip_id from other data in TripDescriptor
   */
  private GtfsRealtimeFuzzyTripMatcher fuzzyTripMatcher;

  public PollingStoptimeUpdater(PollingStoptimeUpdaterParameters parameters) {
    super(parameters);
    // Create update streamer from preferences
    this.feedId = parameters.getFeedId();
    this.updateSource = createSource(parameters);

    // Configure updater FIXME why are the fields objects instead of primitives? this allows null values...
    int logFrequency = parameters.getLogFrequency();
    if (logFrequency >= 0) {
      this.logFrequency = logFrequency;
    }
    int maxSnapshotFrequency = parameters.getMaxSnapshotFrequencyMs();
    if (maxSnapshotFrequency >= 0) {
      this.maxSnapshotFrequency = maxSnapshotFrequency;
    }
    this.purgeExpiredData = parameters.purgeExpiredData();
    this.fuzzyTripMatching = parameters.fuzzyTripMatching();
    this.backwardsDelayPropagationType = parameters.getBackwardsDelayPropagationType();

    LOG.info(
      "Creating stop time updater running every {} seconds : {}",
      pollingPeriodSeconds,
      updateSource
    );
  }

  @Override
  public void setGraphUpdaterManager(WriteToGraphCallback saveResultOnGraph) {
    this.saveResultOnGraph = saveResultOnGraph;
  }

  @Override
  public void setup(Graph graph) {
    if (fuzzyTripMatching) {
      this.fuzzyTripMatcher =
        new GtfsRealtimeFuzzyTripMatcher(
          new RoutingService(graph),
          new DefaultTransitService(graph)
        );
    }

    // Only create a realtime data snapshot source if none exists already
    TimetableSnapshotSource snapshotSource = graph.getOrSetupTimetableSnapshotProvider(
      TimetableSnapshotSource::ofGraph
    );

    // Set properties of realtime data snapshot source
    if (logFrequency != null) {
      snapshotSource.logFrequency = logFrequency;
    }
    if (maxSnapshotFrequency != null) {
      snapshotSource.maxSnapshotFrequency = maxSnapshotFrequency;
    }
    if (purgeExpiredData != null) {
      snapshotSource.purgeExpiredData = purgeExpiredData;
    }
    if (fuzzyTripMatcher != null) {
      snapshotSource.fuzzyTripMatcher = fuzzyTripMatcher;
    }
    if (backwardsDelayPropagationType != null) {
      snapshotSource.backwardsDelayPropagationType = backwardsDelayPropagationType;
    }
  }

  @Override
  public void teardown() {}

  /**
   * Repeatedly makes blocking calls to an UpdateStreamer to retrieve new stop time updates, and
   * applies those updates to the graph.
   */
  @Override
  public void runPolling() {
    // Get update lists from update source
    List<TripUpdate> updates = updateSource.getUpdates();
    boolean fullDataset = updateSource.getFullDatasetValueOfLastUpdates();

    if (updates != null) {
      // Handle trip updates via graph writer runnable
      TripUpdateGraphWriterRunnable runnable = new TripUpdateGraphWriterRunnable(
        fullDataset,
        updates,
        feedId
      );
      saveResultOnGraph.execute(runnable);
    }
  }

  @Override
  public String toString() {
    return ToStringBuilder
      .of(this.getClass())
      .addObj("updateSource", updateSource)
      .addStr("feedId", feedId)
      .addBoolIfTrue("fuzzyTripMatching", fuzzyTripMatching)
      .toString();
  }

  private static TripUpdateSource createSource(PollingStoptimeUpdaterParameters parameters) {
    switch (parameters.getSourceType()) {
      case GTFS_RT_HTTP:
        return new GtfsRealtimeHttpTripUpdateSource(parameters.httpSourceParameters());
      case GTFS_RT_FILE:
        return new GtfsRealtimeFileTripUpdateSource(parameters.fileSourceParameters());
    }
    throw new IllegalArgumentException(
      "Unknown update streamer source type: " + parameters.getSourceType()
    );
  }
}
