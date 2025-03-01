package org.opentripplanner.model.plan;

import static java.util.Locale.ROOT;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.opentripplanner.model.SystemNotice;
import org.opentripplanner.routing.core.Fare;
import org.opentripplanner.transit.raptor.api.path.PathStringBuilder;
import org.opentripplanner.util.lang.DoubleUtils;
import org.opentripplanner.util.lang.ToStringBuilder;

/**
 * An Itinerary is one complete way of getting from the start location to the end location.
 */
public class Itinerary {

  /* final primitive properties */
  private final int durationSeconds;
  private final int transitTimeSeconds;
  private final int numberOfTransfers;
  private final int waitingTimeSeconds;
  private final double nonTransitDistanceMeters;
  private final boolean walkOnly;
  private final boolean streetOnly;
  private final int nonTransitTimeSeconds;

  /* mutable primitive properties */
  private Double elevationLost = 0.0;
  private Double elevationGained = 0.0;
  private int generalizedCost = -1;
  private int waitTimeOptimizedCost = -1;
  private int transferPriorityCost = -1;
  private boolean tooSloped = false;
  private Double maxSlope = null;
  private boolean arrivedAtDestinationWithRentedVehicle = false;

  /* Sandbox experimental properties */
  private Float accessibilityScore;

  /* other properties */

  private final List<SystemNotice> systemNotices = new ArrayList<>();
  private List<Leg> legs;

  private Fare fare = new Fare();

  public Itinerary(List<Leg> legs) {
    setLegs(legs);

    // Set aggregated data
    ItinerariesCalculateLegTotals totals = new ItinerariesCalculateLegTotals(legs);
    this.durationSeconds = totals.totalDurationSeconds;
    this.numberOfTransfers = totals.transfers();
    this.transitTimeSeconds = totals.transitTimeSeconds;
    this.nonTransitTimeSeconds = totals.nonTransitTimeSeconds;
    this.nonTransitDistanceMeters = DoubleUtils.roundTo2Decimals(totals.nonTransitDistanceMeters);
    this.waitingTimeSeconds = totals.waitingTimeSeconds;
    this.walkOnly = totals.walkOnly;
    this.streetOnly = totals.streetOnly;
    this.setElevationGained(totals.totalElevationGained);
    this.setElevationLost(totals.totalElevationLost);
  }

  /**
   * Used to convert a list of itineraries to a SHORT human readable string.
   *
   * @see #toStr()
   * <p>
   * It is great for comparing lists of itineraries in a test: {@code
   * assertEquals(toStr(List.of(it1)), toStr(result))}.
   */
  public static String toStr(List<Itinerary> list) {
    return list.stream().map(Itinerary::toStr).collect(Collectors.joining(", "));
  }

  /**
   * Time that the trip departs.
   */
  public ZonedDateTime startTime() {
    return firstLeg().getStartTime();
  }

  /**
   * Time that the trip arrives.
   */
  public ZonedDateTime endTime() {
    return lastLeg().getEndTime();
  }

  /**
   * Reflects the departureDelay on the first Leg Unit: seconds.
   */
  public int departureDelay() {
    return firstLeg().getDepartureDelay();
  }

  /**
   * Reflects the arrivalDelay on the last Leg Unit: seconds.
   */
  public int arrivalDelay() {
    return lastLeg().getArrivalDelay();
  }

  /**
   * This is the amount of time used to travel. {@code waitingTime} is NOT included.
   */
  public int effectiveDurationSeconds() {
    return getTransitTimeSeconds() + getNonTransitTimeSeconds();
  }

  /**
   * Total distance in meters.
   */
  public double distanceMeters() {
    return getLegs().stream().mapToDouble(Leg::getDistanceMeters).sum();
  }

  /**
   * Return {@code true} if all legs are WALKING.
   */
  public boolean isWalkingAllTheWay() {
    return isWalkOnly();
  }

  /**
   * Return {@code true} if all legs are WALKING.
   */
  public boolean isOnStreetAllTheWay() {
    return isStreetOnly();
  }

  /** TRUE if alt least one leg is a transit leg. */
  public boolean hasTransit() {
    return getTransitTimeSeconds() > 0;
  }

  public Leg firstLeg() {
    return getLegs().get(0);
  }

  public Leg lastLeg() {
    return getLegs().get(getLegs().size() - 1);
  }

  /** Get the first transit leg if one exist */
  public Optional<Leg> firstTransitLeg() {
    return getLegs().stream().filter(Leg::isTransitLeg).findFirst();
  }

  /**
   * An itinerary can be flagged for removal with a system notice.
   * <p>
   * For example when tuning or manually testing the itinerary-filter-chain it you can enable {@link
   * org.opentripplanner.routing.api.request.ItineraryFilterParameters#debug} and instead of
   * removing itineraries from the result the itineraries will be tagged by the filters instead.
   * This enables investigating, why an expected itinerary is missing from the result set. It can be
   * also used by other filters to see the already filtered itineraries.
   */
  public void flagForDeletion(SystemNotice notice) {
    getSystemNotices().add(notice);
  }

  public boolean isFlaggedForDeletion() {
    return !getSystemNotices().isEmpty();
  }

  public Itinerary withTimeShiftToStartAt(ZonedDateTime afterTime) {
    Duration duration = Duration.between(firstLeg().getStartTime(), afterTime);
    List<Leg> timeShiftedLegs = getLegs()
      .stream()
      .map(leg -> leg.withTimeShift(duration))
      .collect(Collectors.toList());
    var newItin = new Itinerary(timeShiftedLegs);
    newItin.setGeneralizedCost(getGeneralizedCost());
    return newItin;
  }

  /** @see #equals(Object) */
  @Override
  public final int hashCode() {
    return super.hashCode();
  }

  /**
   * Return {@code true} it the other object is the same object using the {@link
   * Object#equals(Object)}. An itinerary is a temporary object and the equals method should not be
   * used for comparision of 2 instances, only to check that to objects are the same instance.
   */
  @Override
  public final boolean equals(Object o) {
    return super.equals(o);
  }

  @Override
  public String toString() {
    return ToStringBuilder
      .of(Itinerary.class)
      .addStr("from", firstLeg().getFrom().toStringShort())
      .addStr("to", lastLeg().getTo().toStringShort())
      .addTimeCal("start", firstLeg().getStartTime())
      .addTimeCal("end", lastLeg().getEndTime())
      .addNum("nTransfers", numberOfTransfers, -1)
      .addDurationSec("duration", durationSeconds)
      .addNum("generalizedCost", generalizedCost)
      .addDurationSec("nonTransitTime", nonTransitTimeSeconds)
      .addDurationSec("transitTime", transitTimeSeconds)
      .addDurationSec("waitingTime", waitingTimeSeconds)
      .addNum("nonTransitDistance", nonTransitDistanceMeters, "m")
      .addBool("tooSloped", tooSloped)
      .addNum("elevationLost", elevationLost, 0.0)
      .addNum("elevationGained", elevationGained, 0.0)
      .addCol("legs", legs)
      .addObj("fare", fare)
      .toString();
  }

  /**
   * Used to convert an itinerary to a SHORT human readable string - including just a few of the
   * most important fields. It is much shorter and easier to read then the {@link
   * Itinerary#toString()}.
   * <p>
   * It is great for comparing to itineraries in a test: {@code assertEquals(toStr(it1),
   * toStr(it2))}.
   * <p>
   * Example: {@code A ~ Walk 2m ~ B ~ BUS 55 12:04 12:14 ~ C [cost: 1066]}
   * <p>
   * Reads: Start at A, walk 2 minutes to stop B, take bus 55, board at 12:04 and alight at 12:14
   * ...
   */
  public String toStr() {
    // No translater needed, stop indexes are never passed to the builder
    PathStringBuilder buf = new PathStringBuilder(null);
    buf.stop(firstLeg().getFrom().name.toString());

    for (Leg leg : legs) {
      buf.sep();
      if (leg.isWalkingLeg()) {
        buf.walk((int) leg.getDuration());
      } else if (leg.isTransitLeg()) {
        buf.transit(
          leg.getMode().name(),
          leg.getTrip().logName(),
          leg.getStartTime(),
          leg.getEndTime()
        );
      } else {
        buf.other(leg.getMode().name(), leg.getStartTime(), leg.getEndTime());
      }

      buf.sep();
      buf.stop(leg.getTo().name.toString());
    }

    buf.space().append(String.format(ROOT, "[ $%d ]", generalizedCost));

    return buf.toString();
  }

  /** Total duration of the itinerary in seconds */
  public int getDurationSeconds() {
    return durationSeconds;
  }

  /**
   * How much time is spent on transit, in seconds.
   */
  public int getTransitTimeSeconds() {
    return transitTimeSeconds;
  }

  /**
   * The number of transfers this trip has.
   */
  public int getNumberOfTransfers() {
    return numberOfTransfers;
  }

  /**
   * How much time is spent waiting for transit to arrive, in seconds.
   */
  public int getWaitingTimeSeconds() {
    return waitingTimeSeconds;
  }

  /**
   * How far the user has to walk, bike and/or drive, in meters.
   */
  public double getNonTransitDistanceMeters() {
    return nonTransitDistanceMeters;
  }

  /** TRUE if mode is WALK from start ot end (all legs are walking). */
  public boolean isWalkOnly() {
    return walkOnly;
  }

  /** TRUE if mode is a non transit move from start ot end (all legs are non-transit). */
  public boolean isStreetOnly() {
    return streetOnly;
  }

  /**
   * System notices is used to tag itineraries with system information. For example if you run the
   * itinerary-filter in debug mode, the filters would tag itineraries instead of deleting them from
   * the result. More than one filter might apply, so there can be more than one notice for an
   * itinerary. This is very handy, when tuning the system or debugging - looking for missing
   * expected trips.
   */
  public List<SystemNotice> getSystemNotices() {
    return systemNotices;
  }

  /**
   * A list of Legs. Each Leg is either a walking (cycling, car) portion of the trip, or a transit
   * trip on a particular vehicle. So a trip where the use walks to the Q train, transfers to the 6,
   * then walks to their destination, has four legs.
   */
  public List<Leg> getLegs() {
    return legs;
  }

  public void setLegs(List<Leg> legs) {
    this.legs = List.copyOf(legs);
  }

  /**
   * A sandbox feature for calculating a numeric score between 0 and 1 which indicates how
   * accessible the itinerary is as a whole. This is not a very scientific method but just a rough
   * guidance that expresses certainty or uncertainty about the accessibility.
   * <p>
   * An alternative to this is to use the `generalized-cost` and use that to indicate witch itineraries is the
   * best/most friendly with respect to making the journey in a wheelchair. The `generalized-cost` include, not
   * only a penalty for unknown and inaccessible boardings, but also a penalty for undesired uphill and downhill
   * street traversal.
   * <p>
   * The intended audience for this score are frontend developers wanting to show a simple UI rather
   * than having to iterate over all the stops and trips.
   * <p>
   * Note: the information to calculate this score are all available to the frontend, however
   * calculating them on the backend makes life a little easier and changes are automatically
   * applied to all frontends.
   */
  public Float getAccessibilityScore() {
    return accessibilityScore;
  }

  public void setAccessibilityScore(Float accessibilityScore) {
    this.accessibilityScore = accessibilityScore;
  }

  /**
   * How much time is spent walking/biking/driving, in seconds.
   */
  public int getNonTransitTimeSeconds() {
    return nonTransitTimeSeconds;
  }

  /**
   * How much elevation is lost, in total, over the course of the trip, in meters. As an example, a
   * trip that went from the top of Mount Everest straight down to sea level, then back up K2, then
   * back down again would have an elevationLost of Everest + K2.
   */
  public Double getElevationLost() {
    return elevationLost;
  }

  public void setElevationLost(Double elevationLost) {
    this.elevationLost = DoubleUtils.roundTo2Decimals(elevationLost);
  }

  /**
   * How much elevation is gained, in total, over the course of the trip, in meters. See
   * elevationLost.
   */
  public Double getElevationGained() {
    return elevationGained;
  }

  public void setElevationGained(Double elevationGained) {
    this.elevationGained = DoubleUtils.roundTo2Decimals(elevationGained);
  }

  /**
   * If a generalized cost is used in the routing algorithm, this should be the total cost computed
   * by the algorithm. This is relevant for anyone who want to debug an search and tuning the
   * system. The unit should be equivalent to the cost of "one second of transit".
   * <p>
   * -1 indicate that the cost is not set/computed.
   */
  public int getGeneralizedCost() {
    return generalizedCost;
  }

  public void setGeneralizedCost(int generalizedCost) {
    this.generalizedCost = generalizedCost;
  }

  /**
   * This is the transfer-wait-time-cost. The aim is to distribute wait-time and adding a high
   * penalty on short transfers. Do not use this to compare or filter itineraries. The filtering on
   * this parameter is done on paths, before mapping to itineraries and is provided here as
   * reference information.
   * <p>
   * -1 indicate that the cost is not set/computed.
   */
  public int getWaitTimeOptimizedCost() {
    return waitTimeOptimizedCost;
  }

  public void setWaitTimeOptimizedCost(int waitTimeOptimizedCost) {
    this.waitTimeOptimizedCost = waitTimeOptimizedCost;
  }

  /**
   * This is the transfer-priority-cost. If two paths ride the same trips with different transfers,
   * this cost is used to pick the one with the best transfer constraints (guaranteed, stay-seated,
   * not-allowed ...). Do not use this to compare or filter itineraries. The filtering on this
   * parameter is done on paths, before mapping to itineraries and is provided here as reference
   * information.
   * <p>
   * -1 indicate that the cost is not set/computed.
   */
  public int getTransferPriorityCost() {
    return transferPriorityCost;
  }

  public void setTransferPriorityCost(int transferPriorityCost) {
    this.transferPriorityCost = transferPriorityCost;
  }

  /**
   * This itinerary has a greater slope than the user requested.
   */
  public boolean isTooSloped() {
    return tooSloped;
  }

  public void setTooSloped(boolean tooSloped) {
    this.tooSloped = tooSloped;
  }

  /**
   * The maximum slope for any part of the itinerary.
   */
  public Double getMaxSlope() {
    return maxSlope;
  }

  public void setMaxSlope(Double maxSlope) {
    this.maxSlope = DoubleUtils.roundTo2Decimals(maxSlope);
  }

  /**
   * If {@link org.opentripplanner.routing.api.request.RoutingRequest#allowKeepingRentedVehicleAtDestination}
   * is set than it is possible to end a trip without dropping off the rented bicycle.
   */
  public boolean isArrivedAtDestinationWithRentedVehicle() {
    return arrivedAtDestinationWithRentedVehicle;
  }

  public void setArrivedAtDestinationWithRentedVehicle(
    boolean arrivedAtDestinationWithRentedVehicle
  ) {
    this.arrivedAtDestinationWithRentedVehicle = arrivedAtDestinationWithRentedVehicle;
  }

  /**
   * The cost of this trip
   */
  public Fare getFare() {
    return fare;
  }

  public void setFare(Fare fare) {
    this.fare = fare;
  }
}
