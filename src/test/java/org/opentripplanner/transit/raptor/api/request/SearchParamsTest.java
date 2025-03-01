package org.opentripplanner.transit.raptor.api.request;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.opentripplanner.transit.raptor._data.transit.TestTransfer.walk;

import org.junit.jupiter.api.Test;
import org.opentripplanner.transit.raptor._data.transit.TestTripSchedule;

public class SearchParamsTest {

  @Test
  public void earliestDepartureTimeOrLatestArrivalTimeIsRequired() {
    var p = new RaptorRequestBuilder<TestTripSchedule>().searchParams();
    p.addAccessPaths(walk(1, 30));
    p.addEgressPaths(walk(2, 20));

    assertParamNotValid(p, "'earliestDepartureTime' or 'latestArrivalTime' is required.");
  }

  @Test
  public void accessPathIsRequired() {
    var p = new RaptorRequestBuilder<TestTripSchedule>().searchParams();
    p.earliestDepartureTime(200);
    p.latestArrivalTime(600);
    p.addEgressPaths(walk(2, 20));

    assertParamNotValid(p, "At least one 'accessPath' is required.");
  }

  @Test
  public void egressPathIsRequired() {
    var p = new RaptorRequestBuilder<TestTripSchedule>().searchParams();
    p.earliestDepartureTime(200);
    p.latestArrivalTime(600);
    p.addAccessPaths(walk(1, 30));

    assertParamNotValid(p, "At least one 'egressPath' is required.");
  }

  @Test
  public void latestArrivalTimeRequiredWhenDepartAsLateAsPossibleEnabled() {
    var p = new RaptorRequestBuilder<TestTripSchedule>().searchParams();
    p.earliestDepartureTime(200);
    p.addAccessPaths(walk(1, 30));
    p.addEgressPaths(walk(2, 20));

    p.preferLateArrival(true);

    assertParamNotValid(
      p,
      "The 'latestArrivalTime' is required when 'departAsLateAsPossible' is set."
    );
  }

  @Test
  public void departAsLateAsPossibleAndTimetableEnabled() {
    var p = new RaptorRequestBuilder<TestTripSchedule>().searchParams();
    p.latestArrivalTime(200);
    p.addAccessPaths(walk(1, 30));
    p.addEgressPaths(walk(2, 20));

    p.timetableEnabled(true);
    p.preferLateArrival(true);

    assertParamNotValid(
      p,
      "The 'departAsLateAsPossible' is not allowed together with 'timetableEnabled'."
    );
  }

  public void assertParamNotValid(SearchParamsBuilder<TestTripSchedule> p, String msg) {
    assertThrows(IllegalArgumentException.class, p::build, msg);
  }
}
