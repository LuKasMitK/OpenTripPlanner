package org.opentripplanner.model.impl;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentripplanner.graph_builder.DataImportIssueStore;
import org.opentripplanner.model.Direction;
import org.opentripplanner.model.PickDrop;
import org.opentripplanner.model.StopPattern;
import org.opentripplanner.model.StopTime;
import org.opentripplanner.model.TripPattern;
import org.opentripplanner.model.calendar.ServiceCalendar;
import org.opentripplanner.model.calendar.ServiceCalendarDate;
import org.opentripplanner.model.calendar.ServiceDate;
import org.opentripplanner.model.calendar.ServiceDateInterval;
import org.opentripplanner.routing.trippattern.Deduplicator;
import org.opentripplanner.routing.trippattern.TripTimes;
import org.opentripplanner.transit.model._data.TransitModelForTest;
import org.opentripplanner.transit.model.framework.FeedScopedId;
import org.opentripplanner.transit.model.network.Route;
import org.opentripplanner.transit.model.site.Stop;
import org.opentripplanner.transit.model.timetable.Trip;

/**
 * This test will create a Transit service builder and then limit the service period. The services
 * defined is in period [D0, D3] with D1 and D2 inside that period. Then the service pariod is
 * limited to [D0, D1] excluding services on D2 and D3.
 * <p>
 * All data related in the last part of the service should be removed after D1 until D3.
 *
 * @author Thomas Gran (Capra) - tgr@capraconsulting.no (30.10.2017)
 */
public class OtpTransitServiceBuilderLimitPeriodTest {

  private static final ServiceDate D0 = new ServiceDate(2020, 1, 1);
  private static final ServiceDate D1 = new ServiceDate(2020, 1, 8);
  private static final ServiceDate D2 = new ServiceDate(2020, 1, 15);
  private static final ServiceDate D3 = new ServiceDate(2020, 1, 31);
  private static final FeedScopedId SERVICE_C_IN = TransitModelForTest.id("CalSrvIn");
  private static final FeedScopedId SERVICE_D_IN = TransitModelForTest.id("CalSrvDIn");
  private static final FeedScopedId SERVICE_C_OUT = TransitModelForTest.id("CalSrvOut");
  private static final FeedScopedId SERVICE_D_OUT = TransitModelForTest.id("CalSrvDOut");
  private static final Stop STOP_1 = TransitModelForTest.stop("Stop-1").build();
  private static final Stop STOP_2 = TransitModelForTest.stop("Stop-2").build();
  private static final Deduplicator DEDUPLICATOR = new Deduplicator();
  private static final List<StopTime> STOP_TIMES = List.of(
    createStopTime(STOP_1, 0),
    createStopTime(STOP_2, 300)
  );
  private static final StopPattern STOP_PATTERN = new StopPattern(STOP_TIMES);
  private static int SEQ_NR = 0;
  private final Route route = TransitModelForTest.route(newId().getId()).build();
  private final Trip tripCSIn = createTrip("TCalIn", SERVICE_C_IN);
  private final Trip tripCSOut = createTrip("TCalOut", SERVICE_C_OUT);
  private final Trip tripCSDIn = createTrip("TDateIn", SERVICE_D_IN);
  private final Trip tripCSDOut = createTrip("TDateOut", SERVICE_D_OUT);
  private TripPattern patternInT1;
  private TripPattern patternInT2;
  private OtpTransitServiceBuilder subject;

  @BeforeEach
  public void setUp() {
    subject = new OtpTransitServiceBuilder();

    // Add a service calendar that overlap with the period limit
    subject.getCalendars().add(createServiceCalendar(SERVICE_C_IN, D1, D3));

    // Add a service calendar that is outside the period limit, expected deleted later
    subject.getCalendars().add(createServiceCalendar(SERVICE_C_OUT, D0, D1));

    // Add a service calendar date that is within the period limit
    subject.getCalendarDates().add(new ServiceCalendarDate(SERVICE_D_IN, D2, 1));

    // Add a service calendar date that is OUTSIDE the period limit, expected deleted later
    subject.getCalendarDates().add(new ServiceCalendarDate(SERVICE_D_OUT, D1, 1));

    // Add 2 stops
    subject.getStops().add(STOP_1);
    subject.getStops().add(STOP_2);

    // Add Route
    subject.getRoutes().add(route);

    // Add trips; one for each day and calendar
    subject.getTripsById().addAll(List.of(tripCSIn, tripCSOut, tripCSDIn, tripCSDOut));

    // Pattern with trips that is partially deleted later
    patternInT1 = createTripPattern(List.of(tripCSIn, tripCSOut));
    // Pattern with trip that is inside period
    patternInT2 = createTripPattern(List.of(tripCSDIn));
    // Pattern with trip outside limiting period - pattern is deleted later
    TripPattern patternOut = createTripPattern(List.of(tripCSDOut));

    subject.getTripPatterns().put(STOP_PATTERN, patternInT1);
    subject.getTripPatterns().put(STOP_PATTERN, patternInT2);
    subject.getTripPatterns().put(STOP_PATTERN, patternOut);
  }

  @Test
  public void testLimitPeriod() {
    // Assert the test is set up as expected
    assertEquals(2, subject.getCalendars().size());
    assertEquals(2, subject.getCalendarDates().size());
    assertEquals(4, subject.getTripsById().size());
    assertEquals(3, subject.getTripPatterns().get(STOP_PATTERN).size());
    assertEquals(2, patternInT1.scheduledTripsAsStream().count());
    assertEquals(2, patternInT1.getScheduledTimetable().getTripTimes().size());
    assertEquals(1, patternInT2.scheduledTripsAsStream().count());
    assertEquals(1, patternInT2.getScheduledTimetable().getTripTimes().size());

    // Limit service to last half of month
    subject.limitServiceDays(new ServiceDateInterval(D2, D3), new DataImportIssueStore(false));

    // Verify calendar
    List<ServiceCalendar> calendars = subject.getCalendars();
    assertEquals(1, calendars.size(), calendars.toString());
    assertEquals(SERVICE_C_IN, calendars.get(0).getServiceId(), calendars.toString());

    // Verify calendar dates
    List<ServiceCalendarDate> dates = subject.getCalendarDates();
    assertEquals(1, dates.size(), dates.toString());
    assertEquals(SERVICE_D_IN, dates.get(0).getServiceId(), dates.toString());

    // Verify trips
    EntityById<Trip> trips = subject.getTripsById();
    assertEquals(2, trips.size(), trips.toString());
    assertTrue(trips.containsKey(tripCSIn.getId()), trips.toString());
    assertTrue(trips.containsKey(tripCSDIn.getId()), trips.toString());

    // Verify patterns
    Collection<TripPattern> patterns = subject.getTripPatterns().get(STOP_PATTERN);
    assertEquals(2, patterns.size());
    assertTrue(patterns.contains(patternInT1), patterns.toString());
    assertTrue(patterns.contains(patternInT2), patterns.toString());

    // Verify trips in pattern (one trip is removed from patternInT1)
    assertEquals(1, patternInT1.scheduledTripsAsStream().count());
    assertEquals(tripCSIn, patternInT1.scheduledTripsAsStream().findFirst().orElseThrow());

    // Verify trips in pattern is unchanged (one trip)
    assertEquals(1, patternInT2.scheduledTripsAsStream().count());

    // Verify scheduledTimetable trips (one trip is removed from patternInT1)
    assertEquals(1, patternInT1.getScheduledTimetable().getTripTimes().size());
    assertEquals(tripCSIn, patternInT1.getScheduledTimetable().getTripTimes().get(0).getTrip());

    // Verify scheduledTimetable trips in pattern is unchanged (one trip)
    assertEquals(1, patternInT2.getScheduledTimetable().getTripTimes().size());
  }

  private static ServiceCalendar createServiceCalendar(
    FeedScopedId serviceId,
    ServiceDate start,
    ServiceDate end
  ) {
    ServiceCalendar calendar = new ServiceCalendar();
    calendar.setPeriod(new ServiceDateInterval(start, end));
    calendar.setAllDays(1);
    calendar.setServiceId(serviceId);
    return calendar;
  }

  private static StopTime createStopTime(Stop stop, int time) {
    StopTime st = new StopTime();
    st.setStop(stop);
    st.setDepartureTime(time);
    st.setArrivalTime(time);
    st.setPickupType(PickDrop.NONE);
    st.setDropOffType(PickDrop.NONE);
    return st;
  }

  private static FeedScopedId newId() {
    return TransitModelForTest.id(Integer.toString(++SEQ_NR));
  }

  private TripPattern createTripPattern(Collection<Trip> trips) {
    FeedScopedId patternId = TransitModelForTest.id(
      trips.stream().map(t -> t.getId().getId()).collect(Collectors.joining(":"))
    );
    TripPattern p = new TripPattern(patternId, route, STOP_PATTERN);

    p.setName("Pattern");
    for (Trip trip : trips) {
      p.add(new TripTimes(trip, STOP_TIMES, DEDUPLICATOR));
    }
    return p;
  }

  private Trip createTrip(String id, FeedScopedId serviceId) {
    return TransitModelForTest
      .trip(id)
      .withServiceId(serviceId)
      .withDirection(Direction.valueOfGtfsCode(1))
      .withRoute(route)
      .build();
  }
}
