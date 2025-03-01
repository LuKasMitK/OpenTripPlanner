package org.opentripplanner.netex.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.opentripplanner.common.model.T2;
import org.opentripplanner.transit.model.network.TransitMode;
import org.rutebanken.netex.model.BusSubmodeEnumeration;
import org.rutebanken.netex.model.RailSubmodeEnumeration;
import org.rutebanken.netex.model.StopPlace;
import org.rutebanken.netex.model.VehicleModeEnumeration;

public class StopPlaceTypeMapperTest {

  private final StopPlaceTypeMapper stopPlaceTypeMapper = new StopPlaceTypeMapper();

  @Test
  public void mapWithoutTransportMode() {
    final T2<TransitMode, String> transitMode = stopPlaceTypeMapper.map(new StopPlace());
    assertNull(transitMode.first);
    assertNull(transitMode.second);
  }

  @Test
  public void mapWithTransportModeOnly() {
    final T2<TransitMode, String> transitMode = stopPlaceTypeMapper.map(
      new StopPlace().withTransportMode(VehicleModeEnumeration.RAIL)
    );
    assertEquals(TransitMode.RAIL, transitMode.first);
    assertNull(transitMode.second);
  }

  @Test
  public void mapWithSubMode() {
    final T2<TransitMode, String> transitMode = stopPlaceTypeMapper.map(
      new StopPlace()
        .withTransportMode(VehicleModeEnumeration.RAIL)
        .withRailSubmode(RailSubmodeEnumeration.REGIONAL_RAIL)
    );
    assertEquals(TransitMode.RAIL, transitMode.first);
    assertEquals("regionalRail", transitMode.second);
  }

  @Test
  public void mapWithSubModeOnly() {
    final T2<TransitMode, String> transitMode = stopPlaceTypeMapper.map(
      new StopPlace().withRailSubmode(RailSubmodeEnumeration.REGIONAL_RAIL)
    );
    assertEquals(TransitMode.RAIL, transitMode.first);
    assertEquals("regionalRail", transitMode.second);
  }

  @Test
  public void checkSubModePrecedensOverMainMode() {
    final T2<TransitMode, String> transitMode = stopPlaceTypeMapper.map(
      new StopPlace()
        .withTransportMode(VehicleModeEnumeration.RAIL)
        .withBusSubmode(BusSubmodeEnumeration.SIGHTSEEING_BUS)
    );
    assertEquals(TransitMode.BUS, transitMode.first);
    assertEquals("sightseeingBus", transitMode.second);
  }
}
