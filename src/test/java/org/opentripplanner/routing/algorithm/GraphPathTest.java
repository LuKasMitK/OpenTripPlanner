package org.opentripplanner.routing.algorithm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.opentripplanner.gtfs.GtfsContextBuilder.contextBuilder;

import com.google.common.collect.Lists;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.opentripplanner.ConstantsForTests;
import org.opentripplanner.graph_builder.module.geometry.GeometryAndBlockProcessor;
import org.opentripplanner.gtfs.GtfsContext;
import org.opentripplanner.model.calendar.CalendarServiceData;
import org.opentripplanner.routing.algorithm.astar.AStarBuilder;
import org.opentripplanner.routing.api.request.RoutingRequest;
import org.opentripplanner.routing.core.RoutingContext;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.routing.vertextype.TransitStopVertex;
import org.opentripplanner.util.TestUtils;

/**
 * TODO OTP2 - Test is too close to the implementation and will need to be reimplemented.
 */
@Disabled
public class GraphPathTest {

  private Graph graph;

  @BeforeEach
  public void setUp() throws Exception {
    GtfsContext context = contextBuilder(ConstantsForTests.FAKE_GTFS).build();
    graph = new Graph();
    GeometryAndBlockProcessor hl = new GeometryAndBlockProcessor(context);
    hl.run(graph);
    graph.putService(CalendarServiceData.class, context.getCalendarServiceData());
  }

  @Test
  public void testGraphPathOptimize() {
    String feedId = graph.getFeedIds().iterator().next();

    Vertex stop_a = graph.getVertex(feedId + ":A");
    Vertex stop_c = graph.getVertex(feedId + ":C");
    Vertex stop_e = graph.getVertex(feedId + ":E");

    ShortestPathTree spt;
    GraphPath path;

    RoutingRequest options = new RoutingRequest();
    options.setDateTime(TestUtils.dateInstant("America/New_York", 2009, 8, 7, 0, 0, 0));
    spt =
      AStarBuilder
        .oneToOne()
        .setContext(new RoutingContext(options, graph, stop_a, stop_e))
        .getShortestPathTree();

    path = spt.getPath(stop_e);/* do not optimize yet, since we are testing optimization */
    assertNotNull(path);

    // Check that the resulting path visits the stops in the right order.
    List<Vertex> stopvs = Lists.newArrayList();
    for (State state : path.states) {
      if (state.getVertex() instanceof TransitStopVertex) {
        stopvs.add(state.getVertex());
      }
    }
    assertSame(stopvs.get(0), stop_a);
    assertSame(stopvs.get(1), stop_c);
    assertSame(stopvs.get(2), stop_e);

    long bestStart = TestUtils.dateInSeconds("America/New_York", 2009, 8, 7, 0, 20, 0);
    assertNotSame(bestStart, path.getStartTime());

    path = spt.getPath(stop_e);/* optimize */
    assertEquals(bestStart, path.getStartTime());
  }
}
