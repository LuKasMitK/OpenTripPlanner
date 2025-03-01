package org.opentripplanner.ext.legacygraphqlapi;

import org.opentripplanner.routing.RoutingService;
import org.opentripplanner.standalone.server.Router;
import org.opentripplanner.transit.service.TransitService;

public class LegacyGraphQLRequestContext {

  private final Router router;
  private final RoutingService routingService;
  private final TransitService transitService;

  public LegacyGraphQLRequestContext(
    Router router,
    RoutingService routingService,
    TransitService transitService
  ) {
    this.router = router;
    this.routingService = routingService;
    this.transitService = transitService;
  }

  public Router getRouter() {
    return router;
  }

  public RoutingService getRoutingService() {
    return routingService;
  }

  public TransitService getTransitService() {
    return transitService;
  }
}
