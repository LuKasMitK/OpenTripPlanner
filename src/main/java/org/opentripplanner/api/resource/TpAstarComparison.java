package org.opentripplanner.api.resource;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.util.Calendar;
import java.util.GregorianCalendar;
import java.util.Random;
import java.util.Set;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.joda.time.LocalDate;
import org.opentripplanner.analyst.PointFeature;
import org.opentripplanner.analyst.PointSet;
import org.opentripplanner.analyst.SampleSet;
import org.opentripplanner.api.parameter.QualifiedModeSet;
import org.opentripplanner.common.geometry.SphericalDistanceLibrary;
import org.opentripplanner.profile.ProfileRequest;
import org.opentripplanner.profile.RepeatedRaptorProfileRouter;
import org.opentripplanner.routing.core.RoutingContext;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseModeSet;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.impl.GraphPathFinder;
import org.opentripplanner.routing.impl.TransferPatternPathFinder;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.opentripplanner.standalone.Router;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Iterables;
import com.google.common.collect.Sets;

/**
 * Computes comparison between AStar, RAPTOR and TP using a random subset of
 * source and target stops
 * 
 * @author Sebastian Peter
 *
 */
public class TpAstarComparison {

	private static final Logger LOG = LoggerFactory.getLogger(TpAstarComparison.class);

	/**
	 * Create comparison and save as csv
	 * 
	 * @param router
	 *            router to use
	 */
	public TpAstarComparison(Router router) {
		LOG.info("Setting up comparison...");

		Graph graph = router.graph;

		Set<TransitStationStop> allStops = Sets.newHashSet();
		for (TransitStationStop s : Iterables.filter(graph.getVertices(), TransitStationStop.class)) {
			allStops.add(s);
		}

		final String fileName = "tp_astar_comparison.csv";
		FileWriter fileWriter = null;
		CSVPrinter csvPrinter = null;
		try {
			fileWriter = new FileWriter(fileName);
			csvPrinter = new CSVPrinter(fileWriter, CSVFormat.DEFAULT);
			// headers
			csvPrinter.printRecord(new Object[] { "Origin", "Destination", "Distance (m)", "Dijkstra Rank", "TP (ns)",
					"A* (ns)", "RAPTOR (ns)" });

			final ThreadMXBean threadBean = ManagementFactory.getThreadMXBean();
			final long currentThread = Thread.currentThread().getId();

			Calendar cal = GregorianCalendar.getInstance();
			cal.set(Calendar.YEAR, 2017);
			cal.set(Calendar.MONTH, Calendar.FEBRUARY);
			cal.set(Calendar.DAY_OF_MONTH, 10);
			cal.set(Calendar.HOUR_OF_DAY, 12);
			cal.set(Calendar.MINUTE, 0);
			cal.set(Calendar.SECOND, 0);
			cal.set(Calendar.MILLISECOND, 0);
			final long dateTime = cal.getTimeInMillis() / 1000;

			Random rand = new Random();

			final int everyXth = allStops.size() * allStops.size() / 2000;

			// prepare graph tree index for RAPTOR
			router.graph.index.getStopTreeCache();

			for (TransitStationStop from : allStops) {
				for (TransitStationStop to : allStops) {
					if (from == to)
						continue;

					// only take 1 in x
					if (rand.nextInt(everyXth) > 0)
						continue;

					LOG.info("Calculating route from " + from.getName() + " to " + to.getName());

					RoutingRequest request = createTPRoutingRequest(graph, from, to, dateTime);

					// building RAPTOR request
					ProfileRequest raptorRequest = new ProfileRequest();
					raptorRequest.fromTime = 12 * 60 * 60;
					raptorRequest.toTime = raptorRequest.fromTime;
					raptorRequest.fromLat = request.rctx.fromVertex.getLat();
					raptorRequest.fromLon = request.rctx.fromVertex.getLon();
					raptorRequest.toLat = request.rctx.toVertex.getLat();
					raptorRequest.toLon = request.rctx.toVertex.getLon();
					raptorRequest.walkSpeed = 2;
					raptorRequest.bikeSpeed = 4;
					raptorRequest.carSpeed = 8;
					raptorRequest.date = new LocalDate(cal.get(Calendar.YEAR), cal.get(Calendar.MONTH),
							cal.get(Calendar.DAY_OF_MONTH));
					raptorRequest.maxWalkTime = 20; // minutes
					raptorRequest.accessModes = new QualifiedModeSet("WALK");
					raptorRequest.egressModes = new QualifiedModeSet("WALK");
					raptorRequest.transitModes = new TraverseModeSet("TRANSIT");

					PointFeature pointFeature = new PointFeature();
					pointFeature.setLat(raptorRequest.toLat);
					pointFeature.setLon(raptorRequest.toLon);
					PointSet pointSet = new PointSet(1);
					pointSet.addFeature(pointFeature, 0);
					SampleSet sampleSet = new SampleSet(pointSet, graph.getSampleFactory());
					// END building RAPTOR request

					boolean tpError = false, aStarError = false, raptorError = false;
					int visitedNodes = -1;

					long time1 = threadBean.getThreadCpuTime(currentThread);

					try {
						TransferPatternPathFinder tpPathFinder = new TransferPatternPathFinder(router);
						tpPathFinder.findJourneys(request.clone());
					} catch (Exception e) {
						tpError = true;
						e.printStackTrace();
					}

					long time2 = threadBean.getThreadCpuTime(currentThread);

					try {
						GraphPathFinder gpFinder = new GraphPathFinder(router);
						gpFinder.graphPathFinderEntryPoint(request.clone());
						visitedNodes = gpFinder.getVisitedNodeCount();
					} catch (Exception e) {
						tpError = true;
					}

					long time3 = threadBean.getThreadCpuTime(currentThread);

					try {
						RepeatedRaptorProfileRouter raptorRouter = new RepeatedRaptorProfileRouter(graph, raptorRequest,
								sampleSet);
						raptorRouter.route();
					} catch (Exception e) {
						tpError = true;
					}

					long time4 = threadBean.getThreadCpuTime(currentThread);

					final String tpTime = getResult(time2 - time1, tpError);
					final String aStarTime = getResult(time3 - time2, aStarError);
					final String raptorTime = getResult(time4 - time3, raptorError);

					// skip when no path was found
					if (visitedNodes == -1)
						continue;

					double linearDistance = SphericalDistanceLibrary.distance(from.getCoordinate(), to.getCoordinate());
					csvPrinter.printRecord(new Object[] { from.getName(), to.getName(), String.valueOf(linearDistance),
							visitedNodes, tpTime, aStarTime, raptorTime });
				}
			}

			LOG.info("Done comparing");
		} catch (

		IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if (fileWriter != null) {
					fileWriter.flush();
					fileWriter.close();
				}
				if (csvPrinter != null)
					csvPrinter.close();

				LOG.info("Done writing to CSV");
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

	}

	private static String getResult(long timeDiff, boolean error) {
		if (error)
			return "---";

		return String.valueOf(timeDiff);
	}

	private static RoutingRequest createTPRoutingRequest(Graph graph, TransitStationStop from, TransitStationStop to,
			long dateTime) {
		RoutingRequest req = new RoutingRequest();
		req.modes = new TraverseModeSet(TraverseMode.TRANSIT, TraverseMode.WALK);
		req.dateTime = dateTime;
		// traversal modes
		req.maxWalkDistance = 500;
		req.softWalkLimiting = false;

		// set for one-to-all search
		req.ignoreRealtimeUpdates = true;
		req.useTraffic = false;

		req.rctx = new RoutingContext(req, graph, from, to);

		return req;
	}
}