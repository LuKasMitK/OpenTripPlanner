package org.opentripplanner.routing.graph;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Map.Entry;

import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.TransferPattern.TPNode;
import org.opentripplanner.routing.graph.TransferPattern.TPTravel;
import org.opentripplanner.routing.graph.TransferPatternGraph.TripPatternWithPosition;
import org.opentripplanner.routing.impl.InputStreamGraphSource;
import org.opentripplanner.routing.services.GraphService;
import org.opentripplanner.routing.services.GraphSource;
import org.opentripplanner.routing.vertextype.PatternArriveVertex;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Iterables;

/**
 * Merges chunks of transfer patterns into one transfer pattern graph
 * 
 * @author Sebastian Peter
 *
 */
public class TransferPatternGraphMerger {
	private static final Logger LOG = LoggerFactory.getLogger(TransferPatternGraphMerger.class);

	public static final String CHUNK_PREFIX = "chunk_";
	public static final String MERGED_PREFIX = "merged";

	private File graphDirectory;
	private GraphService graphService;

	private Graph start;
	private Map<String, TripPattern> codeToTripPattern;

	/**
	 * Creates a new graph merger for graph chunks in given directory
	 * 
	 * @param graphDirectory
	 *            directory of graph chunks
	 * @param graphService
	 *            graph service used to load and dispose graphs
	 * @param m
	 *            total number of chunks to merge
	 */
	public TransferPatternGraphMerger(File graphDirectory, GraphService graphService, final int m) {
		this.graphDirectory = graphDirectory;
		this.graphService = graphService;

		LOG.info("Merging " + m + " graphs into one");

		// register first graph
		String name = getChunkName(1, m);
		GraphSource startGraphSource = getGraphSource(name);
		start = startGraphSource.getRouter().graph;

		codeToTripPattern = new HashMap<>();
		for (PatternArriveVertex pav : Iterables.filter(start.getVertices(), PatternArriveVertex.class)) {
			TripPattern tripPattern = pav.getTripPattern();
			codeToTripPattern.put(tripPattern.code, tripPattern);
		}

		// merge all other graphs into this
		for (int i = 2; i <= m; i++) {
			merge(i, m);
		}

		// save final merged graph
		saveGraph();
	}

	/**
	 * Merges chunk number n into the first chunk. Graph objects such as
	 * vertices and trip patterns are replaced by the version of the first
	 * chunk.
	 * 
	 * @param n
	 *            number of chunk to merge into first
	 * @param m
	 *            total number of chunks
	 */
	public void merge(int n, int m) {
		String name = getChunkName(n, m);
		GraphSource source = getGraphSource(name);
		Graph graph = source.getRouter().graph;

		// replacing all graph objects in direct connections
		for (Entry<TransitStationStop, TripPatternWithPosition> entry : graph.tp.getDirectConnections().entries()) {
			TripPatternWithPosition tripPatternWithPos = entry.getValue();
			tripPatternWithPos.tripPattern = replaceTripPattern(tripPatternWithPos.tripPattern);

			TransitStationStop newOrigin = replaceVertex(entry.getKey());

			start.tp.getDirectConnections().put(newOrigin, tripPatternWithPos);
		}

		// replacing all graph objects in transfer patterns
		for (Entry<TransitStationStop, TransferPattern> entry : graph.tp.getTransferPatterns().entrySet()) {
			TransferPattern tp = entry.getValue();
			replaceTpObj(tp);

			start.tp.getTransferPatterns().put(tp.getSourceStop(), tp);
		}

		// all done, throw graph source out
		LOG.debug("Throwing out graph " + n + " in order to save memory");
		graphService.evictRouter(name);
	}

	/**
	 * Returns graph source for graph with given name
	 * 
	 * @param name
	 *            name of the graph source
	 * @return graph source
	 */
	private GraphSource getGraphSource(String name) {
		GraphSource source = graphService.getGraphSourceFactory().createGraphSource(name);
		LOG.info("\tLoading graph " + name + "...");
		graphService.registerGraph(name, source);

		if (source.getRouter() == null)
			throw new RuntimeException("Graph " + name + " does not exist.");

		return source;
	}

	/**
	 * Returns name of nth chunk
	 * 
	 * @param n
	 *            number of chunk
	 * @param m
	 *            total number of chunks
	 * @return its name
	 */
	private static String getChunkName(int n, int m) {
		return CHUNK_PREFIX + n + "_" + m;
	}

	/**
	 * Replaces all OTP objects in this transfer pattern with ones of the first
	 * graph
	 * 
	 * @param tp
	 *            transfer pattern to replace OTP objects of
	 */
	private void replaceTpObj(TransferPattern tp) {
		tp.setSourceStop(replaceVertex(tp.getSourceStop()));

		Collection<TPNode> oldTargetNodes = tp.getTargetNodes();
		HashMap<TransitStationStop, TPNode> newTargetNodes = new HashMap<>(oldTargetNodes.size());

		for (TPNode targetNode : oldTargetNodes) {
			replaceTpNodeObj(targetNode);
			newTargetNodes.put(targetNode.getStop(), targetNode);
		}

		tp.setMap(newTargetNodes);
	}

	/**
	 * Replaces all OTP objects in this TP node with ones of the first graph
	 * 
	 * @param node
	 *            node to replace OTP objects of
	 */
	private void replaceTpNodeObj(TPNode node) {
		node.setStop(replaceVertex(node.getStop()));

		for (TPTravel travel : node) {
			if (travel.getDelayClass() != null) {
				List<TripPattern> tripPatterns = travel.getDelayClass().getTripPatterns();
				ListIterator<TripPattern> tripIt = tripPatterns.listIterator();

				while (tripIt.hasNext()) {
					TripPattern tripPattern = tripIt.next();
					TripPattern newTripPattern = replaceTripPattern(tripPattern);

					tripIt.set(newTripPattern);
				}
			}

			replaceTpNodeObj(travel.getNode());
		}
	}

	/**
	 * Returns the respective vertex in first graph of given vertex
	 * 
	 * @param oldVertex
	 *            vertex to search a replacement for
	 * @return proper replacement vertex
	 */
	private TransitStationStop replaceVertex(TransitStationStop oldVertex) {
		Vertex newVertex = start.getVertex(oldVertex.getLabel());

		if (newVertex == null) {
			throw new RuntimeException("Vertex not found with label " + oldVertex.getLabel());
		}

		return (TransitStationStop) newVertex;
	}

	/**
	 * Returns the respective trip pattern in first graph of given trip patternf
	 * 
	 * @param oldTripPattern
	 *            trip pattern to search a replacement for
	 * @return proper replacement trip pattern
	 */
	private TripPattern replaceTripPattern(TripPattern oldTripPattern) {
		TripPattern newTripPattern = codeToTripPattern.get(oldTripPattern.code);

		if (newTripPattern == null) {
			throw new RuntimeException("TripPattern not found with code " + oldTripPattern.code);
		}

		return newTripPattern;
	}

	/**
	 * Saves merged graph to disk
	 */
	private void saveGraph() {
		LOG.info("\tSaving merged graph...");

		File dir = new File(graphDirectory, MERGED_PREFIX);
		if (!dir.exists())
			dir.mkdir();

		File file = new File(dir, InputStreamGraphSource.GRAPH_FILENAME);
		try {
			start.save(file);
		} catch (IOException e) {
			LOG.error("Saving transfer patterns with graph failed.");
			e.printStackTrace();
		}
	}
}
