package org.opentripplanner.graph_builder.module.osm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import gnu.trove.list.TLongList;
import java.io.File;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentripplanner.graph_builder.DataImportIssueStore;
import org.opentripplanner.openstreetmap.OpenStreetMapProvider;
import org.opentripplanner.openstreetmap.model.OSMNode;
import org.opentripplanner.openstreetmap.model.OSMWay;

public class OpenStreetMapParserTest {

  @Test
  public void testBinaryParser() {
    File osmFile = new File(
      URLDecoder.decode(getClass().getResource("map.osm.pbf").getPath(), StandardCharsets.UTF_8)
    );
    OpenStreetMapProvider pr = new OpenStreetMapProvider(osmFile, true);
    OSMDatabase osmdb = new OSMDatabase(new DataImportIssueStore(false), Set.of());

    pr.readOSM(osmdb);

    assertEquals(2297, osmdb.nodeCount());

    OSMNode nodeA = osmdb.getNode(314192918L);
    assertEquals(314192918, nodeA.getId());
    assertEquals(52.3750447, nodeA.lat, 0.0000001);
    assertEquals(16.8431974, nodeA.lon, 0.0000001);
    assertTrue(nodeA.hasTag("railway"));
    assertEquals("level_crossing", nodeA.getTag("railway"));

    assertEquals(545, osmdb.wayCount());

    OSMWay wayA = osmdb.getWay(13490353L);
    assertEquals(13490353, wayA.getId());
    TLongList nodeRefsA = wayA.getNodeRefs();
    assertEquals(2, nodeRefsA.size());
    assertEquals(123978834, nodeRefsA.get(0));
    assertEquals(123980465, nodeRefsA.get(1));
    assertTrue(wayA.hasTag("highway"));
    assertEquals("Potlatch 0.9a", wayA.getTag("created_by"));
    assertEquals("secondary", wayA.getTag("highway"));
  }
}
