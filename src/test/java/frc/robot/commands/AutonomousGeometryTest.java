package frc.robot.commands;

import static frc.robot.Constants.OdometryConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.AutoConstants.kHopperReturnLeftTrimMeters;
import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

class AutonomousGeometryTest {
    private final ObjectMapper json = new ObjectMapper();
    private final Path deploy = Path.of("src/main/deploy/pathplanner");

    @Test
    void allLinkedHopperAndReturnWaypointsMatchTheCenteredPlacement() throws Exception {
        for (String suffix : new String[] {"", " 2x", " 3x"}) {
            for (String route : new String[] {"hopper front test L", "hopper front test R"}) {
                JsonNode reset = path((route.endsWith("L") ? "reset back L" : "reset back") + suffix).get("waypoints");
                JsonNode returnStart = reset.get(0).get("anchor");
                JsonNode returnEnd = reset.get(reset.size() - 1).get("anchor");
                assertEquals(kHopperCenterXMeters, returnEnd.get("x").asDouble(), 1e-9);
                assertEquals(kHopperCenterYMeters + (route.endsWith("L") ? kHopperReturnLeftTrimMeters : 0),
                    returnEnd.get("y").asDouble(), 1e-9);
                JsonNode points = path(route + suffix).get("waypoints");
                assertHopperCenter(points.get(0).get("anchor"));
                JsonNode end = points.get(points.size() - 1).get("anchor");
                assertEquals(returnStart.get("x").asDouble(), end.get("x").asDouble(), 1e-9);
                assertEquals(returnStart.get("y").asDouble(), end.get("y").asDouble(), 1e-9);
            }
        }
        JsonNode layout = json.readTree(getClass().getResourceAsStream(
            "/edu/wpi/first/apriltag/2026-rebuilt-welded.json"));
        for (JsonNode tag : layout.get("tags")) {
            if (tag.get("ID").asInt() == kBlueHopperTagId) {
                assertEquals(26, kBlueHopperTagId);
                assertEquals(10, kRedHopperTagId);
                assertEquals(tag.get("pose").get("translation").get("y").asDouble(), kHopperTagCenterYMeters, 1e-9);
                assertEquals(0.0, kHopperCenterYMeters - kHopperTagCenterYMeters, 1e-9);
                assertEquals(tag.get("pose").get("translation").get("x").asDouble(), kHopperTagFaceXMeters, 1e-9);
                return;
            }
        }
        fail("Official field layout must contain tag 26");
    }

    @Test
    void returningBumpersHaveClearanceOnBothSidesOfTheTrenchOpening() throws Exception {
        JsonNode layout = json.readTree(getClass().getResourceAsStream(
            "/edu/wpi/first/apriltag/2026-rebuilt-welded.json"));
        JsonNode settings = json.readTree(deploy.resolve("settings.json").toFile());
        double halfRobotWidth = settings.get("robotWidth").asDouble() / 2.0;
        // FIRST 2026 game manual section 5.6: 50.34 inches beneath the trench arm.
        double halfOpening = 50.34 * 0.0254 / 2.0;
        for (String side : new String[] {"L", "R"}) {
            int trenchId = side.equals("L") ? 22 : 17;
            double centerY = Double.NaN;
            for (JsonNode tag : layout.get("tags")) {
                if (tag.get("ID").asInt() == trenchId) {
                    centerY = tag.get("pose").get("translation").get("y").asDouble();
                }
            }
            assertTrue(Double.isFinite(centerY));
            for (String suffix : new String[] {"", " 2x", " 3x"}) {
                JsonNode points = path("hopper front test " + side + suffix).get("waypoints");
                double laneY = points.get(6).get("anchor").get("y").asDouble();
                assertEquals(centerY, laneY, 0.005);
                double lowerClearance = laneY - halfRobotWidth - (centerY - halfOpening);
                double upperClearance = centerY + halfOpening - (laneY + halfRobotWidth);
                assertTrue(lowerClearance >= 0.15, side + " outside/inside bumper margin=" + lowerClearance);
                assertTrue(upperClearance >= 0.15, side + " inside/outside bumper margin=" + upperClearance);
            }
        }
    }

    @Test
    void returnsStayStraightThroughTheTrenchThenTurnOnlyAfterClearingIt() throws Exception {
        JsonNode settings = json.readTree(deploy.resolve("settings.json").toFile());
        double bumperSweepRadius = Math.hypot(settings.get("robotWidth").asDouble(),
            settings.get("robotLength").asDouble()) / 2.0;
        // Conservatively bound every bumper corner at every turn angle. The hub tag
        // face is slightly closer than the trench's alliance-side edge.
        double maxTurnX = kHopperTagFaceXMeters - bumperSweepRadius - 0.15;
        for (String suffix : new String[] {"", " 2x", " 3x"}) {
            for (String route : new String[] {"hopper front test L", "hopper front test R"}) {
                JsonNode points = path(route + suffix).get("waypoints");
                JsonNode entry = points.get(6);
                JsonNode exit = points.get(7);
                double laneY = entry.get("anchor").get("y").asDouble();
                assertTrue(entry.get("anchor").get("x").asDouble() >= 6.25);
                assertTrue(exit.get("anchor").get("x").asDouble() <= maxTurnX);
                assertEquals(laneY, entry.get("nextControl").get("y").asDouble(), 1e-9);
                assertEquals(laneY, exit.get("prevControl").get("y").asDouble(), 1e-9);
                assertEquals(laneY, exit.get("anchor").get("y").asDouble(), 1e-9,
                    "The entire Bezier segment through the trench must be straight");
                // Everything after the straight exit remains on the alliance side, so a
                // wide return turn cannot sweep back toward the trench structure.
                for (int i = 7; i < points.size() - 1; i++) {
                    assertTrue(points.get(i).get("anchor").get("x").asDouble() <= maxTurnX);
                    assertTrue(points.get(i).get("nextControl").get("x").asDouble() <= maxTurnX);
                    assertTrue(points.get(i + 1).get("prevControl").get("x").asDouble() <= maxTurnX);
                    assertTrue(points.get(i + 1).get("anchor").get("x").asDouble() <= maxTurnX);
                }
                JsonNode approach = points.get(points.size() - 2);
                JsonNode end = points.get(points.size() - 1);
                boolean left = route.endsWith("L");
                double expectedX = left ? 2.8487018544935796 : 2.45;
                double expectedY = kHopperCenterYMeters + (left ? kHopperReturnLeftTrimMeters : 0);
                assertEquals(expectedX, end.get("anchor").get("x").asDouble(), 1e-9);
                assertEquals(expectedY, end.get("anchor").get("y").asDouble(), 1e-9);
                double reverseDistance = kHopperCenterXMeters - end.get("anchor").get("x").asDouble();
                assertTrue(reverseDistance > (left ? 0.5 : 0.9) && reverseDistance < (left ? 0.65 : 1.0),
                    "Leave correction room without the previous 1.48 m overshoot");
                assertEquals(expectedY, approach.get("anchor").get("y").asDouble(), 1e-9);
                assertEquals(expectedY, approach.get("nextControl").get("y").asDouble(), 1e-9);
                assertEquals(expectedY, end.get("prevControl").get("y").asDouble(), 1e-9);
                assertTrue(approach.get("anchor").get("x").asDouble()
                    - end.get("anchor").get("x").asDouble() >= (left ? 0.15 : 0.30) - 1e-9,
                    "Finish the turn before the straight handoff to reverse alignment");
            }
        }
    }

    @Test
    void shortenedRoutesKeepAllEventsAndConstraintZonesWithinTheirWaypoints() throws Exception {
        for (String side : new String[] {"L", "R"}) {
            for (String suffix : new String[] {"", " 2x", " 3x"}) {
                JsonNode route = path("hopper front test " + side + suffix);
                int last = route.get("waypoints").size() - 1;
                for (JsonNode zone : route.get("constraintZones")) {
                    assertTrue(zone.get("minWaypointRelativePos").asDouble() >= 0);
                    assertTrue(zone.get("maxWaypointRelativePos").asDouble() <= last);
                }
                for (JsonNode event : route.get("eventMarkers")) {
                    assertTrue(event.get("waypointRelativePos").asDouble() >= 0);
                    assertTrue(event.get("waypointRelativePos").asDouble() <= last);
                    if (event.hasNonNull("endWaypointRelativePos")) {
                        assertTrue(event.get("endWaypointRelativePos").asDouble() <= last,
                            "An intake zone must end before the shortened path ends");
                    }
                }
            }
        }
    }

    @Test
    void everyTrenchRouteReacquiresBeforeReturningAndAlignsAfterward() throws Exception {
        for (String name : new String[] {"Center Left", "Center Right", "Center Left 2x", "Center Right 2x",
                "Center Left 3x", "Center Right 3x"}) {
            JsonNode auto = json.readTree(deploy.resolve("autos/" + name + ".auto").toFile());
            assertFalse(auto.get("resetOdom").asBoolean(), "Explicit hopperStart owns the exact starting pose");
            JsonNode commands = auto.get("command").get("data").get("commands");
            assertNamed(commands.get(0), "hopperStart");
            int returns = 0;
            for (int i = 0; i < commands.size(); i++) {
                JsonNode command = commands.get(i);
                if ("named".equals(command.get("type").asText())
                        && command.get("data").path("name").asText().startsWith("resetBack")) {
                    returns++;
                    assertTrue(i >= 2 && i + 1 < commands.size());
                    assertNamed(commands.get(i - 1), "postTrenchRelocalize");
                    assertTrue(commands.get(i - 2).get("data").path("pathName").asText()
                        .startsWith("hopper front test"));
                    assertNamed(commands.get(i + 1), "visionAlign");
                }
            }
            assertEquals(2, returns);
        }
    }

    private JsonNode path(String name) throws Exception {
        return json.readTree(deploy.resolve("paths/" + name + ".path").toFile());
    }

    private static void assertHopperCenter(JsonNode anchor) {
        assertEquals(kHopperCenterXMeters, anchor.get("x").asDouble(), 1e-9);
        assertEquals(kHopperCenterYMeters, anchor.get("y").asDouble(), 1e-9);
    }

    private static void assertNamed(JsonNode command, String name) {
        assertEquals("named", command.get("type").asText());
        assertEquals(name, command.get("data").get("name").asText());
    }
}
