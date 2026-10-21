package frc.robot.commands;

import static frc.robot.Constants.OdometryConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

class AutonomousGeometryTest {
    private final ObjectMapper json = new ObjectMapper();
    private final Path deploy = Path.of("src/main/deploy/pathplanner");

    @Test
    void linkedHopperAndReturnWaypointsMatchShiftedAlignmentTarget() throws Exception {
        JsonNode reset = path("reset back").get("waypoints");
        JsonNode returnStart = reset.get(0).get("anchor");
        JsonNode returnEnd = reset.get(reset.size() - 1).get("anchor");
        assertHopperCenter(returnEnd);
        for (String route : new String[] {"hopper front test L", "hopper front test R"}) {
            JsonNode points = path(route).get("waypoints");
            assertHopperCenter(points.get(0).get("anchor"));
            JsonNode end = points.get(points.size() - 1).get("anchor");
            assertEquals(returnStart.get("x").asDouble(), end.get("x").asDouble(), 1e-9);
            assertEquals(returnStart.get("y").asDouble(), end.get("y").asDouble(), 1e-9);
        }
        JsonNode layout = json.readTree(getClass().getResourceAsStream(
            "/edu/wpi/first/apriltag/2026-rebuilt-welded.json"));
        for (JsonNode tag : layout.get("tags")) {
            if (tag.get("ID").asInt() == kBlueHopperTagId) {
                assertEquals(26, kBlueHopperTagId);
                assertEquals(10, kRedHopperTagId);
                assertEquals(tag.get("pose").get("translation").get("y").asDouble(), kHopperCenterYMeters, 1e-9);
                assertEquals(tag.get("pose").get("translation").get("x").asDouble(), kHopperTagFaceXMeters, 1e-9);
                return;
            }
        }
        fail("Official field layout must contain tag 26");
    }

    @Test
    void everyTrenchRouteReacquiresBeforeReturningAndAlignsAfterward() throws Exception {
        for (String name : new String[] {"Center Left", "Center Right"}) {
            JsonNode commands = json.readTree(deploy.resolve("autos/" + name + ".auto").toFile())
                .get("command").get("data").get("commands");
            int returns = 0;
            for (int i = 0; i < commands.size(); i++) {
                JsonNode command = commands.get(i);
                if ("path".equals(command.get("type").asText())
                        && "reset back".equals(command.get("data").path("pathName").asText())) {
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
