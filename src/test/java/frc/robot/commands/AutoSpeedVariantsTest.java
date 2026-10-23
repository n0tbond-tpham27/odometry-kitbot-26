package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Path;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Commands;

class AutoSpeedVariantsTest {
    private static final String[] PATHS = {"hopper front test L", "hopper front test R", "reset back"};
    private final ObjectMapper json = new ObjectMapper();
    private final Path deploy = Path.of("src/main/deploy/pathplanner");

    @BeforeAll
    static void initializePathPlannerDependencies() {
        assertTrue(HAL.initialize(500, 0));
        NamedCommands.registerCommand("intake", Commands.none());
    }

    @Test
    void fasterPathsPreserveGeometryReturnOffsetAndAccelerationLimits() throws Exception {
        for (int multiplier : new int[] {2, 3}) {
            for (String name : PATHS) {
                JsonNode original = read("paths", name, ".path");
                JsonNode variant = read("paths", name + " " + multiplier + "x", ".path");
                assertEquals(original.get("waypoints"), variant.get("waypoints"));
                assertEquals(original.get("eventMarkers"), variant.get("eventMarkers"));
                assertEquals(original.get("rotationTargets"), variant.get("rotationTargets"));
                assertEquals(original.get("pointTowardsZones"), variant.get("pointTowardsZones"));
                assertEquals(original.get("reversed"), variant.get("reversed"));
                assertEquals(original.get("goalEndState"), variant.get("goalEndState"));
                assertEquals(original.get("idealStartingState"), variant.get("idealStartingState"));
                assertFalse(variant.get("useDefaultConstraints").asBoolean());
                int returnMultiplier = name.equals("reset back") ? 1 : multiplier;
                assertSpeedConstraints(original.get("globalConstraints"), variant.get("globalConstraints"), returnMultiplier);
                if (name.equals("reset back")) {
                    assertEquals(0.25, variant.get("globalConstraints").get("maxVelocity").asDouble(), 1e-9);
                    assertEquals(0.6, variant.get("globalConstraints").get("maxAcceleration").asDouble(), 1e-9);
                }
                JsonNode originalZones = original.get("constraintZones");
                JsonNode variantZones = variant.get("constraintZones");
                assertEquals(originalZones.size(), variantZones.size());
                for (int i = 0; i < originalZones.size(); i++) {
                    assertEquals(originalZones.get(i).get("minWaypointRelativePos"), variantZones.get(i).get("minWaypointRelativePos"));
                    assertEquals(originalZones.get(i).get("maxWaypointRelativePos"), variantZones.get(i).get("maxWaypointRelativePos"));
                    int zoneMultiplier = "Trench Return".equals(originalZones.get(i).path("name").asText())
                        ? 1 : returnMultiplier;
                    assertSpeedConstraints(originalZones.get(i).get("constraints"), variantZones.get(i).get("constraints"), zoneMultiplier);
                }
            }
        }
    }

    @Test
    void allReturnTrenchTrajectoriesStayCenteredAndSlowBeforeThePostTrenchTurn() throws Exception {
        RobotConfig config = RobotConfig.fromGUISettings();
        for (String side : new String[] {"L", "R"}) {
            double laneY = side.equals("L") ? 7.424674 : 0.644398;
            for (String suffix : new String[] {"", " 2x", " 3x"}) {
                String name = "hopper front test " + side + suffix;
                var trajectory = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
                int crossingStates = 0;
                var states = trajectory.getStates();
                for (int i = 1; i < states.size(); i++) {
                    var current = states.get(i);
                    var previous = states.get(i - 1);
                    if (current.pose.getX() > 3.5 && current.pose.getX() < 5.7
                            && Math.abs(current.pose.getY() - laneY) < 0.2
                            && Math.abs(Math.abs(current.pose.getRotation().getDegrees()) - 180.0) < 2.0
                            && Math.abs(Math.abs(previous.pose.getRotation().getDegrees()) - 180.0) < 2.0) {
                        crossingStates++;
                        assertEquals(laneY, current.pose.getY(), 0.005, name);
                        double dt = current.timeSeconds - previous.timeSeconds;
                        assertTrue(dt > 0, name);
                        double speed = current.pose.getTranslation().getDistance(previous.pose.getTranslation()) / dt;
                        assertTrue(speed <= 0.46, name + " return trench speed=" + speed);
                    }
                }
                assertTrue(crossingStates > 5, name + " must contain a slow straight return through the trench");
            }
        }
    }

    @Test
    void eachFastAutoUsesOnlyMatchingFastPathsAndPreservesMechanismAndVisionSteps() throws Exception {
        for (String name : new String[] {"Center Left", "Center Right"}) {
            for (int multiplier : new int[] {2, 3}) {
                JsonNode original = read("autos", name, ".auto");
                JsonNode variant = read("autos", name + " " + multiplier + "x", ".auto");
                assertEquals(original.get("resetOdom"), variant.get("resetOdom"));
                assertEquals(original.get("choreoAuto"), variant.get("choreoAuto"));
                JsonNode steps = original.get("command").get("data").get("commands");
                JsonNode fastSteps = variant.get("command").get("data").get("commands");
                assertEquals(steps.size(), fastSteps.size());
                for (int i = 0; i < steps.size(); i++) {
                    if ("path".equals(steps.get(i).get("type").asText())) {
                        assertEquals("path", fastSteps.get(i).get("type").asText());
                        String fastPath = fastSteps.get(i).get("data").get("pathName").asText();
                        assertEquals(steps.get(i).get("data").get("pathName").asText() + " " + multiplier + "x", fastPath);
                        assertTrue(deploy.resolve("paths/" + fastPath + ".path").toFile().isFile());
                    } else if ("resetBack".equals(steps.get(i).get("data").path("name").asText())) {
                        assertEquals("named", fastSteps.get(i).get("type").asText());
                        assertEquals("resetBack" + multiplier + "x", fastSteps.get(i).get("data").get("name").asText());
                    } else {
                        assertEquals(steps.get(i), fastSteps.get(i));
                    }
                }
            }
        }
    }

    @Test
    void pathPlannerGeneratesFiniteFasterTrajectoriesWithTheInstalledRobotConfiguration() throws Exception {
        RobotConfig config = RobotConfig.fromGUISettings();
        for (String name : PATHS) {
            double baseTime = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow().getTotalTimeSeconds();
            assertTrue(Double.isFinite(baseTime) && baseTime > 0.0);
            double previousTime = baseTime;
            for (int multiplier : new int[] {2, 3}) {
                String fastName = name + " " + multiplier + "x";
                var path = PathPlannerPath.fromPathFile(fastName);
                var trajectory = path.getIdealTrajectory(config).orElseThrow();
                double fastTime = trajectory.getTotalTimeSeconds();
                assertTrue(Double.isFinite(fastTime) && fastTime > 0.0, fastName);
                if (name.equals("reset back")) {
                    assertEquals(baseTime, fastTime, 1e-6, "All returns keep the slow approach speed");
                    double previousX = Double.NEGATIVE_INFINITY;
                    for (var state : trajectory.getStates()) {
                        assertTrue(state.pose.getX() >= previousX - 1e-6, "Return must not loop backward");
                        // FollowPathCommand applies reverse() at execution for differential drive.
                        assertEquals(180.0, Math.abs(state.reverse().pose.getRotation().getDegrees()), 1e-6,
                            "Reversed return must keep the rear toward the hub");
                        previousX = state.pose.getX();
                    }
                } else {
                    assertTrue(fastTime < baseTime, fastName + " should drive faster than the original");
                }
                assertTrue(fastTime <= previousTime + 1e-6, fastName + " should not slow down at a higher speed setting");
                for (var state : trajectory.getStates()) {
                    assertTrue(Double.isFinite(state.timeSeconds), fastName);
                    assertTrue(Double.isFinite(state.pose.getX()) && Double.isFinite(state.pose.getY()), fastName);
                }
                previousTime = fastTime;
            }
        }
    }

    private JsonNode read(String folder, String name, String extension) throws Exception {
        return json.readTree(deploy.resolve(folder + "/" + name + extension).toFile());
    }

    private static void assertSpeedConstraints(JsonNode original, JsonNode variant, int multiplier) {
        assertEquals(original.get("maxVelocity").asDouble() * multiplier, variant.get("maxVelocity").asDouble(), 1e-9);
        for (String unchanged : new String[] {"maxAcceleration", "maxAngularVelocity", "maxAngularAcceleration", "nominalVoltage", "unlimited"}) {
            assertEquals(original.get(unchanged), variant.get(unchanged), unchanged);
        }
    }
}
