package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
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
    private static final String[] PATHS = {"hopper front test L", "hopper front test L repeat", "hopper front test R", "reset back"};
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
                    assertEquals(kHopperReturnCruiseSpeedMetersPerSecond,
                        variant.get("globalConstraints").get("maxVelocity").asDouble(), 1e-9);
                    assertEquals(0.6, variant.get("globalConstraints").get("maxAcceleration").asDouble(), 1e-9);
                }
                JsonNode originalZones = original.get("constraintZones");
                JsonNode variantZones = variant.get("constraintZones");
                assertEquals(originalZones.size(), variantZones.size());
                for (int i = 0; i < originalZones.size(); i++) {
                    assertEquals(originalZones.get(i).get("minWaypointRelativePos"), variantZones.get(i).get("minWaypointRelativePos"));
                    assertEquals(originalZones.get(i).get("maxWaypointRelativePos"), variantZones.get(i).get("maxWaypointRelativePos"));
                    int zoneMultiplier = java.util.Set.of("Trench Return", "Midfield Intake", "Trench Entry Turn",
                        "Trench Reentry", "Post-Trench Turn")
                        .contains(originalZones.get(i).path("name").asText())
                        ? 1 : returnMultiplier;
                    assertSpeedConstraints(originalZones.get(i).get("constraints"), variantZones.get(i).get("constraints"), zoneMultiplier);
                }
            }
        }
    }

    @Test
    void leftMidfieldActuallySlowsInGeneratedTrajectoriesForEverySpeedAndCycle() throws Exception {
        RobotConfig config = RobotConfig.fromGUISettings();
        for (String cycle : new String[] {"", " repeat"}) {
            for (String suffix : new String[] {"", " 2x", " 3x"}) {
                String name = "hopper front test L" + cycle + suffix;
                int midfieldStates = 0;
                for (var state : PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow().getStates()) {
                    // Both collection passes lie entirely within the midfield zone here.
                    if (state.pose.getX() > 6.0 && state.pose.getY() > 2.5 && state.pose.getY() < 5.5) {
                        midfieldStates++;
                        assertTrue(Math.abs(state.linearVelocity) <= 1.001, name + " must respect the midfield limit");
                    }
                }
                assertTrue(midfieldStates > 20, name);
            }
        }
    }

    @Test
    void leftTrimMatchesFallbackAndSecondCycleWithoutMovingTheInitialPlacement() throws Exception {
        RobotConfig config = RobotConfig.fromGUISettings();
        for (String suffix : new String[] {"", " 2x", " 3x"}) {
            String returnName = "reset back L" + suffix;
            assertEquals(0.2, Autos.hopperReturnLeftTrim(returnName), 1e-9);
            assertEquals(0.0, Autos.hopperReturnLeftTrim("reset back" + suffix), 1e-9);
            var fallback = PathPlannerPath.fromPathFile(returnName).getIdealTrajectory(config).orElseThrow();
            var end = fallback.getStates().get(fallback.getStates().size() - 1).reverse().pose;
            var repeat = PathPlannerPath.fromPathFile("hopper front test L repeat" + suffix)
                .getIdealTrajectory(config).orElseThrow().getStates().get(0).pose;
            assertEquals(end.getX(), repeat.getX(), 1e-9);
            assertEquals(end.getY(), repeat.getY(), 1e-9);
            assertEquals(0, end.getRotation().minus(repeat.getRotation()).getDegrees(), 1e-6);
            var commands = read("autos", "Center Left" + suffix, ".auto").get("command").get("data").get("commands");
            int paths = 0;
            for (var step : commands) {
                if (step.get("type").asText().equals("path")) {
                    assertEquals("hopper front test L" + (paths++ == 0 ? "" : " repeat") + suffix,
                        step.get("data").get("pathName").asText());
                }
            }
            assertEquals(2, paths);
        }
    }

    @Test
    void reverseFallbackCruisesFasterButSlowsBeforeTheHubWithBrakingRoom() throws Exception {
        RobotConfig config = RobotConfig.fromGUISettings();
        for (String suffix : new String[] {"", " 2x", " 3x"}) {
            String name = "reset back" + suffix;
            JsonNode data = read("paths", name, ".path");
            JsonNode zone = data.get("constraintZones").get(0);
            assertEquals("Hopper Final Approach", zone.get("name").asText());
            assertEquals(kHopperApproachMaxSpeedMetersPerSecond,
                zone.get("constraints").get("maxVelocity").asDouble(), 1e-9);
            double goalX = data.get("waypoints").get(1).get("anchor").get("x").asDouble();
            double length = goalX - data.get("waypoints").get(0).get("anchor").get("x").asDouble();
            double slowDistance = length * (1.0 - zone.get("minWaypointRelativePos").asDouble());
            assertEquals(kHopperReturnSlowdownDistanceMeters, slowDistance, 1e-9);
            double cruiseStoppingDistance = Math.pow(kHopperReturnCruiseSpeedMetersPerSecond, 2)
                / (2.0 * kHopperApproachBrakingAccelMetersPerSecondSq);
            assertTrue(slowDistance > cruiseStoppingDistance + 0.30,
                "Slow well before the cruise stopping distance, with room for alignment");
            double peakSpeed = 0.0;
            for (var state : PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow().getStates()) {
                double speed = Math.abs(state.linearVelocity);
                peakSpeed = Math.max(peakSpeed, speed);
                assertTrue(speed <= kHopperReturnCruiseSpeedMetersPerSecond + 1e-6, name);
                if (goalX - state.pose.getX() <= slowDistance - 0.01) {
                    assertTrue(speed <= kHopperApproachMaxSpeedMetersPerSecond + 1e-6,
                        name + " must be slow before the hub");
                }
            }
            assertTrue(peakSpeed > kHopperApproachMaxSpeedMetersPerSecond + 0.05,
                name + " should actually cruise faster outside the slow zone");
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
                        assertTrue(speed <= (side.equals("L") ? 1.21 : 0.46), name + " return trench speed=" + speed);
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
                    } else if (steps.get(i).get("data").path("name").asText().startsWith("resetBack")) {
                        assertEquals("named", fastSteps.get(i).get("type").asText());
                        assertEquals(steps.get(i).get("data").get("name").asText() + multiplier + "x",
                            fastSteps.get(i).get("data").get("name").asText());
                    } else {
                        assertEquals(steps.get(i), fastSteps.get(i));
                    }
                }
            }
        }
    }

    @Test
    void leftReturnsConnectAtTheEditedEndpointAndKeepBumpersClearWhileCurvingBack() throws Exception {
        RobotConfig config = RobotConfig.fromGUISettings();
        JsonNode baseReset = read("paths", "reset back L", ".path");
        JsonNode settings = json.readTree(deploy.resolve("settings.json").toFile());
        double halfLength = settings.get("robotLength").asDouble() / 2.0;
        double halfWidth = settings.get("robotWidth").asDouble() / 2.0;
        for (String suffix : new String[] {"", " 2x", " 3x"}) {
            String name = "reset back L" + suffix;
            JsonNode reset = read("paths", name, ".path");
            assertEquals(baseReset, reset, "All left returns use the same slow correction curve");
            assertTrue(reset.get("reversed").asBoolean());
            var returning = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
            var route = PathPlannerPath.fromPathFile("hopper front test L" + suffix)
                .getIdealTrajectory(config).orElseThrow();
            var routeEnd = route.getStates().get(route.getStates().size() - 1);
            var returnStart = returning.getStates().get(0).reverse();
            var returnEnd = returning.getStates().get(returning.getStates().size() - 1).reverse();
            assertEquals(routeEnd.pose.getX(), returnStart.pose.getX(), 1e-6);
            assertEquals(routeEnd.pose.getY(), returnStart.pose.getY(), 1e-6);
            assertEquals(0, routeEnd.pose.getRotation().minus(returnStart.pose.getRotation()).getDegrees(), 2.0);
            assertEquals(180, Math.abs(returnEnd.pose.getRotation().getDegrees()), 1e-6);
            assertEquals(0, routeEnd.linearVelocity, 1e-6);
            for (var state : returning.getStates()) {
                assertTrue(Double.isFinite(state.timeSeconds));
                assertTrue(Math.abs(state.linearVelocity) <= kHopperApproachMaxSpeedMetersPerSecond + 1e-6);
                double heading = state.pose.getRotation().getRadians();
                double bumperX = state.pose.getX() + halfLength * Math.abs(Math.cos(heading))
                    + halfWidth * Math.abs(Math.sin(heading));
                assertTrue(bumperX <= frc.robot.Constants.OdometryConstants.kHopperTagFaceXMeters - 0.12,
                    name + " bumper must clear the hub through the curved fallback");
            }
            boolean afterTrench = false;
            for (var state : route.getStates()) {
                if (state.pose.getX() < 3.25 && state.pose.getY() > 7.35) { afterTrench = true; }
                if (afterTrench) {
                    assertTrue(Math.abs(state.linearVelocity) <= 0.46,
                        "Faster variants must retain a slow final turn to limit drift");
                }
            }
            assertTrue(afterTrench);
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
