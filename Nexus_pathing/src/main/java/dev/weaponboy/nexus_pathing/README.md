# Nexus Pathing

A path generation and following library for FTC robots. This document covers the public-facing API: building paths with `PathsManager`, and following them with `Follower`.

---

## Quick Start

```java
// 1. Set up a path
PathsManager pathsManager = new PathsManager();
pathsManager.addNewPath("scorePath");

SectionBuilder[] sections = new SectionBuilder[]{
    () -> pathsManager.addPoints(new Vector2D(0, 0), new Vector2D(24, 24)),
    () -> pathsManager.addPoints(new Vector2D(24, 24), new Vector2D(30, 10), new Vector2D(48, 48))
};
pathsManager.buildPath(sections, "scorePath");

// 2. Follow it
Follower follower = new Follower();
follower.setPath(pathsManager.returnCurrentPath());

// 3. In your loop
RobotPower power = follower.followPathAuto(targetHeading, currentHeading, x, y, xVelocity, yVelocity);
drive(power.getVertical(), power.getHorizontal(), power.getPivot());
```

---

## PathsManager

Handles creating, building, and storing multiple named paths.

### About `SectionBuilder`

`SectionBuilder` is a functional interface with a single method, `buildSection()`. In practice, a `SectionBuilder[]` is just a list of lambdas, each one calling `addPoints(...)` to add a straight line, quadratic curve, or cubic curve segment to the path being built:

```java
SectionBuilder[] sections = new SectionBuilder[]{
    () -> pathsManager.addPoints(new Vector2D(0, 0), new Vector2D(24, 24)),
    () -> pathsManager.addPoints(new Vector2D(24, 24), new Vector2D(48, 0))
};
```

Passing this array into `buildPath(...)` runs each lambda in order (adding all the segments), then smooths the combined path, generates the motion profile, and calculates headings.

### Constructors

| Constructor | Description |
|---|---|
| `PathsManager()` | Creates a manager using the default `RobotConfig`. |
| `PathsManager(RobotConfig customConfig)` | Creates a manager using a custom `RobotConfig`. |

### Methods

**`addNewPath(String pathName)`**
Registers a new, empty path under `pathName` and makes it the current path.

**`addNewPath(String pathName, SectionBuilder[] pathSections)`**
Registers a new path under `pathName` and immediately builds it from `pathSections`.

**`buildPath(SectionBuilder[] pathSections, String pathName)`**
Builds an already-registered path (found by name) from the given sections.

**`buildPath(SectionBuilder[] pathSections, String pathName, double newAccelMax)`**
Same as above, but overrides the max acceleration used for the motion profile.

**`buildPath(SectionBuilder[] pathSections)`**
Builds the *current* path from the given sections.

**`buildPath(SectionBuilder[] pathSections, double newAccelMax)`**
Builds the current path from the given sections, with a custom max acceleration.

**`addPoints(Vector2D start, Vector2D end)`**
Adds a straight-line segment to the current path.

**`addPoints(Vector2D start, Vector2D control, Vector2D end)`**
Adds a quadratic Bézier curve segment to the current path.

**`addPoints(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end)`**
Adds a cubic Bézier curve segment to the current path.

**`getCurrentPath()`**
Returns the name of the currently selected path.

**`setCurrentPath(String currentPath)`**
Switches the active path to an already-registered path by name. Does nothing if the name doesn't exist.

**`returnPathingPoints()`** / **`returnPathingPoints(String pathName)`**
Returns the list of `Vector2D` points that make up the followable path (current path, or the named one — the named version also switches the current path).

**`returnVectorField()`** / **`returnVectorField(String pathName)`**
Returns the list of `PathingVelocity` values (the motion profile) for the path.

**`returnPath(String pathName)`**
Returns the raw `PathBuilder` object for a named path.

**`returnCurrentPath()`**
Returns the raw `PathBuilder` object for the current path. This is what you pass into `Follower.setPath(...)`.

---

## Follower

Drives the robot along a path built by `PathsManager`, using PID correction. Call `followPathAuto(...)` once per loop iteration.

### Constructors

| Constructor | Description |
|---|---|
| `Follower()` | Creates a follower using the default `RobotConfig`. |
| `Follower(RobotConfig customConfig)` | Creates a follower using a custom `RobotConfig`. |

### Setup / Setting a Path

**`setPath(PathBuilder path)`**
Loads a new path to follow (get this from `pathsManager.returnCurrentPath()` or `returnPath(name)`). Resets the finished/stopped state.

### Main Loop Method

**`followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV)`**
Call this every loop cycle. Computes drive power to follow the current path.
- `targetHeading` — heading to hold if `usePathHeadings` is disabled.
- `H` — robot's current heading (degrees).
- `X`, `Y` — robot's current field position.
- `XV`, `YV` — robot's current field velocity.
- Returns a `RobotPower` with the motor outputs to apply: `getVertical()` (forward/back), `getHorizontal()` (strafe), and `getPivot()` (turn).

### Standalone Helpers

**`getTurnPower(double targetHeading, double currentHeading, double Xvelo, double Yvelo)`**
Computes turning power to rotate toward `targetHeading`. Can be used outside of `followPathAuto` if you just need heading correction.

**`pidToPoint(Vector2D robotPos, Vector2D targetPos, double heading, double XVelocity, double YVelocity)`**
Runs a PID correction directly toward a single target point (bypasses path following). Returns a `PathingPower`.

### Toggles / Configuration

| Method | Effect |
|---|---|
| `robotHeadingBackwards(boolean)` | Flips the robot's effective heading by 180° (for driving "backwards" along a path). |
| `disableGlobalFollowing(boolean)` | Disables field-relative correction, using robot-relative error instead. |
| `setHeadingOffset(double)` | Adds a constant offset (degrees) to the path-based target heading. |
| `setHeadingLookAheadDistance(int)` | Sets how many points ahead on the path to sample for the target heading. |
| `slowerHeading(boolean)` | Switches between the fast and slow heading PID controllers. |
| `usePathHeadings(boolean)` | If `true`, heading is driven by the path's recorded headings instead of the `targetHeading` argument. |
| `holdPositionAtPathEnd(boolean)` | If `true`, the robot actively holds its position once the path is finished. |
| `isHoldPosition()` | Returns whether hold-position-at-end is enabled. |
| `headingErrorToOvercomeFriction(boolean)` | Enables/disables a small ramping heading error added to overcome static friction near the target heading. |

### Path Progress & Error Checking

**`resetClosestPoint(Vector2D robotPos)`**
Forces a full-path search to re-find the closest point on the path to `robotPos`. Useful if the robot's position jumps unexpectedly.

**`createNewPathOperator(PathBuilder path)`**
Lower-level version of `setPath(...)` — builds the internal path-tracking object without resetting the finished/stopped flags. Prefer `setPath(...)` in normal use.

**`clipLookAheadIndex(int index)`**
Clamps `index` to a valid index into the path's heading list, and stores it for use by `calculateTargetHeading()`.

**`calculateTargetHeading()`**
Returns the target heading (degrees) at the current look-ahead index, plus any configured offsets.

**`finishPath()`**
Manually marks the current path as finished and locks in the current position as the hold point.

**`isFinished()`**
Returns `true` if the robot is within 1 unit of the path's end point (in both X and Y), or if the path has already been marked finished.

**`isFinished(double xTolerance, double yTolerance)`**
Same as above, but with custom tolerances. Also marks the path as finished the first time the tolerance is met.

**`getErrorToEnd()`**
Returns the straight-line distance from the robot's current position to the path's end point.

**`getXError()`** / **`getYError()`**
Returns the absolute X / Y distance from the robot to the path's end point.

**`getErrorToPointOnPath(double currentX, double currentY)`**
Returns the `Vector2D` error (end point minus given position).

**`getErrorToPath()`**
Finds the robot's closest point on the path and returns the `Vector2D` error between the robot and that point. Also updates internal tracking state used elsewhere by the follower.

---

## RobotConfig

Holds the PID and physical-limit constants used by both `PathsManager` (for motion profiling) and `Follower` (for path correction). Optional — both classes work with a default `RobotConfig` if you don't supply one.

### Constructors

| Constructor | Description |
|---|---|
| `RobotConfig()` | Uses built-in default constants — customize afterward with the builder-style setters below. |

### Builder-Style Setters

These return `this`, so they can be chained after the constructor:

| Method | Sets |
|---|---|
| `setXLastAdjustmentPD(p, d)` | P/D gains for the final X correction at the end of a path. |
| `setYLastAdjustmentPD(p, d)` | P/D gains for the final Y correction at the end of a path. |
| `setXOnPathPD(p, d)` | P/D gains for X correction while following a path. |
| `setYOnPathPD(p, d)` | P/D gains for Y correction while following a path. |
| `setFastHeadingPD(p, d)` | P/D gains for the "fast" heading PID (used unless `slowerHeading(true)`). |
| `setSlowHeadingPD(p, d)` | P/D gains for the "slow" heading PID. |
| `setRobotConstants(maxXVelocity, maxYVelocity, maxXAcceleration, maxYAcceleration)` | The robot's max velocity/acceleration in X and Y, used for motion profiling. |
| `logDebugging(boolean)` | Turns on/off console logging during path building. |

Example:

```java
RobotConfig config = new RobotConfig()
    .setRobotConstants(163, 120, 280, 90)
    .setXOnPathPD(0.06, 0.005)
    .setYOnPathPD(0.075, 0.005);

PathsManager pathsManager = new PathsManager(config);
Follower follower = new Follower(config);
```

> Use the **same** `RobotConfig` instance (or at least matching values) for both `PathsManager` and `Follower` — the motion profile `PathsManager` generates assumes the same velocity/acceleration limits that `Follower` will be correcting against.

---

## Typical Usage Pattern

1. Create one `PathsManager` (optionally with a custom `RobotConfig`) and build all your paths ahead of time with `addNewPath` + `addPoints`/`buildPath`.
2. Create one `Follower` (optionally with the same `RobotConfig`).
3. When you want the robot to run a path: `follower.setPath(pathsManager.returnPath("pathName"))`.
4. Every loop cycle, call `follower.followPathAuto(...)` and apply the returned power to your drivetrain.
5. Use `follower.isFinished()` to know when to move on to the next path or action.
