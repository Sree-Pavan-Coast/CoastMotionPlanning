const svg = document.getElementById("mapSvg");
const mapLayers = document.getElementById("mapLayers");
const emptyState = document.getElementById("emptyState");

const refs = {
  mapFileInput: document.getElementById("mapFileInput"),
  loadMapButton: document.getElementById("loadMapButton"),
  setStartToolButton: document.getElementById("setStartToolButton"),
  setGoalToolButton: document.getElementById("setGoalToolButton"),
  fitViewButton: document.getElementById("fitViewButton"),
  resetViewButton: document.getElementById("resetViewButton"),
  planButton: document.getElementById("planButton"),
  robotSelect: document.getElementById("robotSelect"),
  robotHelpMessage: document.getElementById("robotHelpMessage"),
  robotTypeLabel: document.getElementById("robotTypeLabel"),
  robotWheelbaseLabel: document.getElementById("robotWheelbaseLabel"),
  robotSteerLabel: document.getElementById("robotSteerLabel"),
  robotTurningRadiusLabel: document.getElementById("robotTurningRadiusLabel"),
  windowMapLabel: document.getElementById("windowMapLabel"),
  windowToolLabel: document.getElementById("windowToolLabel"),
  mapNameLabel: document.getElementById("mapNameLabel"),
  boundsLabel: document.getElementById("boundsLabel"),
  mapSourceLabel: document.getElementById("mapSourceLabel"),
  mapIdLabel: document.getElementById("mapIdLabel"),
  activeToolBadge: document.getElementById("activeToolBadge"),
  statusBadge: document.getElementById("statusBadge"),
  statusMessage: document.getElementById("statusMessage"),
  selectedProfileLabel: document.getElementById("selectedProfileLabel"),
  attemptedProfilesLabel: document.getElementById("attemptedProfilesLabel"),
  pathPoseCountLabel: document.getElementById("pathPoseCountLabel"),
  cursorWorldLabel: document.getElementById("cursorWorldLabel"),
  statusBarToolLabel: document.getElementById("statusBarToolLabel"),
  statusBarMapLabel: document.getElementById("statusBarMapLabel"),
  statusBarCursorLabel: document.getElementById("statusBarCursorLabel"),
  statusBarHintLabel: document.getElementById("statusBarHintLabel"),
  toggles: {
    grid: document.getElementById("showGridToggle"),
    searchSpace: document.getElementById("showSearchSpaceToggle"),
    zones: document.getElementById("showZonesToggle"),
    lanes: document.getElementById("showLanesToggle"),
    start: document.getElementById("showStartToggle"),
    goal: document.getElementById("showGoalToggle"),
    path: document.getElementById("showPathToggle"),
  },
  inputs: {
    start: {
      x: document.getElementById("startX"),
      y: document.getElementById("startY"),
      heading_deg: document.getElementById("startHeading"),
    },
    goal: {
      x: document.getElementById("goalX"),
      y: document.getElementById("goalY"),
      heading_deg: document.getElementById("goalHeading"),
    },
  },
};

const colors = {
  grid: "rgba(255,255,255,0.05)",
  gridStrong: "rgba(255,255,255,0.16)",
  searchSpaceFill: "rgba(120,220,160,0.07)",
  searchSpaceStroke: "rgba(120,220,160,0.55)",
  trackFill: "rgba(74,169,255,0.14)",
  trackStroke: "rgba(102,185,255,0.55)",
  maneuverFill: "rgba(240,165,58,0.14)",
  maneuverStroke: "rgba(240,165,58,0.58)",
  lane: "rgba(102,199,255,0.88)",
  forward: "#5dc9ff",
  reverse: "#f39a4a",
  start: "#52c0ff",
  startFill: "rgba(82,192,255,0.16)",
  goal: "#ffaf5b",
  goalFill: "rgba(255,175,91,0.16)",
  label: "rgba(221,228,236,0.78)",
};

function createDefaultPoses() {
  return {
    start: { x: null, y: null, heading_deg: 0 },
    goal: { x: null, y: null, heading_deg: 0 },
  };
}

function createEmptySearchSpacePreview() {
  return {
    success: false,
    detail: "",
    search_boundary: [],
  };
}

function formatDraftValue(key, value) {
  if (value == null) {
    return key === "heading_deg" ? "0" : "";
  }
  return String(value);
}

function createPoseDraftsFromPoses(poses) {
  return {
    start: {
      x: formatDraftValue("x", poses.start.x),
      y: formatDraftValue("y", poses.start.y),
      heading_deg: formatDraftValue("heading_deg", poses.start.heading_deg),
    },
    goal: {
      x: formatDraftValue("x", poses.goal.x),
      y: formatDraftValue("y", poses.goal.y),
      heading_deg: formatDraftValue("heading_deg", poses.goal.heading_deg),
    },
  };
}

const state = {
  robots: [],
  selectedRobotName: "",
  map: null,
  activeTool: "start",
  poses: createDefaultPoses(),
  poseDrafts: createPoseDraftsFromPoses(createDefaultPoses()),
  plan: null,
  searchSpacePreview: createEmptySearchSpacePreview(),
  viewBox: { x: -10, y: -10, width: 20, height: 20 },
  baseViewBox: { x: -10, y: -10, width: 20, height: 20 },
  cursorWorld: null,
  pointer: null,
};

let searchSpacePreviewTimer = null;
let searchSpacePreviewRequestId = 0;

function setStatus(kind, message) {
  refs.statusBadge.className = `status-badge ${kind}`;
  refs.statusBadge.textContent = kind === "idle"
    ? "Idle"
    : kind === "loading"
      ? "Planning"
      : kind === "success"
        ? "Success"
        : "Error";
  refs.statusMessage.textContent = message;
}

function formatMaybeNumber(value) {
  return value == null ? "-" : value.toFixed(2);
}

function formatRadians(value) {
  return value == null ? "-" : `${value.toFixed(3)} rad`;
}

function getSelectedRobot() {
  return state.robots.find((robot) => robot.name === state.selectedRobotName) || null;
}

function updateRobotSummary() {
  const robot = getSelectedRobot();
  refs.robotSelect.value = robot?.name || "";
  refs.robotTypeLabel.textContent = robot ? robot.type : "-";
  refs.robotWheelbaseLabel.textContent = robot?.planning_supported
    ? `${robot.vehicle.wheelbase_m.toFixed(3)} m`
    : "-";
  refs.robotSteerLabel.textContent = robot?.planning_supported
    ? formatRadians(robot.vehicle.max_steer_angle_rad)
    : "-";
  refs.robotTurningRadiusLabel.textContent = robot?.planning_supported
    ? `${robot.vehicle.min_turning_radius_m.toFixed(3)} m`
    : "-";
  refs.robotHelpMessage.textContent = robot?.planning_supported
    ? "Selected robot kinematics drive the motion primitive constraints at runtime."
    : "Select a supported Car robot before placing poses and planning.";
}

function renderRobotOptions() {
  refs.robotSelect.innerHTML = state.robots.map((robot) => {
    const suffix = robot.planning_supported ? "" : " (unsupported in visualizer)";
    return `<option value="${escapeHtml(robot.name)}"${robot.planning_supported ? "" : " disabled"}>
      ${escapeHtml(robot.name)}${suffix}
    </option>`;
  }).join("");
}

function setCursorLabel(point) {
  const label = point ? `${point.x.toFixed(2)}, ${point.y.toFixed(2)}` : "-";
  refs.cursorWorldLabel.textContent = label;
  refs.statusBarCursorLabel.textContent = `Cursor: ${label}`;
}

function updatePlannerSummary() {
  refs.selectedProfileLabel.textContent = state.plan?.selected_profile || "-";
  refs.attemptedProfilesLabel.textContent = state.plan?.attempted_profiles?.length
    ? state.plan.attempted_profiles.join(", ")
    : "-";
  refs.pathPoseCountLabel.textContent = String(state.plan?.path?.length || 0);
}

function updateMapSummary() {
  const mapName = state.map?.map_name || "No map loaded";
  refs.windowMapLabel.textContent = mapName;
  refs.mapNameLabel.textContent = mapName;
  refs.mapSourceLabel.textContent = state.map?.filename || "-";
  refs.mapIdLabel.textContent = state.map?.map_id || "-";
  refs.boundsLabel.textContent = state.map ? formatBounds(state.map.bounds) : "-";
  refs.statusBarMapLabel.textContent = `Map: ${state.map?.map_name || "none"}`;
}

function updateToolSummary() {
  const label = state.activeTool === "start" ? "Set Start" : "Set Goal";
  refs.windowToolLabel.textContent = `Tool: ${label}`;
  refs.activeToolBadge.textContent = label;
  refs.statusBarToolLabel.textContent = `Tool: ${label}`;
  refs.statusBarHintLabel.textContent =
    state.activeTool === "start"
      ? "Click in the viewport to place the start pose."
      : "Click in the viewport to place the goal pose.";
  refs.setStartToolButton.classList.toggle("is-active", state.activeTool === "start");
  refs.setGoalToolButton.classList.toggle("is-active", state.activeTool === "goal");
}

function writePoseInputs() {
  ["start", "goal"].forEach((poseName) => {
    refs.inputs[poseName].x.value = state.poseDrafts[poseName].x;
    refs.inputs[poseName].y.value = state.poseDrafts[poseName].y;
    refs.inputs[poseName].heading_deg.value = state.poseDrafts[poseName].heading_deg;
  });
}

function formatBounds(bounds) {
  return `x ${bounds.min_x.toFixed(1)}..${bounds.max_x.toFixed(1)} | y ${bounds.min_y.toFixed(1)}..${bounds.max_y.toFixed(1)}`;
}

function computeInitialViewBox(bounds) {
  const width = Math.max(bounds.max_x - bounds.min_x, 6);
  const height = Math.max(bounds.max_y - bounds.min_y, 6);
  const padding = Math.max(Math.max(width, height) * 0.1, 1.5);
  return {
    x: bounds.min_x - padding,
    y: -(bounds.max_y + padding),
    width: width + padding * 2,
    height: height + padding * 2,
  };
}

function clearPlan(reason) {
  if (!state.plan) {
    return;
  }
  state.plan = null;
  updatePlannerSummary();
  if (reason) {
    setStatus("idle", reason);
  }
}

function resetSearchSpacePreview() {
  state.searchSpacePreview = createEmptySearchSpacePreview();
}

function clearSearchSpacePreview() {
  if (searchSpacePreviewTimer) {
    clearTimeout(searchSpacePreviewTimer);
    searchSpacePreviewTimer = null;
  }
  searchSpacePreviewRequestId += 1;
  resetSearchSpacePreview();
}

function roundTo(value, digits = 2) {
  return Number.parseFloat(value.toFixed(digits));
}

function isIntermediateNumericInput(rawValue) {
  return rawValue === "-" ||
    rawValue === "+" ||
    rawValue === "." ||
    rawValue === "-." ||
    rawValue === "+.";
}

function toRadians(degrees) {
  return (degrees || 0) * Math.PI / 180;
}

function worldToSvg(point) {
  return { x: point.x, y: -point.y };
}

function getVehicleGeometry() {
  return state.map?.vehicle || null;
}

function getPoseFootprintWorldPoints(pose) {
  const vehicle = getVehicleGeometry();
  if (!vehicle || pose.x == null || pose.y == null) {
    return [];
  }

  const halfWidth = vehicle.width_m / 2;
  const minX = -vehicle.rear_overhang_m;
  const maxX = vehicle.wheelbase_m + vehicle.front_overhang_m;
  const heading = toRadians(pose.heading_deg);
  const cosHeading = Math.cos(heading);
  const sinHeading = Math.sin(heading);
  const localCorners = [
    { x: minX, y: -halfWidth },
    { x: maxX, y: -halfWidth },
    { x: maxX, y: halfWidth },
    { x: minX, y: halfWidth },
  ];

  return localCorners.map((corner) => ({
    x: pose.x + corner.x * cosHeading - corner.y * sinHeading,
    y: pose.y + corner.x * sinHeading + corner.y * cosHeading,
  }));
}

function getPoseBodyCenterWorld(pose) {
  const vehicle = getVehicleGeometry();
  if (!vehicle || pose.x == null || pose.y == null) {
    return pose;
  }

  const bodyCenterOffset = (vehicle.wheelbase_m + vehicle.front_overhang_m - vehicle.rear_overhang_m) / 2;
  const heading = toRadians(pose.heading_deg);
  return {
    x: pose.x + Math.cos(heading) * bodyCenterOffset,
    y: pose.y + Math.sin(heading) * bodyCenterOffset,
  };
}

function syncPoseDraft(poseName, key) {
  state.poseDrafts[poseName][key] = formatDraftValue(key, state.poses[poseName][key]);
}

function updatePoseField(poseName, key, rawValue) {
  state.poseDrafts[poseName][key] = rawValue;

  if (rawValue === "") {
    state.poses[poseName][key] = key === "heading_deg" ? 0 : null;
    clearPlan("Scenario changed. Click Plan Path to recompute.");
    scheduleSearchSpacePreview();
    render();
    return;
  }

  if (isIntermediateNumericInput(rawValue)) {
    clearPlan("Scenario changed. Click Plan Path to recompute.");
    scheduleSearchSpacePreview();
    render();
    return;
  }

  const numericValue = Number(rawValue);
  if (!Number.isFinite(numericValue)) {
    clearSearchSpacePreview();
    render();
    return;
  }

  state.poses[poseName][key] = numericValue;
  clearPlan("Scenario changed. Click Plan Path to recompute.");
  scheduleSearchSpacePreview();
  render();
}

function parsePoseDraftValue(poseName, key) {
  const rawValue = state.poseDrafts[poseName][key].trim();
  if (rawValue === "") {
    if (key === "heading_deg") {
      return 0;
    }
    throw new Error(`${poseName === "start" ? "Start" : "Goal"} pose requires ${key}.`);
  }

  if (isIntermediateNumericInput(rawValue)) {
    throw new Error(`${poseName === "start" ? "Start" : "Goal"} ${key} is incomplete.`);
  }

  const numericValue = Number(rawValue);
  if (!Number.isFinite(numericValue)) {
    throw new Error(`${poseName === "start" ? "Start" : "Goal"} ${key} must be numeric.`);
  }

  return numericValue;
}

function setActiveTool(tool) {
  state.activeTool = tool;
  updateToolSummary();
}

function chooseGridStep(width, height) {
  const targetLines = 12;
  const raw = Math.max(width, height) / targetLines;
  const exponent = Math.floor(Math.log10(raw || 1));
  const base = 10 ** exponent;
  const normalized = raw / base;

  if (normalized < 1.5) {
    return base;
  }
  if (normalized < 3) {
    return base * 2;
  }
  if (normalized < 7) {
    return base * 5;
  }
  return base * 10;
}

function renderGrid() {
  if (!refs.toggles.grid.checked) {
    return "";
  }

  const step = chooseGridStep(state.viewBox.width, state.viewBox.height);
  const minX = Math.floor(state.viewBox.x / step) * step;
  const maxX = Math.ceil((state.viewBox.x + state.viewBox.width) / step) * step;
  const minWorldY = -state.viewBox.y - state.viewBox.height;
  const maxWorldY = -state.viewBox.y;
  const minY = Math.floor(minWorldY / step) * step;
  const maxY = Math.ceil(maxWorldY / step) * step;

  const lines = [];
  for (let x = minX; x <= maxX; x += step) {
    const isAxis = Math.abs(x) < step * 0.1;
    lines.push(
      `<line x1="${x}" y1="${state.viewBox.y}" x2="${x}" y2="${state.viewBox.y + state.viewBox.height}"
        stroke="${isAxis ? colors.gridStrong : colors.grid}" stroke-width="${isAxis ? 0.12 : 0.05}"
        vector-effect="non-scaling-stroke" />`
    );
  }

  for (let worldY = minY; worldY <= maxY; worldY += step) {
    const svgY = -worldY;
    const isAxis = Math.abs(worldY) < step * 0.1;
    lines.push(
      `<line x1="${state.viewBox.x}" y1="${svgY}" x2="${state.viewBox.x + state.viewBox.width}" y2="${svgY}"
        stroke="${isAxis ? colors.gridStrong : colors.grid}" stroke-width="${isAxis ? 0.12 : 0.05}"
        vector-effect="non-scaling-stroke" />`
    );
  }

  return lines.join("");
}

function escapeHtml(value) {
  return value
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function tryBuildPreviewPose(poseName) {
  try {
    return {
      x: parsePoseDraftValue(poseName, "x"),
      y: parsePoseDraftValue(poseName, "y"),
      heading_deg: parsePoseDraftValue(poseName, "heading_deg"),
    };
  } catch (error) {
    return null;
  }
}

function buildSearchSpacePreviewRequest() {
  if (!state.map) {
    return null;
  }

  const start = tryBuildPreviewPose("start");
  const goal = tryBuildPreviewPose("goal");
  if (!start || !goal) {
    return null;
  }

  return {
    map_id: state.map.map_id,
    start,
    goal,
  };
}

function scheduleSearchSpacePreview() {
  const request = buildSearchSpacePreviewRequest();
  if (!request) {
    clearSearchSpacePreview();
    return;
  }

  if (searchSpacePreviewTimer) {
    clearTimeout(searchSpacePreviewTimer);
  }

  const requestId = ++searchSpacePreviewRequestId;
  searchSpacePreviewTimer = window.setTimeout(async () => {
    searchSpacePreviewTimer = null;
    try {
      const response = await fetch("/api/search-space/preview", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(request),
      });
      const body = await response.json();
      if (requestId !== searchSpacePreviewRequestId || request.map_id !== state.map?.map_id) {
        return;
      }
      if (!response.ok || !body.success) {
        resetSearchSpacePreview();
        render();
        return;
      }

      state.searchSpacePreview = {
        success: true,
        detail: body.detail || "",
        search_boundary: body.search_boundary || [],
      };
      render();
    } catch (error) {
      if (requestId !== searchSpacePreviewRequestId) {
        return;
      }
      resetSearchSpacePreview();
      render();
    }
  }, 180);
}

function renderSearchSpace() {
  if (!state.searchSpacePreview.search_boundary.length || !refs.toggles.searchSpace.checked) {
    return "";
  }

  const points = state.searchSpacePreview.search_boundary
    .map((point) => `${point.x},${-point.y}`)
    .join(" ");
  return `
    <g class="search-space-layer">
      <polygon points="${points}" fill="${colors.searchSpaceFill}" stroke="${colors.searchSpaceStroke}"
        stroke-width="0.32" stroke-dasharray="1.2 0.6" vector-effect="non-scaling-stroke" />
    </g>
  `;
}

function renderZones() {
  if (!state.map || !refs.toggles.zones.checked) {
    return "";
  }

  return state.map.zones.map((zone) => {
    const fill = zone.type === "TrackMainRoad" ? colors.trackFill : colors.maneuverFill;
    const stroke = zone.type === "TrackMainRoad" ? colors.trackStroke : colors.maneuverStroke;
    const points = zone.polygon.map((point) => `${point.x},${-point.y}`).join(" ");
    const centroid = zone.polygon.reduce((acc, point) => ({
      x: acc.x + point.x / zone.polygon.length,
      y: acc.y + point.y / zone.polygon.length,
    }), { x: 0, y: 0 });

    return `
      <g class="zone-layer">
        <polygon points="${points}" fill="${fill}" stroke="${stroke}" stroke-width="0.18" vector-effect="non-scaling-stroke" />
        <text x="${centroid.x}" y="${-centroid.y}" fill="${colors.label}" font-size="0.78" text-anchor="middle">${escapeHtml(zone.name)}</text>
      </g>
    `;
  }).join("");
}

function renderLanes() {
  if (!state.map || !refs.toggles.lanes.checked) {
    return "";
  }

  const lines = [];
  state.map.zones.forEach((zone) => {
    zone.lanes.forEach((lane) => {
      const points = lane.poses.map((pose) => `${pose.x},${-pose.y}`).join(" ");
      lines.push(
        `<polyline points="${points}" fill="none" stroke="${colors.lane}" stroke-width="0.22"
          stroke-dasharray="0.46 0.28" vector-effect="non-scaling-stroke" />`
      );
    });
  });
  return lines.join("");
}

function renderPath() {
  if (!state.plan?.path?.length || !refs.toggles.path.checked) {
    return "";
  }

  const poses = state.plan.path;
  const backdrop = `<polyline points="${poses.map((pose) => `${pose.x},${-pose.y}`).join(" ")}"
    fill="none" stroke="rgba(255,255,255,0.1)" stroke-width="0.52" stroke-linecap="round"
    stroke-linejoin="round" vector-effect="non-scaling-stroke" />`;

  const segments = [];
  for (let index = 0; index < poses.length - 1; index += 1) {
    const start = poses[index];
    const end = poses[index + 1];
    const direction = state.plan.segment_directions?.[index] || "Forward";
    segments.push(
      `<line x1="${start.x}" y1="${-start.y}" x2="${end.x}" y2="${-end.y}"
        stroke="${direction === "Reverse" ? colors.reverse : colors.forward}" stroke-width="0.32"
        stroke-linecap="round" vector-effect="non-scaling-stroke" />`
    );
  }

  return `<g class="path-layer">${backdrop}${segments.join("")}</g>`;
}

function renderPoseMarker(poseName, pose) {
  const toggle = poseName === "start" ? refs.toggles.start : refs.toggles.goal;
  if (!toggle.checked || pose.x == null || pose.y == null) {
    return "";
  }

  const color = poseName === "start" ? colors.start : colors.goal;
  const fill = poseName === "start" ? colors.startFill : colors.goalFill;
  const label = poseName === "start" ? "S" : "G";
  const vehicle = getVehicleGeometry();
  const radians = toRadians(pose.heading_deg);
  const bodyCenter = getPoseBodyCenterWorld(pose);
  const footprintPoints = getPoseFootprintWorldPoints(pose)
    .map((point) => {
      const svgPoint = worldToSvg(point);
      return `${svgPoint.x},${svgPoint.y}`;
    })
    .join(" ");
  const arrowLength = vehicle
    ? vehicle.wheelbase_m + vehicle.front_overhang_m + 0.35
    : 1.4;
  const tip = {
    x: pose.x + Math.cos(radians) * arrowLength,
    y: pose.y + Math.sin(radians) * arrowLength,
  };
  const wingAngle = Math.PI / 7;
  const wingLength = vehicle ? Math.max(vehicle.width_m * 0.28, 0.36) : 0.42;
  const left = {
    x: tip.x - Math.cos(radians - wingAngle) * wingLength,
    y: tip.y - Math.sin(radians - wingAngle) * wingLength,
  };
  const right = {
    x: tip.x - Math.cos(radians + wingAngle) * wingLength,
    y: tip.y - Math.sin(radians + wingAngle) * wingLength,
  };

  return `
    <g class="pose-marker ${poseName}">
      ${footprintPoints
        ? `<polygon points="${footprintPoints}" fill="${fill}" stroke="${color}" stroke-width="0.18"
          vector-effect="non-scaling-stroke" />`
        : ""}
      <line x1="${pose.x}" y1="${-pose.y}" x2="${tip.x}" y2="${-tip.y}"
        stroke="${color}" stroke-width="0.28" vector-effect="non-scaling-stroke" />
      <polyline points="${left.x},${-left.y} ${tip.x},${-tip.y} ${right.x},${-right.y}"
        fill="none" stroke="${color}" stroke-width="0.28" vector-effect="non-scaling-stroke" />
      <circle cx="${pose.x}" cy="${-pose.y}" r="0.56" fill="#11161c" stroke="${color}" stroke-width="0.22"
        vector-effect="non-scaling-stroke" />
      <text x="${bodyCenter.x}" y="${-bodyCenter.y + 0.06}" fill="${color}" text-anchor="middle" font-size="0.72">${label}</text>
    </g>
  `;
}

function render() {
  svg.setAttribute("viewBox", `${state.viewBox.x} ${state.viewBox.y} ${state.viewBox.width} ${state.viewBox.height}`);
  updatePlannerSummary();
  updateMapSummary();
  updateRobotSummary();
  updateToolSummary();
  writePoseInputs();
  setCursorLabel(state.cursorWorld);

  if (!state.map) {
    emptyState.hidden = false;
    mapLayers.innerHTML = "";
    return;
  }

  emptyState.hidden = true;
  mapLayers.innerHTML = [
    `<rect x="${state.viewBox.x}" y="${state.viewBox.y}" width="${state.viewBox.width}" height="${state.viewBox.height}" fill="rgba(255,255,255,0.01)" />`,
    renderGrid(),
    renderSearchSpace(),
    renderZones(),
    renderLanes(),
    renderPath(),
    renderPoseMarker("start", state.poses.start),
    renderPoseMarker("goal", state.poses.goal),
  ].join("");
}

function eventToWorld(event) {
  const ctm = svg.getScreenCTM();
  if (!ctm) {
    return null;
  }

  const point = svg.createSVGPoint();
  point.x = event.clientX;
  point.y = event.clientY;
  const svgPoint = point.matrixTransform(ctm.inverse());
  return { x: svgPoint.x, y: -svgPoint.y };
}

function fitView() {
  if (!state.map) {
    return;
  }
  state.viewBox = { ...state.baseViewBox };
  render();
}

function clearScenarioForNewMap() {
  state.plan = null;
  state.poses = createDefaultPoses();
  state.poseDrafts = createPoseDraftsFromPoses(state.poses);
  state.cursorWorld = null;
  clearSearchSpacePreview();
}

function syncSelectedRobotToMap() {
  const robot = getSelectedRobot();
  if (!robot?.planning_supported || !state.map) {
    return;
  }
  state.map = {
    ...state.map,
    vehicle: robot.vehicle,
  };
}

function setSelectedRobot(robotName, reason = "Robot changed. Click Plan Path to recompute.") {
  state.selectedRobotName = robotName;
  syncSelectedRobotToMap();
  clearPlan(reason);
  setStatus("idle", reason);
  render();
}

function validateSelectedRobot() {
  const robot = getSelectedRobot();
  if (!robot) {
    throw new Error("Select a robot before planning.");
  }
  if (!robot.planning_supported) {
    throw new Error(`Robot ${robot.name} is not supported by the visualizer yet.`);
  }
  return robot;
}

async function loadRobots() {
  setStatus("loading", "Loading robot catalog...");
  try {
    const response = await fetch("/api/robots");
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Failed to load robot catalog.");
    }

    state.robots = body.robots || [];
    renderRobotOptions();

    const preferredRobot = state.robots.find((robot) => robot.name === body.default_robot_name && robot.planning_supported)
      || state.robots.find((robot) => robot.planning_supported);
    if (!preferredRobot) {
      throw new Error("No supported Car robots were found in robots.yaml.");
    }

    state.selectedRobotName = preferredRobot.name;
    refs.robotSelect.value = preferredRobot.name;
    setStatus("idle", "Select a robot and load a map to begin.");
    render();
  } catch (error) {
    setStatus("error", error.message);
    render();
  }
}

async function loadSelectedMap(file) {
  if (!file) {
    setStatus("error", "Select a YAML file to load.");
    return;
  }

  setStatus("loading", `Loading ${file.name}...`);
  try {
    const robot = validateSelectedRobot();
    const yaml = await file.text();
    const response = await fetch("/api/map/load", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ filename: file.name, yaml, robot_name: robot.name }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Map loading failed.");
    }

    state.map = body;
    clearScenarioForNewMap();
    state.baseViewBox = computeInitialViewBox(body.bounds);
    state.viewBox = { ...state.baseViewBox };
    setActiveTool("start");
    setStatus("idle", `Map loaded for ${robot.name}. Use the Set Start and Set Goal tools to place poses.`);
    render();
  } catch (error) {
    state.map = null;
    clearSearchSpacePreview();
    setStatus("error", error.message);
    render();
  }
}

function validatePoseForPlanning(poseName) {
  const x = parsePoseDraftValue(poseName, "x");
  const y = parsePoseDraftValue(poseName, "y");
  const headingDeg = parsePoseDraftValue(poseName, "heading_deg");

  state.poses[poseName].x = x;
  state.poses[poseName].y = y;
  state.poses[poseName].heading_deg = headingDeg;

  return {
    x,
    y,
    heading_deg: headingDeg,
  };
}

async function planPath() {
  if (!state.map) {
    setStatus("error", "Load a map before planning.");
    return;
  }

  try {
    const robot = validateSelectedRobot();
    const request = {
      map_id: state.map.map_id,
      robot_name: robot.name,
      start: validatePoseForPlanning("start"),
      goal: validatePoseForPlanning("goal"),
    };

    setStatus("loading", "Planning path...");
    const response = await fetch("/api/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Planning request failed.");
    }

    state.plan = body;
    updatePlannerSummary();
    setStatus(body.success ? "success" : "error", body.detail || (body.success ? "Path found." : "Planner failed."));
    render();
  } catch (error) {
    state.plan = null;
    updatePlannerSummary();
    setStatus("error", error.message);
    render();
  }
}

refs.loadMapButton.addEventListener("click", () => {
  refs.mapFileInput.click();
});

refs.mapFileInput.addEventListener("change", () => {
  const [file] = refs.mapFileInput.files;
  if (file) {
    loadSelectedMap(file);
  }
});

refs.robotSelect.addEventListener("change", (event) => {
  const nextRobotName = event.target.value;
  const nextRobot = state.robots.find((robot) => robot.name === nextRobotName);
  if (!nextRobot) {
    return;
  }

  const reason = state.map
    ? `Robot changed to ${nextRobot.name}. Existing plan cleared; poses kept.`
    : "Robot updated. Load a map to begin.";
  setSelectedRobot(nextRobot.name, reason);
});

refs.setStartToolButton.addEventListener("click", () => setActiveTool("start"));
refs.setGoalToolButton.addEventListener("click", () => setActiveTool("goal"));
refs.fitViewButton.addEventListener("click", fitView);
refs.resetViewButton.addEventListener("click", fitView);
refs.planButton.addEventListener("click", planPath);

Object.values(refs.toggles).forEach((toggle) => {
  toggle.addEventListener("change", render);
});

["start", "goal"].forEach((poseName) => {
  Object.entries(refs.inputs[poseName]).forEach(([key, input]) => {
    input.addEventListener("input", (event) => {
      updatePoseField(poseName, key, event.target.value);
    });
  });
});

svg.addEventListener("wheel", (event) => {
  if (!state.map) {
    return;
  }
  event.preventDefault();

  const worldPoint = eventToWorld(event);
  if (!worldPoint) {
    return;
  }

  const zoomFactor = event.deltaY < 0 ? 0.88 : 1.12;
  const nextWidth = Math.max(state.viewBox.width * zoomFactor, 2);
  const nextHeight = Math.max(state.viewBox.height * zoomFactor, 2);
  const cursorSvgX = worldPoint.x;
  const cursorSvgY = -worldPoint.y;
  const offsetX = (cursorSvgX - state.viewBox.x) / state.viewBox.width;
  const offsetY = (cursorSvgY - state.viewBox.y) / state.viewBox.height;

  state.viewBox = {
    x: cursorSvgX - offsetX * nextWidth,
    y: cursorSvgY - offsetY * nextHeight,
    width: nextWidth,
    height: nextHeight,
  };
  render();
}, { passive: false });

svg.addEventListener("pointerdown", (event) => {
  if (!state.map || event.button !== 0) {
    return;
  }

  const anchorWorld = eventToWorld(event);
  if (!anchorWorld) {
    return;
  }

  state.pointer = {
    id: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    anchorWorld,
    dragging: false,
  };
  svg.setPointerCapture(event.pointerId);
});

svg.addEventListener("pointermove", (event) => {
  const worldPoint = eventToWorld(event);
  state.cursorWorld = worldPoint;
  setCursorLabel(worldPoint);

  if (!state.pointer || event.pointerId !== state.pointer.id) {
    return;
  }

  const deltaX = event.clientX - state.pointer.startX;
  const deltaY = event.clientY - state.pointer.startY;
  if (!state.pointer.dragging && Math.hypot(deltaX, deltaY) > 4) {
    state.pointer.dragging = true;
  }
  if (!state.pointer.dragging) {
    return;
  }

  const anchorWorld = state.pointer.anchorWorld;
  if (!worldPoint || !anchorWorld) {
    return;
  }

  const worldDeltaX = anchorWorld.x - worldPoint.x;
  const worldDeltaY = anchorWorld.y - worldPoint.y;
  state.viewBox = {
    x: state.viewBox.x + worldDeltaX,
    y: state.viewBox.y - worldDeltaY,
    width: state.viewBox.width,
    height: state.viewBox.height,
  };
  state.cursorWorld = anchorWorld;
  render();
});

svg.addEventListener("pointerleave", () => {
  state.cursorWorld = null;
  setCursorLabel(null);
});

svg.addEventListener("pointerup", (event) => {
  if (!state.pointer || event.pointerId !== state.pointer.id) {
    return;
  }

  const pointerState = state.pointer;
  state.pointer = null;
  svg.releasePointerCapture(event.pointerId);

  if (pointerState.dragging || !state.map) {
    return;
  }

  const worldPoint = eventToWorld(event);
  if (!worldPoint) {
    return;
  }

  state.poses[state.activeTool].x = roundTo(worldPoint.x);
  state.poses[state.activeTool].y = roundTo(worldPoint.y);
  syncPoseDraft(state.activeTool, "x");
  syncPoseDraft(state.activeTool, "y");
  clearPlan("Scenario changed. Click Plan Path to recompute.");
  scheduleSearchSpacePreview();
  render();
});

svg.addEventListener("pointercancel", () => {
  state.pointer = null;
});

updateMapSummary();
updateToolSummary();
updatePlannerSummary();
writePoseInputs();
setCursorLabel(null);
setStatus("idle", "Loading robot catalog...");
loadRobots();
render();
