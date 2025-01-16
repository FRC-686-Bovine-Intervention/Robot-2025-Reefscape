import { displaySelectedLevel } from "./levelSelector.js";
import { displaySelectedBranch } from "./branchSelector.js";
import { displayTime } from "./timer.js";
import { displayErrors, displayInfos, displayWarnings } from "./alerts.js";

import { NT4_Client } from "./NT4.js";

const nodeRobotToDashboardTopic = "/node_selector/node_robot_to_dashboard";
const nodeDashboardToRobotTopic = "/node_selector/node_dashboard_to_robot";
const matchTimeAdvantagekitToDashboardTopic =
  "/AdvantageKit/DriverStation/MatchTime";
const autonomousAdvantagekitToDashboardTopic =
  "/AdvantageKit/DriverStation/Autonomous";
const alertsSmartdashboardToDashboardTopic = {
  warnings: "/SmartDashboard/Alerts/warnings",
  infos: "/SmartDashboard/Alerts/infos",
  errors: "/SmartDashboard/Alerts/errors",
};

let selectedNode = [-1, -1, -1];
let isAuto = false;
let matchTime = 0;

let client = new NT4_Client(
  window.location.hostname,
  "NodeSelector",
  (topic) => {}, // Topic Announce
  () => {}, // Topic Unannounce
  (topic, timestamp, value) => {
    if (topic.name === nodeRobotToDashboardTopic) {
      const rack = value[0];
      const level = value[1];
      const side = value[2];
      displaySelectedBranch(rack, side);
      displaySelectedLevel(level);

      selectedNode = value;
    } else if (topic.name === matchTimeAdvantagekitToDashboardTopic) {
      matchTime = Math.max(0, value);
      displayTime(matchTime, isAuto);
    } else if (topic.name === autonomousAdvantagekitToDashboardTopic) {
      isAuto = value;
      displayTime(matchTime, isAuto);
    } else if (topic.name === alertsSmartdashboardToDashboardTopic.infos) {
      displayInfos(value);
    } else if (topic.name === alertsSmartdashboardToDashboardTopic.warnings) {
      displayWarnings(value);
    } else if (topic.name === alertsSmartdashboardToDashboardTopic.errors) {
      displayErrors(value);
    }
  }, // New data
  () => {
    const overlay = document.getElementById("overlay");
    if (overlay) overlay.remove();
  }, // Connect
  () => {
    displaySelectedBranch();
    displaySelectedLevel();
    displayTime(0, false);
    displayInfos([]);
    displayWarnings([]);
    displayErrors([]);
    if (!document.getElementById("overlay")) {
      const overlay = document.createElement("div");
      overlay.id = "overlay";
      document.body.appendChild(overlay);
    }
  } // Disconnect
);

window.onload = () => {
  client.subscribe(
    [
      nodeRobotToDashboardTopic,
      matchTimeAdvantagekitToDashboardTopic,
      autonomousAdvantagekitToDashboardTopic,
      alertsSmartdashboardToDashboardTopic.infos,
      alertsSmartdashboardToDashboardTopic.warnings,
      alertsSmartdashboardToDashboardTopic.errors,
    ],
    false,
    false,
    0.02
  );
  client.publishTopic(nodeDashboardToRobotTopic, "int[]");
  client.connect();
};

export function sendSelectedBranch(rack, side) {
  if (selectedNode[0] !== rack || selectedNode[2] !== side) {
    client.addSample(nodeDashboardToRobotTopic, [rack, selectedNode[1], side]);
  }
}

export function sendSelectedLevel(level) {
  if (selectedNode[1] !== level) {
    client.addSample(nodeDashboardToRobotTopic, [
      selectedNode[0],
      level,
      selectedNode[2],
    ]);
  }
}
