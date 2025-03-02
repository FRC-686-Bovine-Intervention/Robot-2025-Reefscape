import { displaySelectedLevel } from "./levelSelector.js";
import {
  createBranchSelector,
  displaySelectedBranch,
  displaySelectedRack,
} from "./branchSelector.js";
import { displayTime } from "./timer.js";
import { displayErrors, displayInfos, displayWarnings } from "./alerts.js";
import "./keyboardShortcuts.js";

import { NT4_Client } from "./NT4.js";
import { getCoral, getCoralIdx } from "./utils.js";
import { displaySelectedAlgaeScoring } from "./algaeScoringSelector.js";

const matchTimeAdvantagekitToDashboardTopic =
  "/AdvantageKit/DriverStation/MatchTime";
const autonomousAdvantagekitToDashboardTopic =
  "/AdvantageKit/DriverStation/Autonomous";
const alertsSmartdashboardToDashboardTopic = {
  warnings: "/SmartDashboard/Alerts/warnings",
  infos: "/SmartDashboard/Alerts/infos",
  errors: "/SmartDashboard/Alerts/errors",
};

const coralRobotToDashboardTopic =
  "/objective_selector/coral_robot_to_dashboard";
const coralDashboardToRobotTopic =
  "/objective_selector/coral_dashboard_to_robot";
const algaeRobotToDashboardTopic =
  "/objective_selector/algae_robot_to_dashboard";
const algaeDashboardToRobotTopic =
  "/objective_selector/algae_dashboard_to_robot";
const intakeRobotToDashboardTopic =
  "/objective_selector/intake_robot_to_dashboard";
const intakeDashboardToRobotTopic =
  "/objective_selector/intake_dashboard_to_robot";

let isAuto = false;
let matchTime = 0;

export let coral = -1;
export let algae = -1;
export let intake = -1;

let client = new NT4_Client(
  window.location.hostname,
  "ObjectiveSelector",
  (topic) => { }, // Topic Announce
  () => { }, // Topic Unannounce
  (topic, timestamp, value) => {
    if (topic.name === matchTimeAdvantagekitToDashboardTopic) {
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

    switch (topic.name) {
      case coralRobotToDashboardTopic:
        const { rack, side, level } = getCoral(value);
        displaySelectedBranch(rack, side);
        displaySelectedLevel(level);
        coral = value;
        break;
      case algaeRobotToDashboardTopic:
        displaySelectedAlgaeScoring(value);
        intake = value;
        break;
      case intakeRobotToDashboardTopic:
        displaySelectedRack(value);
        algae = value;
        break;
    }
  }, // New data
  () => {
    const overlay = document.getElementById("overlay");
    if (overlay) overlay.remove();
  }, // Connect
  () => {
    displaySelectedRack();
    displaySelectedBranch();
    displaySelectedLevel();
    displaySelectedAlgaeScoring();
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
      matchTimeAdvantagekitToDashboardTopic,
      autonomousAdvantagekitToDashboardTopic,
      alertsSmartdashboardToDashboardTopic.infos,
      alertsSmartdashboardToDashboardTopic.warnings,
      alertsSmartdashboardToDashboardTopic.errors,
      coralRobotToDashboardTopic,
      algaeRobotToDashboardTopic,
      intakeRobotToDashboardTopic,
    ],
    false,
    false,
    0.02
  );
  client.publishTopic(coralDashboardToRobotTopic, "int");
  client.publishTopic(algaeDashboardToRobotTopic, "int");
  client.publishTopic(intakeDashboardToRobotTopic, "int");
  client.connect();

  createBranchSelector();
};

export function sendSelectedBranch(rack, side) {
  const { rack: prevRack, side: prevSide, level } = getCoral(coral);
  if (prevRack !== rack || prevSide !== side) {
    client.addSample(
      coralDashboardToRobotTopic,
      getCoralIdx({ rack, side, level })
    );
  }
}

export function sendSelectedLevel(level) {
  const { rack, side, level: prevLevel } = getCoral(coral);
  if (prevLevel !== level) {
    client.addSample(
      coralDashboardToRobotTopic,
      getCoralIdx({ rack, side, level })
    );
  }
}

export function sendSelectedAlgaeScoring(value) {
  if (intake !== value) {
    client.addSample(algaeDashboardToRobotTopic, value);
  }
}

export function sendSelectedRack(rack) {
  if (algae !== rack) {
    client.addSample(intakeDashboardToRobotTopic, rack);
  }
}
