import { NT4_Client } from "./NT4.js";

// ***** NETWORKTABLES *****

const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";

const coralGoalTopicName = "CoralGoal";
const algaeGoalTopicName = "AlgaeGoal";

const selectedLevelTopicName = "SelectedLevel";
const l1TopicName = "Level1";
const l2TopicName = "Level2";
const l3TopicName = "Level3";
const l4TopicName = "Level4";
const algaeTopicName = "Algae";
const coopTopicName = "Coop";

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefTracker",
  (topic) => {}, // Topic Announce
  (topic) => {}, // Topic Unannounce
  (topic, timestamp, value) => {
    if (topic.name === toDashboardPrefix + coralGoalTopicName) {
      coralGoal = value;
    } else if (topic.name === toDashboardPrefix + algaeGoalTopicName) {
      algaeGoal = value;
    } else if (topic.name === toDashboardPrefix + selectedLevelTopicName) {
      selectedLevel = value;
    } else if (topic.name === toDashboardPrefix + l1TopicName) {
      l1State = value;
    } else if (topic.name === toDashboardPrefix + l2TopicName) {
      l2State = value;
    } else if (topic.name === toDashboardPrefix + l3TopicName) {
      l3State = value;
    } else if (topic.name === toDashboardPrefix + l4TopicName) {
      l4State = value;
    } else if (topic.name === toDashboardPrefix + algaeTopicName) {
      algaeState = value;
    } else if (topic.name === toDashboardPrefix + coopTopicName) {
      coopState = value;
    } else {
      return;
    }
    updateUI();
  }, // New data
  () => {
    const overlay = document.getElementById("overlay");
    if (overlay) overlay.remove();
  }, // Connect
  () => {
    if (!document.getElementById("overlay")) {
      const overlay = document.createElement("div");
      overlay.id = "overlay";

      Object.assign(overlay.style, {
        position: "fixed",
        width: "100%",
        height: "100%",
        top: "0",
        left: "0",
        right: "0",
        bottom: "0",
        backgroundColor: "rgba(0, 0, 0, 0.75)",
        zIndex: "1000",
        pointerEvents: "none",
      });

      document.body.appendChild(overlay);
    }
  } // Disconnect
);

// Start NT connection
window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      toDashboardPrefix + coralGoalTopicName,
      toDashboardPrefix + algaeGoalTopicName,
      toDashboardPrefix + selectedLevelTopicName,
      toDashboardPrefix + l1TopicName,
      toDashboardPrefix + l2TopicName,
      toDashboardPrefix + l3TopicName,
      toDashboardPrefix + l4TopicName,
      toDashboardPrefix + algaeTopicName,
      toDashboardPrefix + coopTopicName,
    ],
    false,
    false,
    0.02
  );

  ntClient.publishTopic(toRobotPrefix + coralGoalTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + algaeGoalTopicName, "int");

  ntClient.publishTopic(toRobotPrefix + selectedLevelTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l1TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l2TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l3TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l4TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + algaeTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + coopTopicName, "boolean");
  ntClient.connect();
});

let DUMB_MODE = localStorage.getItem("DUMB_MODE") === "true" || false;
let coralGoal = 0;
let algaeGoal = 0;

let selectedLevel = 0;
let l1State = 0;
let l2State = 0;
let l3State = 0;
let l4State = 0;
let algaeState = 0;
let coopState = false;

const dumbModeToggleDOM = document.getElementById("dumb_mode");
dumbModeToggleDOM.checked = DUMB_MODE;
dumbModeToggleDOM.addEventListener("change", (e) => {
  DUMB_MODE = e.target.checked;
  localStorage.setItem("DUMB_MODE", DUMB_MODE);
  updateUI();
});

const levelDOM = Array.from(document.querySelectorAll(".level")).reverse();
const levelCounterDOM = levelDOM.map((el) => el.querySelector(".level-text"));
const pipeDOM = Array.from(document.querySelectorAll(".pipe"));
const algaeDOM = Array.from(document.querySelectorAll(".algae"));
const l1AddDOM = document.getElementById("add");
const l1SubtractDOM = document.getElementById("subtract");
const coopDOM = document.getElementById("coop");
const netDOM = document.getElementById("net");
const processorDOM = document.getElementById("processor");
const oppProcessorDOM = document.getElementById("opp_processor");
const algaeGoalDOM = [netDOM, processorDOM, oppProcessorDOM];

function updateUI() {
  if (DUMB_MODE) {
    l1AddDOM.style.display = "none";
    l1SubtractDOM.style.display = "none";
    coopDOM.style.display = "none";
  } else {
    l1AddDOM.style.display = "";
    l1SubtractDOM.style.display = "";
    coopDOM.style.display = "";
  }

  levelDOM.forEach((element, index) => {
    if (
      (!DUMB_MODE && index > 0 && selectedLevel === index - 1) ||
      (DUMB_MODE && getCoral(coralGoal).level === index)
    ) {
      element.classList.add("selected");
    } else {
      element.classList.remove("selected");
    }
  });

  let rpLevelCount = 0;
  levelCounterDOM.forEach((element, index) => {
    if (DUMB_MODE) {
      element.innerHTML = "L" + (index + 1);
      return;
    }
    if (index === 0) {
      element.innerText = l1State;
      if (l1State >= 5) rpLevelCount++;
    } else {
      let count = 0;
      let levelState = [l2State, l3State, l4State][index - 1];
      for (let i = 0; i < 12; i++) {
        if (((1 << i) & levelState) > 0) {
          count++;
        }
      }
      element.innerText = count;
      if (count >= 5) rpLevelCount++;
    }
  });

  pipeDOM.forEach((element, index) => {
    let levelState = [l2State, l3State, l4State][selectedLevel];
    if (
      (!DUMB_MODE && ((1 << index) & levelState) > 0) ||
      (DUMB_MODE &&
        getCoral(coralGoal).rack * 2 + getCoral(coralGoal).side === index)
    ) {
      element.classList.add("selected");
    } else {
      element.classList.remove("selected");
    }
  });

  algaeDOM.forEach((element, index) => {
    if (DUMB_MODE) {
      element.style.display = "none";
      return;
    } else {
      element.style.display = "";
    }

    if (((1 << index) & algaeState) > 0) {
      element.classList.add("selected");
    } else {
      element.classList.remove("selected");
    }
  });

  algaeGoalDOM.forEach((element, index) => {
    if (algaeGoal === index) {
      element.classList.add("selected");
    } else {
      element.classList.remove("selected");
    }
  })

  if (coopState) {
    coopDOM.classList.add("selected");
  } else {
    coopDOM.classList.remove("selected");
  }
}

function bind(element, callback) {
  let lastActivation = 0;
  let activate = () => {
    if (new Date().getTime() - lastActivation > 250) {
      callback();
      lastActivation = new Date().getTime();
    }
  };

  element.addEventListener("touchstart", activate);
  element.addEventListener("click", activate);
  element.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    activate();
  });
}

window.addEventListener("load", () => {
  levelDOM.forEach((element, index) => {
    bind(element, () => {
      if (DUMB_MODE) {
        ntClient.addSample(
          toRobotPrefix + coralGoalTopicName,
          getCoralBin({ ...getCoral(coralGoal), level: index })
        );
      } else if (index > 0) {
        ntClient.addSample(toRobotPrefix + selectedLevelTopicName, index - 1);
      }
    });
  });

  pipeDOM.forEach((element, index) => {
    bind(element, () => {
      if (DUMB_MODE) {
        ntClient.addSample(
          toRobotPrefix + coralGoalTopicName,
          getCoralBin({
            ...getCoral(coralGoal),
            rack: Math.floor(index / 2),
            side: index % 2,
          })
        );
        return;
      }
      switch (selectedLevel) {
        case 0:
          ntClient.addSample(
            toRobotPrefix + l2TopicName,
            l2State ^ (1 << index)
          );
          break;
        case 1:
          ntClient.addSample(
            toRobotPrefix + l3TopicName,
            l3State ^ (1 << index)
          );
          break;
        case 2:
          ntClient.addSample(
            toRobotPrefix + l4TopicName,
            l4State ^ (1 << index)
          );
          break;
      }
    });
  });

  algaeDOM.forEach((element, index) => {
    bind(element, () => {
      if (DUMB_MODE) return;
      ntClient.addSample(
        toRobotPrefix + algaeTopicName,
        algaeState ^ (1 << index)
      );
    });
  });

  bind(l1AddDOM, () => {
    if (DUMB_MODE) return;
    if (l1State > 0) {
      ntClient.addSample(toRobotPrefix + l1TopicName, l1State - 1);
    }
  });
  bind(l1SubtractDOM, () => {
    if (DUMB_MODE) return;
    ntClient.addSample(toRobotPrefix + l1TopicName, l1State + 1);
  });

  bind(coopDOM, () => {
    ntClient.addSample(toRobotPrefix + coopTopicName, !coopState);
  });

  algaeGoalDOM.forEach((element, index) => {
    bind(element, () => {
      ntClient.addSample(toRobotPrefix + algaeGoalTopicName, index);
    })
  });
});

// DUMB MODE
/*
  4 bits for rack: [0 - 11]
  1 bit for side: [0, 1]
  2 bits for level: [0 - 3]
*/
const RACK_BITS = 4;
const SIDE_BITS = 1;
const LEVEL_BITS = 2;

function createBinaryOnes(numBits) {
  return Math.pow(2, numBits) - 1;
}

export function getCoral(bin) {
  return {
    rack: (bin >> (LEVEL_BITS + SIDE_BITS)) & createBinaryOnes(RACK_BITS),
    side: (bin >> LEVEL_BITS) & createBinaryOnes(SIDE_BITS),
    level: bin & createBinaryOnes(LEVEL_BITS),
  };
}

export function getCoralBin({ rack, side, level }) {
  return (rack << (SIDE_BITS + LEVEL_BITS)) | (side << LEVEL_BITS) | level;
}
