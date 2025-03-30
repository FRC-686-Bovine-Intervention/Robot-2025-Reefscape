import { NT4_Client } from "../lib/NT4.js";

const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";

const modeTopicName = "Mode";
const coralGoalTopicName = "CoralGoal";
const algaeGoalTopicName = "AlgaeGoal";

const coralTopicName = "Coral";
const l1TopicName = "Level1";
const algaeTopicName = "Algae";
const coopTopicName = "Coop";
const priorityListTopicName = "PriorityList";

let mode = "SMART";
let coralGoal = 0;
let algaeGoal = 0;
let l1State = 0;
let coralState = [];
let algaeState = [];
let coopState = 0;
let priorityListState = [];

let selectedLevel = 0;

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefTracker",
  (topic) => {}, // Topic Announce
  (topic) => {}, // Topic Unannounce
  (topic, timestamp, value) => {
    if (topic.name === toDashboardPrefix + modeTopicName) {
      mode = value === 0 ? "SMART" : "DUMB";
    } else if (topic.name === toDashboardPrefix + coralGoalTopicName) {
      coralGoal = value;
    } else if (topic.name === toDashboardPrefix + algaeGoalTopicName) {
      algaeGoal = value;
    } else if (topic.name === toDashboardPrefix + coralTopicName) {
      coralState = convertIntToBooleanArr(value, 36);
    } else if (topic.name === toDashboardPrefix + l1TopicName) {
      l1State = value;
    } else if (topic.name === toDashboardPrefix + algaeTopicName) {
      algaeState = convertIntToBooleanArr(value, 6);
    } else if (topic.name === toDashboardPrefix + coopTopicName) {
      coopState = value;
    } else if (topic.name === toDashboardPrefix + priorityListTopicName) {
      priorityListState = unpackInt(value, 24, 3);
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
      });

      document.body.appendChild(overlay);
    }
  } // Disconnect
);

// Start NT connection
window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      toDashboardPrefix + modeTopicName,
      toDashboardPrefix + coralGoalTopicName,
      toDashboardPrefix + algaeGoalTopicName,
      toDashboardPrefix + coralTopicName,
      toDashboardPrefix + l1TopicName,
      toDashboardPrefix + algaeTopicName,
      toDashboardPrefix + coopTopicName,
      toDashboardPrefix + priorityListTopicName,
    ],
    false,
    false,
    0.02
  );

  ntClient.publishTopic(toRobotPrefix + modeTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + coralGoalTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + algaeGoalTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + coralTopicName, "double");
  ntClient.publishTopic(toRobotPrefix + l1TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + algaeTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + coopTopicName, "boolean");
  ntClient.publishTopic(toRobotPrefix + priorityListTopicName, "int[]");
  ntClient.connect();
});

const levelDOM = Array.from(document.querySelectorAll(".level")).reverse();
const levelCounterDOM = levelDOM.map((el) => el.querySelector(".level-text"));
const pipeDOM = Array.from(document.querySelectorAll(".pipe"));
const algaeDOM = Array.from(document.querySelectorAll(".algae"));
const l1AddDOM = document.getElementById("add");
const l1SubtractDOM = document.getElementById("subtract");
const coopDOM = document.getElementById("coop");
const algaeGoalDOM = [
  document.getElementById("net"),
  document.getElementById("processor"),
  document.getElementById("opp_processor"),
];
const modeToggleDOM = document.getElementById("mode");
const priorityListDOM = document.getElementById("priority_list");
const priorityDOM = Array.from(priorityListDOM.querySelectorAll(".priority"));
const prioritySlotDOM = priorityDOM.map((element) => element.parentElement);
const priorityItems = new Map(
  priorityDOM.map((item) => [item.dataset.idx, item])
);
const priorityUpdatedIndicated = document.querySelector(
  "#priority_list .updated"
);

function updateUI() {
  if (mode === "DUMB") {
    l1AddDOM.style.display = "none";
    l1SubtractDOM.style.display = "none";
    coopDOM.style.display = "none";
    priorityListDOM.style.display = "none";
  } else {
    l1AddDOM.style.display = "";
    l1SubtractDOM.style.display = "";
    coopDOM.style.display = "";
    priorityListDOM.style.display = "";
  }

  modeToggleDOM.checked = mode === "SMART";

  priorityListState.forEach((idx, i) => {
    const item = priorityItems.get(String(idx));
    if (item) prioritySlotDOM[i].appendChild(item);
    priorityUpdatedIndicated.style.display = "";
  });

  levelDOM.forEach((element, index) => {
    if (
      (mode === "SMART" && index > 0 && selectedLevel === index - 1) ||
      (mode === "DUMB" && getCoral(coralGoal).level === index)
    ) {
      element.classList.add("selected");
    } else {
      element.classList.remove("selected");
    }
  });

  let rpLevelCount = 0;
  levelCounterDOM.forEach((element, index) => {
    if (mode === "DUMB") {
      element.innerHTML = "L" + (index + 1);
      return;
    }
    let count = 0;
    if (index === 0) {
      count = l1State;
    } else {
      for (let i = 0; i < 12; i++) {
        count += coralState[getCoralID({ level: index - 1, pipe: i })] ? 1 : 0;
      }
    }
    if (count >= 5) rpLevelCount++;
    element.innerText = count;
    priorityDOM
      .filter((element) => element.dataset.level - 1 == index)
      .forEach((element) => {
        const neededCount = element.dataset.count;
        const percentage = Math.min(count / neededCount, 1);
        if (neededCount)
          element.style.setProperty("--percentage-complete", percentage);
        if (percentage === 1) {
          element.classList.add("complete");
        } else {
          element.classList.remove("complete");
        }
      });
  });

  pipeDOM.forEach((element, index) => {
    if (
      (mode === "SMART" &&
        coralState[getCoralID({ level: selectedLevel, pipe: index })]) ||
      (mode === "DUMB" && getCoral(coralGoal).pipe === index)
    ) {
      element.classList.add("selected");
    } else {
      element.classList.remove("selected");
    }
  });

  algaeDOM.forEach((element, index) => {
    if (mode === "DUMB") {
      element.style.display = "none";
      return;
    } else {
      element.style.display = "";
    }

    if (!algaeState[index]) {
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
  });

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

  element.addEventListener("touchstart", (event) => {
    event.preventDefault();
    activate();
  });
  element.addEventListener("click", (event) => {
    event.preventDefault();
    activate();
  });
  element.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    activate();
  });
}

let swaps = [];

window.addEventListener("load", () => {
  bind(modeToggleDOM, () => {
    ntClient.addSample(toRobotPrefix + modeTopicName, mode === "SMART" ? 1 : 0);
  });

  levelDOM.forEach((element, index) => {
    bind(element, () => {
      if (mode === "DUMB") {
        ntClient.addSample(
          toRobotPrefix + coralGoalTopicName,
          getCoralID({ ...getCoral(coralGoal), level: index })
        );
      } else if (index > 0) {
        selectedLevel = index - 1;
        updateUI();
      }
    });
  });

  pipeDOM.forEach((element, index) => {
    bind(element, () => {
      if (mode === "DUMB") {
        ntClient.addSample(
          toRobotPrefix + coralGoalTopicName,
          getCoralID({
            ...getCoral(coralGoal),
            pipe: index,
          })
        );
        return;
      }
      const id = getCoralID({ level: selectedLevel, pipe: index });
      const offset = coralState[id] ? 36 : 0;
      ntClient.addSample(toRobotPrefix + coralTopicName, id - offset);
    });
  });

  algaeDOM.forEach((element, index) => {
    bind(element, () => {
      if (mode === "DUMB") return;
      const id = index;
      const offset = algaeState[id] ? 6 : 0;
      ntClient.addSample(toRobotPrefix + algaeTopicName, id - offset);
    });
  });

  bind(l1AddDOM, () => {
    if (mode === "DUMB") return;
    ntClient.addSample(toRobotPrefix + l1TopicName, +1);
  });
  bind(l1SubtractDOM, () => {
    if (mode === "DUMB") return;
    if (l1State > 0) {
      ntClient.addSample(toRobotPrefix + l1TopicName, -1);
    }
  });

  bind(coopDOM, () => {
    ntClient.addSample(toRobotPrefix + coopTopicName, !coopState);
  });

  algaeGoalDOM.forEach((element, index) => {
    bind(element, () => {
      ntClient.addSample(toRobotPrefix + algaeGoalTopicName, index);
    });
  });

  const swapy = Swapy.createSwapy(priorityListDOM);
  swapy.onSwap((event) => {
    swaps.push([parseInt(event.fromSlot), parseInt(event.toSlot)]);
  });

  swapy.onSwapEnd(() => {
    ntClient.addSample(
      toRobotPrefix + priorityListTopicName,
      swaps.map((swap) => packInt(swap, 3))
    );
    swaps = [];
    priorityUpdatedIndicated.style.display = "none";
  });
});

function getCoralID({ pipe, level }) {
  return level * 12 + pipe;
}

function getCoral(id) {
  return { pipe: id % 12, level: Math.floor(id / 12) };
}

function convertIntToBooleanArr(n, len) {
  if (typeof n !== "bigint") {
    n = BigInt(n);
  }
  const arr = [];
  for (let i = len - 1; i >= 0; i--) {
    arr[i] = (n & 1n) === 1n;
    n = n >> 1n;
  }
  return arr;
}

function unpackInt(n, totalBits, size) {
  let values = [];
  for (let i = 0; i < totalBits / size; i++) {
    values.unshift(n & ((1 << size) - 1));
    n >>= size;
  }
  return values;
}

function packInt(values, size) {
  let n = 0;
  for (let i = 0; i < values.length; i++) {
    n = (n << size) | values[i];
  }
  return n;
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}