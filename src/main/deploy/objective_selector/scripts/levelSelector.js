import { sendSelectedLevel } from "./index.js";
import { formatString } from "./utils.js";

const BRANCH_SIZE = 400;
const LEVEL_BUTTON_SIZE = 100;
const LEVEL_BUTTON_TEMPLATE_STRING = "L%s";

const levelSelectorContainer = document.getElementById(
  "level_selector_container"
);
levelSelectorContainer.style.setProperty("--branch-size", BRANCH_SIZE + "px");
levelSelectorContainer.style.setProperty(
  "--level-button-size",
  LEVEL_BUTTON_SIZE + "px"
);

const branch = document.getElementById("branch");
const svg = branch.querySelector("svg");
const paths = svg.querySelectorAll("path");

const levelsDOM = [];

paths.forEach((path, i) => {
  const pathLength = path.getTotalLength();

  let topMostX = 0;
  let topMostY = Infinity;

  for (let i = 0; i <= pathLength; i++) {
    const point = path.getPointAtLength(i);

    if (point.y <= topMostY) {
      topMostX = point.x;
      topMostY = point.y;
    }
  }

  const levelButton = document.createElement("div");
  levelButton.classList.add("level_button");
  levelButton.style.setProperty("left", topMostX + "px");
  levelButton.style.setProperty("top", topMostY + "px");
  levelButton.textContent = formatString(LEVEL_BUTTON_TEMPLATE_STRING, i + 1);

  branch.appendChild(levelButton);
  levelsDOM.push(levelButton);

  const onLevelButtonClick = () => sendSelectedLevel(i);

  levelButton.onclick = onLevelButtonClick;
  levelButton.oncontextmenu = onLevelButtonClick;
});

export function displaySelectedLevel(level) {
  levelsDOM.forEach((dom, i) => {
    if (i === level) {
      dom.classList.add("selected");
    } else {
      dom.classList.remove("selected");
    }
  });
}

export function setFlipped(value) {
  if (value) {
    branch.classList.add("flipped");
  } else {
    branch.classList.remove("flipped");
  }
}
