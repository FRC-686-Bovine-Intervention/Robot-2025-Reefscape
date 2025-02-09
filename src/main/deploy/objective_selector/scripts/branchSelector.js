import { sendSelectedBranch, sendSelectedRack } from "./index.js";
import { degreesToRadians, formatString, rotatePoint } from "./utils.js";

const HUB_SIZE = 500;
const BRANCH_BUTTON_SIZE = 70;
const BRANCH_BUTTON_NORMAL_OFFSET = 20;
const BRANCH_TEXT_SIDE_OFFSET = 90;
const BRANCH_TEXT_NORMAL_OFFSET = -75;
const SHOW_NUMBERS = false;
const BRANCH_TEXT_TEMPLATE_STRING = `%s ${SHOW_NUMBERS ? "(%s)" : ""}`;
const RACK_BUTTON_TEMPLATE_STRING = `%s ${SHOW_NUMBERS ? "(%s)" : ""}`;

const branchSelectorContainer = document.getElementById(
  "branch_selector_container"
);
branchSelectorContainer.style.setProperty("--hub-size", HUB_SIZE + "px");
branchSelectorContainer.style.setProperty(
  "--branch-button-size",
  BRANCH_BUTTON_SIZE + "px"
);
branchSelectorContainer.style.setProperty(
  "--branch-button-normal-offset",
  BRANCH_BUTTON_NORMAL_OFFSET + "px"
);

const branchDOM = [];
const rackDOM = [];

export function createBranchSelector() {
  const ring = document.createElement("div");
  ring.id = "ring";
  branchSelectorContainer.appendChild(ring);
  rackDOM.push(ring);

  const x = (HUB_SIZE / 2) * Math.tan(degreesToRadians(30));
  const y = HUB_SIZE / 2;

  const letters = "abcdefghijklmnopqrstuvwxyz".toUpperCase();

  for (let i = 0; i < 6; i++) {
    const angle = -degreesToRadians(i * 60);

    const [rackX, rackY] = rotatePoint(
      [x - HUB_SIZE / Math.sqrt(3) / 2, y + BRANCH_BUTTON_NORMAL_OFFSET],
      angle
    );
    const rackButton = document.createElement("div");
    rackButton.classList.add("rack_button");
    rackButton.style.setProperty("--x", rackX + "px");
    rackButton.style.setProperty("--y", rackY + "px");
    rackButton.textContent = formatString(
      RACK_BUTTON_TEMPLATE_STRING,
      i % 2 === 0 ? "H" : "L",
      i + 1,
    );
    branchSelectorContainer.appendChild(rackButton);

    rackDOM.push(rackButton);

    const [leftX, leftY] = rotatePoint(
      [-x + BRANCH_TEXT_SIDE_OFFSET, y + BRANCH_TEXT_NORMAL_OFFSET],
      angle
    );
    const leftBranchText = document.createElement("p");
    leftBranchText.classList.add("branch_text");
    leftBranchText.style.setProperty("--x", leftX + "px");
    leftBranchText.style.setProperty("--y", leftY + "px");
    leftBranchText.textContent = formatString(
      BRANCH_TEXT_TEMPLATE_STRING,
      letters[i * 2],
      i * 2 + 1
    );
    branchSelectorContainer.appendChild(leftBranchText);

    const [rightX, rightY] = rotatePoint(
      [x - BRANCH_TEXT_SIDE_OFFSET, y + BRANCH_TEXT_NORMAL_OFFSET],
      angle
    );
    const rightBranchText = document.createElement("p");
    rightBranchText.classList.add("branch_text");
    rightBranchText.style.setProperty("--x", rightX + "px");
    rightBranchText.style.setProperty("--y", rightY + "px");
    rightBranchText.textContent = formatString(
      BRANCH_TEXT_TEMPLATE_STRING,
      letters[i * 2 + 1],
      i * 2 + 2,
    );
    branchSelectorContainer.appendChild(rightBranchText);

    const leftBranchButton = document.createElement("div");
    leftBranchButton.style.setProperty("--rotation", angle + "rad");
    leftBranchButton.classList.add("branch_button", "left");
    branchSelectorContainer.appendChild(leftBranchButton);

    const rightBranchButton = document.createElement("div");
    rightBranchButton.style.setProperty("--rotation", angle + "rad");
    rightBranchButton.classList.add("branch_button", "right");
    branchSelectorContainer.appendChild(rightBranchButton);

    branchDOM.push([leftBranchButton, rightBranchButton]);

    const onLeftBranchButtonClick = () =>
      sendSelectedBranch(i, branchDOM[i].indexOf(leftBranchButton));

    const onRightBranchButtonClick = () =>
      sendSelectedBranch(i, branchDOM[i].indexOf(rightBranchButton));

    leftBranchButton.onclick = onLeftBranchButtonClick;
    leftBranchText.onclick = onLeftBranchButtonClick;
    leftBranchButton.oncontextmenu = onLeftBranchButtonClick;
    leftBranchText.oncontextmenu = onLeftBranchButtonClick;
    rightBranchButton.onclick = onRightBranchButtonClick;
    rightBranchText.onclick = onRightBranchButtonClick;
    rightBranchButton.oncontextmenu = onRightBranchButtonClick;
    rightBranchText.oncontextmenu = onRightBranchButtonClick;
  }

  rackDOM.forEach((rackButton, i) => {
    const onRackButtonClick = () => sendSelectedRack(i);
    rackButton.onclick = onRackButtonClick;
    rackButton.oncontextmenu = onRackButtonClick;
  });
}

export function displaySelectedBranch(rack, pipe) {
  branchDOM.forEach((arr, i) => {
    arr.forEach((dom, j) => {
      if (i === rack && j === pipe) {
        dom.classList.add("selected");
      } else {
        dom.classList.remove("selected");
      }
    });
  });
}

export function displaySelectedRack(rack) {
  rackDOM.forEach((dom, i) => {
    if (i === rack) {
      dom.classList.add("selected");
    } else {
      dom.classList.remove("selected");
    }
  });
}
