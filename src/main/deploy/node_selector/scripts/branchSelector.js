import { sendSelectedBranch } from "./index.js";
import { degreesToRadians, rotatePoint } from "./utils.js";

const HUB_SIZE = 400;
const BRANCH_BUTTON_SIZE = 70;
const BRANCH_BUTTON_SIDE_OFFSET = 65;
const BRANCH_BUTTON_NORMAL_OFFSET = 20;

const branchSelectorContainer = document.getElementById(
  "branch_selector_container"
);
branchSelectorContainer.style.setProperty("--hub-size", HUB_SIZE + "px");
branchSelectorContainer.style.setProperty(
  "--branch-button-size",
  BRANCH_BUTTON_SIZE + "px"
);

// const x = (HUB_SIZE / 2) * Math.tan(degreesToRadians(30));
// const y = HUB_SIZE / 2;

// const branchButton = document.createElement("div");
// branchButton.classList.add("branch_button");

const reefDOM = [];

for (let i = 0; i < 6; i++) {
  const angle = -i * 60;

  /*
  const [leftX, leftY] = rotatePoint(
    [-x + BRANCH_BUTTON_SIDE_OFFSET, y + BRANCH_BUTTON_NORMAL_OFFSET],
    angle
  );
  const leftBranchButton = branchButton.cloneNode();
  leftBranchButton.style.setProperty("--x", leftX + "px");
  leftBranchButton.style.setProperty("--y", leftY + "px");
  leftBranchButton.textContent = i * 2 + 1;
  branchSelectorContainer.appendChild(leftBranchButton);

  const [rightX, rightY] = rotatePoint(
    [x - BRANCH_BUTTON_SIDE_OFFSET, y + BRANCH_BUTTON_NORMAL_OFFSET],
    angle
  );
  const rightBranchButton = branchButton.cloneNode();
  rightBranchButton.style.setProperty("--x", rightX + "px");
  rightBranchButton.style.setProperty("--y", rightY + "px");
  rightBranchButton.textContent = i * 2 + 2;
  branchSelectorContainer.appendChild(rightBranchButton);
  */

  const leftBranchButton = document.createElement("div");
  leftBranchButton.style.setProperty("--rotation", angle + "deg");
  leftBranchButton.classList.add("branch_button", "left");
  branchSelectorContainer.appendChild(leftBranchButton);
  
  const rightBranchButton = document.createElement("div");
  rightBranchButton.style.setProperty("--rotation", angle + "deg");
  rightBranchButton.classList.add("branch_button", "right");
  branchSelectorContainer.appendChild(rightBranchButton);

  reefDOM.push([leftBranchButton, rightBranchButton]);

  leftBranchButton.onclick = () =>
    sendSelectedBranch(i, reefDOM[i].indexOf(leftBranchButton));
  rightBranchButton.onclick = () =>
    sendSelectedBranch(i, reefDOM[i].indexOf(rightBranchButton));
}

export function displaySelectedBranch(rack, side) {
  reefDOM.forEach((arr, i) => {
    arr.forEach((dom, j) => {
      if (i === rack && j === side) {
        dom.classList.add("selected");
      } else {
        dom.classList.remove("selected");
      }
    });
  });
}
