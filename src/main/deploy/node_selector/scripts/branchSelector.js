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

const y = HUB_SIZE / 2;
const x = (HUB_SIZE / 2) * Math.tan(degreesToRadians(30));

const branchButton = document.createElement("div");
branchButton.classList.add("branch_button");

for (let i = 0; i < 6; i++) {
  const angle = degreesToRadians(i * 60);

  const [leftX, leftY] = rotatePoint(
    [-x + BRANCH_BUTTON_SIDE_OFFSET, y + BRANCH_BUTTON_NORMAL_OFFSET],
    angle
  );
  const leftBranch = branchButton.cloneNode();
  leftBranch.style.setProperty("--x", leftX + "px");
  leftBranch.style.setProperty("--y", leftY + "px");
  leftBranch.textContent = i * 2 + 2;
  branchSelectorContainer.appendChild(leftBranch);

  const [rightX, rightY] = rotatePoint(
    [x - BRANCH_BUTTON_SIDE_OFFSET, y + BRANCH_BUTTON_NORMAL_OFFSET],
    angle
  );
  const rightBranch = branchButton.cloneNode();
  rightBranch.style.setProperty("--x", rightX + "px");
  rightBranch.style.setProperty("--y", rightY + "px");
  rightBranch.textContent = i * 2 + 1;
  branchSelectorContainer.appendChild(rightBranch);
}
