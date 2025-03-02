import { moveSidebarLeft, moveSidebarRight } from "./alerts.js";
import {
  algae,
  coral,
  sendSelectedBranch,
  sendSelectedAlgaeScoring,
  sendSelectedLevel,
  sendSelectedRack,
} from "./index.js";
import { buttons, selectedButtonIndex } from "./algaeScoringSelector.js";
import { getCoral, isWithinRange, wrapNumber } from "./utils.js";

const branchKeybinds = [
  ["Digit1", "Numpad1", "KeyA"],
  ["Digit2", "Numpad2", "KeyB"],
  ["Digit3", "Numpad3", "KeyC"],
  ["Digit4", "Numpad4", "KeyD"],
  ["Digit5", "Numpad5", "KeyE"],
  ["Digit6", "Numpad6", "KeyF"],
  ["Digit7", "Numpad7", "KeyG"],
  ["Digit8", "Numpad8", "KeyH"],
  ["Digit9", "Numpad9", "KeyI"],
  ["Digit0", "NumpadDivide", "KeyJ"],
  ["Minus", "NumpadMultiply", "KeyK"],
  ["Equal", "NumpadSubtract", "KeyL"],
];

const directionalKeybinds = {
  up: "ArrowUp",
  down: "ArrowDown",
  left: "ArrowLeft",
  right: "ArrowRight",
};

function getBranchWithKeyBind(key) {
  return branchKeybinds.findIndex(
    (arr) => (Array.isArray(arr) && arr.includes(key)) || arr === key
  );
}

function getDirectionWithKeyBind(key) {
  return Object.keys(directionalKeybinds)[
    Object.values(directionalKeybinds).findIndex(
      (arr) => (Array.isArray(arr) && arr.includes(key)) || arr === key
    )
  ];
}

window.onkeydown = (e) => {
  const node = getBranchWithKeyBind(e.code);
  if (node >= 0) {
    if (e.ctrlKey) {
      if (node <= 6) {
        e.preventDefault();
        sendSelectedRack(node);
      }
    } else {
      const rack = Math.floor(node / 2);
      const side = node % 2;
      sendSelectedBranch(rack, side);
    }
  }

  const selectedCoral = getCoral(coral);
  const direction = getDirectionWithKeyBind(e.code);
  switch (direction) {
    case "up":
    case "down":
      sendSelectedLevel(
        wrapNumber(selectedCoral.level + (direction === "down" ? -1 : +1), 0, 3)
      );
      sendSelectedLevel(
        wrapNumber(selectedCoral.level + (direction === "down" ? -1 : +1), 0, 3)
      );
      break;
    case "left":
    case "right":
      if (e.ctrlKey) {
        e.preventDefault();
        sendSelectedRack(
          wrapNumber(algae + (direction === "left" ? -1 : +1), 0, 6)
        );
      } else if (e.shiftKey) {
        sendSelectedAlgaeScoring(
          wrapNumber(
            selectedButtonIndex + (direction === "left" ? -1 : +1),
            0,
            buttons.length - 1
          )
        );
      } else {
        let newSide = selectedCoral.side + (direction === "left" ? -1 : +1);
        const moveRack = !isWithinRange(newSide, 0, 1);
        const side = wrapNumber(newSide, 0, 1);
        const rack = wrapNumber(selectedCoral.rack + (moveRack ? (direction === "left" ? -1 : +1) : 0), 0, 5);
        sendSelectedBranch(rack, side);
      }
      break;
  }

  if (e.ctrlKey && e.shiftKey) {
    if (e.code === "KeyL") {
      moveSidebarLeft();
    } else if (e.code === "KeyR") {
      e.preventDefault();
      moveSidebarRight();
    }
  }
};
