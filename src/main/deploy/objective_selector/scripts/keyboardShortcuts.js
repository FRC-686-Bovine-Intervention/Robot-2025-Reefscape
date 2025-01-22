import { moveSidebarLeft, moveSidebarRight } from "./alerts.js";
import {
  algae,
  coral,
  sendSelectedBranch,
  sendSelectedIntake,
  sendSelectedLevel,
  sendSelectedRack
} from "./index.js";
import { buttons, selectedButtonIndex } from "./intakeSelector.js";
import { getCoral, wrapNumber } from "./utils.js";

const nodeKeybinds = [
  ["Digit1", "Numpad1"],
  ["Digit2", "Numpad2"],
  ["Digit3", "Numpad3"],
  ["Digit4", "Numpad4"],
  ["Digit5", "Numpad5"],
  ["Digit6", "Numpad6"],
  ["Digit7", "Numpad7"],
  ["Digit8", "Numpad8"],
  ["Digit9", "Numpad9"],
  ["Digit0", "NumpadDivide"],
  ["Minus", "NumpadMultiply"],
  ["Equal", "NumpadSubtract"],
];

const directionalKeybinds = {
  up: ["KeyW", "ArrowUp"],
  down: ["KeyS", "ArrowDown"],
  left: ["KeyA", "ArrowLeft"],
  right: ["KeyD", "ArrowRight"],
};

function getNodeWithKeyBind(key) {
  return nodeKeybinds.findIndex(
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
  const node = getNodeWithKeyBind(e.code);
  if (node >= 0) {
    if (e.ctrlKey) {
      e.preventDefault();
      if (node < 6) {
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
      if (e.shiftKey) {
        sendSelectedIntake(wrapNumber(selectedButtonIndex + (direction === "down" ? +1 : -1), 0, buttons.length - 1));
      } else {
        sendSelectedLevel(wrapNumber(selectedCoral.level + (direction === "down" ? -1 : +1), 0, 3));
        sendSelectedLevel(wrapNumber(selectedCoral.level + (direction === "down" ? -1 : +1), 0, 3));
      }
      break;
    case "left":
    case "right":
      if (e.ctrlKey) {
        e.preventDefault();
        sendSelectedRack(wrapNumber(algae + (direction === "left" ? -1 : +1), 0, 5));
      } else {
        const node = wrapNumber(
          selectedCoral.rack * 2 +
            selectedCoral.side +
            (direction === "left" ? -1 : +1),
          0,
          11
        );
        const rack = Math.floor(node / 2);
        const side = node % 2;
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
