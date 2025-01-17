import { moveSidebarLeft, moveSidebarRight } from "./alerts.js";
import {
  getSelectedNode,
  sendSelectedBranch,
  sendSelectedLevel,
} from "./index.js";
import { wrapNumber } from "./utils.js";

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
    const rack = Math.floor(node / 2);
    const side = Math.ceil(node / 2 - rack);
    sendSelectedBranch(rack, side);
  }

  const selectedNode = getSelectedNode();
  const direction = getDirectionWithKeyBind(e.code);
  switch (direction) {
    case "up":
      sendSelectedLevel(wrapNumber(selectedNode[1] + 1, 0, 3));
      break;
    case "down":
      sendSelectedLevel(wrapNumber(selectedNode[1] - 1, 0, 3));
      break;
    case "left":
    case "right":
      const node = wrapNumber(
        selectedNode[0] * 2 +
          selectedNode[2] +
          (direction === "left" ? -1 : +1),
        0,
        11
      );
      const rack = Math.floor(node / 2);
      const side = Math.ceil(node / 2 - rack);
      sendSelectedBranch(rack, side);
      break;
  }

  if (e.ctrlKey && e.shiftKey) {
    e.preventDefault();
    if (e.code === "KeyL") {
      moveSidebarLeft();
    } else if (e.code === "KeyR") {
      moveSidebarRight();
    }
  }
};
