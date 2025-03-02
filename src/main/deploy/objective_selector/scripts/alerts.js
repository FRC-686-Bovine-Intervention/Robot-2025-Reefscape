import { setFlipped } from "./levelSelector.js";

const infosDOM = document.getElementById("infos");
const warningsDOM = document.getElementById("warnings");
const errorsDOM = document.getElementById("errors");

const icons = {
  info: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-info"><circle cx="12" cy="12" r="10" /><path d="M12 16v-4" /><path d="M12 8h.01" /></svg>`,
  warning: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-triangle-alert"><path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3"/><path d="M12 9v4" /><path d="M12 17h.01" /></svg>`,
  error: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-circle-x"><circle cx="12" cy="12" r="10" /><path d="m15 9-6 6" /><path d="m9 9 6 6" /></svg>`,
};

const moveSidebarRightButton = document.getElementById("move_sidebar_right");
const moveSidebarLeftButton = document.getElementById("move_sidebar_left");

const container = document.getElementById("container");
moveSidebarLeftButton.onclick = moveSidebar;
moveSidebarRightButton.onclick = moveSidebar;
function moveSidebar() {
  if (moveSidebarLeftButton.style.display === "none") {
    moveSidebarRight();
  } else {
    moveSidebarLeft();
  }
}

function swapPanels() {
  const childNodes = Array.from(container.children);
  if (container.style.gridTemplateColumns) {
    container.style.gridTemplateColumns = "";
  } else {
    container.style.gridTemplateColumns = "1fr 2fr 1fr 1fr"
  }
  childNodes.reverse().forEach((element, i) => {
    if (element.style.order) {
      element.style.order = "";
    } else {
      element.style.order = i;
    }
  });
}

export function moveSidebarRight() {
  if (moveSidebarLeftButton.style.display !== "block") {
    swapPanels();
    moveSidebarLeftButton.style.display = "block";
    moveSidebarRightButton.style.display = "none";
    setFlipped(true);
  }
}

export function moveSidebarLeft() {
  if (moveSidebarLeftButton.style.display === "block") {
    swapPanels();
    moveSidebarLeftButton.style.display = "none";
    moveSidebarRightButton.style.display = "block";
    setFlipped(false);
  }
}

function alert(type, text) {
  const alert = document.createElement("div");
  alert.classList.add("alert", type);
  alert.innerHTML = icons[type];
  alert.append(text);
  return alert;
}

export function displayInfos(texts) {
  if (texts.length > 0) {
    infosDOM.style.display = "";
  } else {
    infosDOM.style.display = "none";
  }
  infosDOM.innerHTML = "";
  texts
    .map((text) => alert("info", text))
    .forEach((dom) => infosDOM.appendChild(dom));
}

export function displayWarnings(texts) {
  if (texts.length > 0) {
    warningsDOM.style.display = "";
  } else {
    warningsDOM.style.display = "none";
  }
  warningsDOM.innerHTML = "";
  texts
    .map((text) => alert("warning", text))
    .forEach((dom) => warningsDOM.appendChild(dom));
}

export function displayErrors(texts) {
  if (texts.length > 0) {
    errorsDOM.style.display = "";
  } else {
    errorsDOM.style.display = "none";
  }
  errorsDOM.innerHTML = "";
  texts
    .map((text) => alert("error", text))
    .forEach((dom) => errorsDOM.appendChild(dom));
}
