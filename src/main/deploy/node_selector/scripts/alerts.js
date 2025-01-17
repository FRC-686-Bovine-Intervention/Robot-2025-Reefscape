import { setFlipped } from "./levelSelector.js";

const infosDOM = document.getElementById("infos");
const warningsDOM = document.getElementById("warnings");
const errorsDOM = document.getElementById("errors");

// const infoHeader = document.getElementById("info_header");
// const warningHeader = document.getElementById("warning_header");
// const errorHeader = document.getElementById("error_header");

const icons = {
  info: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-info"><circle cx="12" cy="12" r="10" /><path d="M12 16v-4" /><path d="M12 8h.01" /></svg>`,
  warning: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-triangle-alert"><path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3"/><path d="M12 9v4" /><path d="M12 17h.01" /></svg>`,
  error: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-circle-x"><circle cx="12" cy="12" r="10" /><path d="m15 9-6 6" /><path d="m9 9 6 6" /></svg>`,
};

const moveSidebarRight = document.getElementById("move_sidebar_right");
const moveSidebarLeft = document.getElementById("move_sidebar_left");

const container = document.getElementById("container");
moveSidebarLeft.onclick = moveSidebar;
moveSidebarRight.onclick = moveSidebar;
function moveSidebar() {
  const childNodes = Array.from(container.children);
  [childNodes[0], childNodes[2]] = [childNodes[2], childNodes[0]];
  childNodes.forEach((element) => container.appendChild(element));
  if (moveSidebarLeft.style.display === "none") {
    moveSidebarLeft.style.display = "block";
    moveSidebarRight.style.display = "none";
    setFlipped(true);
  } else {
    moveSidebarLeft.style.display = "none";
    moveSidebarRight.style.display = "block";
    setFlipped(false);
  }
};

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
    // infoHeader.style.display = "";
  } else {
    infosDOM.style.display = "none";
    // infoHeader.style.display = "none";
  }
  infosDOM.innerHTML = "";
  texts
    .map((text) => alert("info", text))
    .forEach((dom) => infosDOM.appendChild(dom));
}

export function displayWarnings(texts) {
  if (texts.length > 0) {
    warningsDOM.style.display = "";
    // warningHeader.style.display = "";
  } else {
    warningsDOM.style.display = "none";
    // warningHeader.style.display = "none";
  }
  warningsDOM.innerHTML = "";
  texts
    .map((text) => alert("warning", text))
    .forEach((dom) => warningsDOM.appendChild(dom));
}

export function displayErrors(texts) {
  if (texts.length > 0) {
    // errorHeader.style.display = "";
    errorsDOM.style.display = "";
  } else {
    // errorHeader.style.display = "none";
    errorsDOM.style.display = "none";
  }
  errorsDOM.innerHTML = "";
  texts
    .map((text) => alert("error", text))
    .forEach((dom) => errorsDOM.appendChild(dom));
}
