const infosDOM = document.getElementById("infos");
const warningsDOM = document.getElementById("warnings");
const errorsDOM = document.getElementById("errors");

const icons = {
  info: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-info"><circle cx="12" cy="12" r="10" /><path d="M12 16v-4" /><path d="M12 8h.01" /></svg>`,
  warning: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-triangle-alert"><path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3"/><path d="M12 9v4" /><path d="M12 17h.01" /></svg>`,
  error: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-circle-x"><circle cx="12" cy="12" r="10" /><path d="m15 9-6 6" /><path d="m9 9 6 6" /></svg>`,
};

const newAlert = {
  info: (text) => {
    const alert = document.createElement("div");
    alert.classList.add("alert", "info");
    alert.innerHTML = icons.info;
    alert.append(text);
    return alert;
  },
  warning: (text) => {
    const alert = document.createElement("div");
    alert.classList.add("alert", "warning");
    alert.innerHTML = icons.warning;
    alert.append(text);
    return alert;
  },
  error: (text) => {
    const alert = document.createElement("div");
    alert.classList.add("alert", "error");
    alert.innerHTML = icons.error;
    alert.append(text);
    return alert;
  },
};

export function displayInfos(texts) {
  infosDOM.innerHTML = "";
  texts
    .map((text) => newAlert.info(text))
    .forEach((dom) => (infosDOM.appendChild(dom)));
}

export function displayWarnings(texts) {
  warningsDOM.innerHTML = "";
  texts
    .map((text) => newAlert.warning(text))
    .forEach((dom) => (warningsDOM.appendChild(dom)));
}

export function displayErrors(texts) {
  errorsDOM.innerHTML = "";
  texts
    .map((text) => newAlert.error(text))
    .forEach((dom) => (errorsDOM.appendChild(dom)));
}
