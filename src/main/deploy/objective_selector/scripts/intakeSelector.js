import { sendSelectedIntake } from "./index.js";

const intakeSelectorContainer = document.getElementById(
  "intake_selector_container"
);

const INTAKE_BUTTON_SIZE = 100;
intakeSelectorContainer.style.setProperty(
  "--intake-button-size",
  INTAKE_BUTTON_SIZE + "px"
);

const icons = {
  grid: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-grid-3x3"><rect width="18" height="18" x="3" y="3" rx="2"/><path d="M3 9h18"/><path d="M3 15h18"/><path d="M9 3v18"/><path d="M15 3v18"/></svg>`,
  cpu: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-cpu"><rect width="16" height="16" x="4" y="4" rx="2"/><rect width="6" height="6" x="9" y="9" rx="1"/><path d="M15 2v2"/><path d="M15 20v2"/><path d="M2 15h2"/><path d="M2 9h2"/><path d="M20 15h2"/><path d="M20 9h2"/><path d="M9 2v2"/><path d="M9 20v2"/></svg>`,
};

export const buttons = [
  {
    name: "Net",
    icon: icons.grid,
    value: 0,
    category: "common",
  },
  {
    name: "Processor",
    icon: icons.cpu,
    value: 1,
    category: "common",
  },
  {
    name: "Opponent Processor",
    icon: icons.cpu,
    value: 2,
    category: "uncommon",
  },
];

const categoryDOM = {};
const buttonDOM = [];

buttons.forEach((button) => {
  const category =
    categoryDOM[button.category] || document.createElement("div");
  categoryDOM[button.category] = category;
  category.classList.add("intake_button_container");
  const dom = document.createElement("div");
  dom.classList.add("intake_button");
  dom.innerHTML += button.icon;
  const text = document.createElement("p");
  text.textContent = button.name;
  dom.appendChild(text);
  category.appendChild(dom);
  intakeSelectorContainer.appendChild(category);
  buttonDOM.push(dom);

  const onIntakeButtonClick = () => sendSelectedIntake(button.value);
    
  dom.onclick = onIntakeButtonClick;
  dom.oncontextmenu = onIntakeButtonClick;
});

export let selectedButtonIndex = -1;

export function displaySelectedIntake(value) {
  buttons.forEach((button, i) => {
    if (button.value === value) {
      selectedButtonIndex = i;
      buttonDOM[i].classList.add("selected");
    } else {
      buttonDOM[i].classList.remove("selected");
    }
  });
}
