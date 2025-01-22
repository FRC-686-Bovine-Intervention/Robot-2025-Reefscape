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
  gallery_vertical: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-gallery-vertical"><path d="M3 2h18"/><rect width="18" height="12" x="3" y="6" rx="2"/><path d="M3 22h18"/></svg>`,
  credit_card: `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-credit-card"><rect width="20" height="14" x="2" y="5" rx="2"/><line x1="2" x2="22" y1="10" y2="10"/></svg>`,
};

export const buttons = [
  {
    name: "Coral Station",
    icon: icons.credit_card,
    value: 0,
    category: "coral",
  },
  {
    name: "Net",
    icon: icons.grid,
    value: 1,
    category: "algae_common",
  },
  {
    name: "Processor",
    icon: icons.gallery_vertical,
    value: 2,
    category: "algae_common",
  },
  {
    name: "Opponent Processor",
    icon: icons.gallery_vertical,
    value: 3,
    category: "algae_uncommon",
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
