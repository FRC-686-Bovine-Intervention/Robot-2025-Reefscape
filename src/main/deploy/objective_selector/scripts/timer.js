import { formatTime } from "./utils.js";

const timerContainer = document.getElementById("timer_container");
const timerDOM = document.getElementById("timer");

export function displayTime(timeSeconds, isAuto) {
  timerContainer.classList = "";
  if (timeSeconds > 30 || timeSeconds === 0) {
    timerContainer.classList.add("teleop-1");
  } else if (timeSeconds > 15) {
    timerContainer.classList.add("teleop-2");
  } else if (isAuto) {
    timerContainer.classList.add("auto");
  } else {
    timerContainer.classList.add("teleop-3");
  }

  const modeTimeSeconds = isAuto ? 15 : 135;
  timerContainer.style.setProperty(
    "--percentage-fill",
    timeSeconds / modeTimeSeconds
  );
  timerDOM.textContent = formatTime(timeSeconds);
}
