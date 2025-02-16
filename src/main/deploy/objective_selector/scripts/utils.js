export function degreesToRadians(degrees) {
  return (degrees * Math.PI) / 180;
}

export function rotatePoint([x, y], angle) {
  return [
    x * Math.cos(angle) - y * Math.sin(angle),
    y * Math.cos(angle) + x * Math.sin(angle),
  ];
}

export function formatString(template, ...values) {
  return template.replace(/%s/g, () => values.shift());
}

export function formatTime(seconds) {
  if (seconds < 0) {
    return "-" + formatTime(-seconds);
  }

  const minutes = Math.floor(seconds / 60);
  const remainingSeconds = Math.floor(seconds % 60);
  const milliseconds = Math.floor((seconds % 1) * 1000);

  const paddedMinutes = String(minutes).padStart(2, "0");
  const paddedSeconds = String(remainingSeconds).padStart(2, "0");
  const paddedMilliseconds = String(milliseconds).padStart(3, "0");

  return `${paddedMinutes}:${paddedSeconds}.${paddedMilliseconds}`;
}

export function wrapNumber(num, min, max) {
  const range = max - min + 1;
  return ((((num - min) % range) + range) % range) + min;
}

export function isWithinRange(value, min, max) {
  return value >= min && value <= max;
}

/*
  4 bits for rack: [0 - 11]
  1 bit for side: [0, 1]
  2 bits for level: [0 - 3]
*/
const RACK_BITS = 4;
const SIDE_BITS = 1;
const LEVEL_BITS = 2;

function createBinaryOnes(numBits) {
  return Math.pow(2, numBits) - 1
}

export function getCoral(idx) {
  return {
    rack: idx >> (LEVEL_BITS + SIDE_BITS) & createBinaryOnes(RACK_BITS),
    side: idx >> LEVEL_BITS & createBinaryOnes(SIDE_BITS),
    level: idx & createBinaryOnes(LEVEL_BITS)
  }
}

export function getCoralIdx({ rack, side, level }) {
  return (rack << (SIDE_BITS + LEVEL_BITS)) | (side << LEVEL_BITS) | level;
}
