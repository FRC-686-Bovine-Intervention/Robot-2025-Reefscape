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
    return '-' + formatTime(-seconds);
  }

  const minutes = Math.floor(seconds / 60);
  const remainingSeconds = Math.floor(seconds % 60);
  const milliseconds = Math.floor((seconds % 1) * 1000);

  const paddedMinutes = String(minutes).padStart(2, '0');
  const paddedSeconds = String(remainingSeconds).padStart(2, '0');
  const paddedMilliseconds = String(milliseconds).padStart(3, '0');

  return `${paddedMinutes}:${paddedSeconds}.${paddedMilliseconds}`;
}