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
