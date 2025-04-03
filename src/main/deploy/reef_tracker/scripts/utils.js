export function wrapNumber(num, min, max) {
  const range = max - min + 1;
  return ((((num - min) % range) + range) % range) + min;
}
