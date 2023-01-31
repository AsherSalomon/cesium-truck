// https://stackoverflow.com/questions/1400213/3d-least-squares-plane

let barX = 0;
let barY = 0;
let barH = 0;
let barA0 = 0;
let barA1 = 0;

export function fitHeightPlane(points = []) { // points in the form [[x, y, h], ...]
  // https://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
  // 3.2.1 Pseudocode for Fitting by a Plane
  if (points.length < 3) { return false; }
  
  // Compute the mean of the points.
  const mean = [0, 0, 0];
  for (let i = 0; i < points.length; i++) {
    for (let j = 0; j < 3; j++) {
      mean[j] += points[i][j];
    }
  }
  for (let j = 0; j < 3; j++) {
    mean[j] /= points.length;
  }
  
  // Compute the linear system matrix and vector elements.
  let xxSum = 0;
  let xySum = 0;
  let xhSum = 0;
  let yySum = 0;
  let yhSum = 0;
  for (let i = 0; i < points.length; i++) {
    const diff = [0, 0, 0];
    for (let j = 0; j < 3; j++) {
      diff[j] = points[i][j] - mean[j];
    }
    xxSum += diff[0] * diff[0];
    xySum += diff[0] * diff[1];
    xhSum += diff[0] * diff[2];
    yySum += diff[1] * diff[1];
    yhSum += diff[1] * diff[2];
  }
  
  // Solve the linear system.
  const det = xxSum * yySum - xySum * xySum;
  console.log(xxSum, yySum, xySum, xySum);
  if (det != 0) {
    // Compute the fitted plane h(x, y) = barH + barA0 * (x - barX) + barA1 * (y - barY).
    barX = mean[0];
    barY = mean[1];
    barH = mean[2];
    barA0 = (yySum * xhSum - xySum * yhSum) / det;
    barA1 = (xxSum * yhSum - xySum * xhSum) / det;
//     console.log(barX, barY, barH, barA0, barA1);
    return true;
  } else {
    // The output is invalid. The points are all the same or they are collinear.
    return false;
  }
}

export function extrapolate(x, y) {
  const h = barA0 * (x - barX) + barA1 * (y - barY) + barH;
  return h;
}
