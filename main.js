import * as physics from './physics.js';

const truckEntities = [];
const initPosition = [-71.303343, 44.269824, 1916.7 - 33.9];

// Your access token can be found at: https://ion.cesium.com/tokens.
// Replace `your_access_token` with your Cesium ion access token.
Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1ZGI0ZWQ4Ny0zNjNjLTRmZDAtYWI2YS0yNTc0N2ViNjY0OTEiLCJpZCI6MTAwNDY2LCJpYXQiOjE2NzMwOTcwNjl9.7118r-iVka0QWiKRgKK2u46jqyibkOwYaewwdnFlHUU';

// Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
const viewer = new Cesium.Viewer('cesiumContainer', {
  terrainProvider: Cesium.createWorldTerrain({
    requestWaterMask: true,
    requestVertexNormals: true,
  }),
  geocoder: false,
  homeButton: false,
  sceneModePicker: false,
  baseLayerPicker: false,
  navigationHelpButton: false,
  animation: false,
  timeline: false,
  fullscreenButton: false,
});

viewer.clock.clockStep = Cesium.ClockStep.SYSTEM_CLOCK;
viewer.scene.globe.enableLighting = true;
viewer.shadows = true;
viewer.scene.globe.depthTestAgainstTerrain = true;
viewer.scene.moon = new Cesium.Moon();

// Add Cesium OSM Buildings, a global 3D buildings layer.
const buildingTileset = viewer.scene.primitives.add(Cesium.createOsmBuildings());   

viewer.camera.flyTo({
  destination : Cesium.Cartesian3.fromDegrees(
    initPosition[0] + 0.0003,
    initPosition[1],
    initPosition[2] + 6.6
  ),
  orientation : {
    heading : Cesium.Math.toRadians(270),
    pitch : Cesium.Math.toRadians(-15),
  }
});

function createModel(url) {
  viewer.entities.removeAll();
  const position = Cesium.Cartesian3.fromDegrees(
    initPosition[0],
    initPosition[1],
    initPosition[2]
  );
  const heading = Cesium.Math.toRadians(54);
  const pitch = 0;
  const roll = 0;
  const hpr = new Cesium.HeadingPitchRoll(heading, pitch, roll);
  const orientation = Cesium.Transforms.headingPitchRollQuaternion(
    position,
    hpr
  );
  const entity = viewer.entities.add({
    name: url,
    position: position,
    orientation: orientation,
    model: {
      uri: url, // Cesium.ModelGraphics
    },
  });
  viewer.trackedEntity = entity;
  return entity;
}

truckEntities[0] = createModel('1984_Ford_F350.glb');
// truckEntities[0] = viewer.entities.add({model: {uri: '1984_Ford_F350.glb'}});
for (let i = 1; i <= 4; i++) {
  truckEntities[i] = viewer.entities.add({model: {uri: '1984_Ford_F350_wheel.glb'}});
}
truckEntities.now = function() { return viewer.clock.currentTime; }

let followTruck = false;
document.addEventListener('mousemove', function(e) {
  followTruck = false;
});
var mouseDown = false;
document.body.onmousedown = function() { 
  mouseDown = true;
}
document.body.onmouseup = function() {
  mouseDown = false;
}
window.addEventListener('keydown', function(e) {
  followTruck = true;
//   if (e.keyCode == 69) { // the E key
  const key = e.keyCode;
  if (key == 65 || key == 68 || key == 83 || key == 87) { // the E key
//     if (viewer.trackedEntity == truckEntities[0]) {
//       viewer.trackedEntity = null;
//     } else 
    if (viewer.trackedEntity != truckEntities[0]) {
      const matrix4 = new Cesium.Matrix4();
      Cesium.Transforms.eastNorthUpToFixedFrame(
        truckEntities[0].position._value, Cesium.Ellipsoid.WGS84, matrix4);
      Cesium.Matrix4.inverse(matrix4, matrix4);
      const position = new Cesium.Cartesian3();
      Cesium.Matrix4.multiplyByPoint(matrix4, viewer.camera.position, position);
      truckEntities[0].viewFrom = position;
      viewer.trackedEntity = truckEntities[0];
    }
  }
});

function adjustHeightForTerrain(controller) {
  controller._adjustedHeightForTerrain = true;
  const scene = controller._scene;
  const mode = scene.mode;
  const globe = scene.globe;
  if (
    !Cesium.defined(globe) ||
    mode === Cesium.SceneMode.SCENE2D ||
    mode === Cesium.SceneMode.MORPHING
  ) {
    return;
  }
  const camera = scene.camera;
  const ellipsoid = globe.ellipsoid;
  const projection = scene.mapProjection;
  let transform;
  let mag;
  if (!Cesium.Matrix4.equals(camera.transform, Cesium.Matrix4.IDENTITY)) {
    transform = Cesium.Matrix4.clone(camera.transform, new Cesium.Matrix4());
    mag = Cesium.Cartesian3.magnitude(camera.position);
    camera._setTransform(Cesium.Matrix4.IDENTITY);
  }
  const cartographic = new Cesium.Cartographic();
  if (mode === Cesium.SceneMode.SCENE3D) {
    ellipsoid.cartesianToCartographic(camera.position, cartographic);
  } else {
    projection.unproject(camera.position, cartographic);
  }
  let heightUpdated = false;
  if (cartographic.height < controller._minimumCollisionTerrainHeight) {
    const globeHeight = controller._scene.globeHeight;
    if (Cesium.defined(globeHeight)) {
      const height = globeHeight + controller.minimumZoomDistance;
      if (cartographic.height < height) {
        cartographic.height = height;
        if (mode === Cesium.SceneMode.SCENE3D) {
          ellipsoid.cartographicToCartesian(cartographic, camera.position);
        } else {
          projection.project(cartographic, camera.position);
        }
        heightUpdated = true;
      }
    }
  }
  if (Cesium.defined(transform)) {
    camera._setTransform(transform);
    if (heightUpdated) {
      Cesium.Cartesian3.normalize(camera.position, camera.position);
      Cesium.Cartesian3.negate(camera.position, camera.direction);
      Cesium.Cartesian3.multiplyByScalar(
        camera.position,
        Math.max(mag, controller.minimumZoomDistance),
        camera.position
      );
      Cesium.Cartesian3.normalize(camera.direction, camera.direction);
      Cesium.Cartesian3.cross(camera.direction, camera.up, camera.right);
      Cesium.Cartesian3.cross(camera.right, camera.direction, camera.up);
    }
  }
}

function update() {

  if (viewer.trackedEntity == truckEntities[0] && followTruck && mouseDown == false) {
    const vehicleDirection = new Cesium.Cartesian3(0, 1, 0); // -0.2);
    const quaternion = truckEntities[0].orientation.getValue(truckEntities.now());
    const matrix3 = new Cesium.Matrix3();
    Cesium.Matrix3.fromQuaternion(quaternion, matrix3);
    Cesium.Matrix3.multiplyByVector(matrix3, vehicleDirection, vehicleDirection);
    const crossProduct = new Cesium.Cartesian3();
    Cesium.Cartesian3.cross(viewer.camera.directionWC, vehicleDirection, crossProduct);
    const dotProduct = Cesium.Cartesian3.dot(viewer.camera.upWC, crossProduct);
    viewer.camera.rotateRight(dotProduct * Math.PI / 128);
    
    const camToTruck = Cesium.Cartesian3.subtract(truckEntities[0].position._value, viewer.camera.positionWC, new Cesium.Cartesian3);
    const forwardMove = (Cesium.Cartesian3.magnitude(camToTruck) - 20) * 0.05;
    viewer.camera.moveForward(forwardMove);
    
//     const dotProductRight = Cesium.Cartesian3.dot(viewer.camera.rightWC, crossProduct);
//     viewer.camera.rotateUp(dotProductRight * Math.PI / 128);
    
    const upDirection = truckEntities[0].position._value.clone();
    Cesium.Cartesian3.normalize(upDirection, upDirection);
    const verticalComponent = Cesium.Cartesian3.dot(vehicleDirection, upDirection);
    const truckAngle = Math.atan2(verticalComponent, 1);
    console.log(truckAngle);
    console.log(viewer.camera.pitch); // top down is -PI/2, look up is PI/2, horizon is 0
    const desiredCameraPitch = truckAngle;
    
    
    
//     const vehicleUp = new Cesium.Cartesian3(0, 0, 1);
//     Cesium.Matrix3.multiplyByVector(matrix3, vehicleUp, vehicleUp);
// //     const cameraUp = new Cesium.Cartesian3(0, 0, 1);
// //     viewer.camera.cameraToWorldCoordinatesVector(cameraUp, cameraUp);
//     const angle = Cesium.Cartesian3.angleBetween(upDirection, vehicleUp) * 180 / Math.PI;
//     if (angle > 90) { angle = 0; }
//     console.log(angle);
//     const crossProduct2 = new Cesium.Cartesian3();
//     Cesium.Cartesian3.cross(vehicleUp, upDirection, crossProduct2);
//     const dotProductRight = Cesium.Cartesian3.dot(viewer.camera.rightWC, crossProduct2);
//     viewer.camera.rotateUp(dotProductRight * Math.PI / 128);
    
  }

  adjustHeightForTerrain(viewer.scene.screenSpaceCameraController);

  if (viewer.trackedEntity != truckEntities[0]) {
    // https://sandcastle.cesium.com/?src=Parallels%20and%20Meridians.html&label=All
    const centerScreen = new Cesium.Cartesian2(
      viewer.canvas.width / 2, viewer.canvas.height / 2);
    const ray = viewer.camera.getPickRay(centerScreen);
    const cartesian = viewer.scene.globe.pick(ray, viewer.scene);
    if (Cesium.defined(cartesian)) {
      const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
      cartographic.height += 1;
      Cesium.Cartographic.toCartesian(
        cartographic, viewer.camera.ellipsoid, truckEntities[0].position._value);
      const headingPitchRoll = new Cesium.HeadingPitchRoll(
        viewer.camera.heading + Math.PI / 2, 0, 0);
      const fixedFrameTransform = Cesium.Transforms.localFrameToFixedFrameGenerator(
        "north", "west");
      Cesium.Transforms.headingPitchRollQuaternion(
        truckEntities[0].position._value,
        headingPitchRoll,
        Cesium.Ellipsoid.WGS84,
        fixedFrameTransform,
        truckEntities[0].orientation._value
      );
    }
  }

}

let start, previousTimeStamp;
function animate(timestamp) {
  if (start === undefined) {start = timestamp;}
  const delta = (timestamp - previousTimeStamp) / 1000;
  previousTimeStamp = timestamp;

  update();
  if (waitingForPhysicsInit == false) {
    physics.update(delta);
  }

  window.requestAnimationFrame(animate);
}

let waitingForPhysicsInit = true;
Ammo().then(function (AmmoLib) {
  Ammo = AmmoLib;
  setTimeout(function() {
    physics.init(truckEntities, viewer);
    waitingForPhysicsInit = false;
  }, 1000);
  animate();
});
