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
// Fly the camera to San Francisco at the given longitude, latitude, and height.
viewer.camera.flyTo({
  destination : Cesium.Cartesian3.fromDegrees(-122.4175, 37.655, 400),
  orientation : {
    heading : Cesium.Math.toRadians(0.0),
    pitch : Cesium.Math.toRadians(-15.0),
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

// let followTruck = false;
window.addEventListener('keydown', function(e) {
  // followTruck = true;
  if (e.keyCode == 69) {
    if (viewer.trackedEntity == truckEntities[0]) {
      viewer.trackedEntity = null;
    } else if (viewer.trackedEntity != truckEntities[0]) {
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

function update() {
	
  if (viewer.trackedEntity == truckEntities[0] && followTruck) {
    const vehicleDirection = new Cesium.Cartesian3(0, 1, 0);
    const quaternion = truckEntities[0].orientation.getValue(truckEntities.now());
    const matrix3 = new Cesium.Matrix3();
    Cesium.Matrix3.fromQuaternion(quaternion, matrix3);
    Cesium.Matrix3.multiplyByVector(matrix3, vehicleDirection, vehicleDirection);
    const crossProduct = new Cesium.Cartesian3();
    Cesium.Cartesian3.cross(viewer.camera.directionWC, vehicleDirection, crossProduct);
    const dotProduct = Cesium.Cartesian3.dot(viewer.camera.upWC, crossProduct);
    viewer.camera.rotateRight(dotProduct * Math.PI / 256);
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
    console.log('update');
  }

}

let start, previousTimeStamp;
function animate(timestamp) {
  if (start === undefined) {start = timestamp;}
	const delta = (timestamp - previousTimeStamp) / 1000;
  previousTimeStamp = timestamp;

  update();

  window.requestAnimationFrame(animate);
}
