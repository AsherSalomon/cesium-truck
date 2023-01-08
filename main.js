const truckEntities = [];

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

truckEntities[0] = viewer.entities.add({model: {uri: '1984_Ford_F350.glb'}});
for (let i = 1; i <= 4; i++) {
  truckEntities[i] = viewer.entities.add({model: {uri: '1984_Ford_F350_wheel.glb'}});
}

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
