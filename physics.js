// https://github.com/kripken/ammo.js/blob/main/examples/webgl_demo_vehicle/index.html

import * as extrapolation from './extrapolation.js';
const framesBetweenExtrapolationFit = 10;

const quadtreeLevel = 22;
const quadtreePower = Math.pow(2, quadtreeLevel);
const quadtreeGridWidth = 8;
const quadtreeGridHeight = 6;
const showQuadtreeGrid = false;

let viewer;

let truckEntities;
const terrainBodies = [];
let originOffset;

let vehicle;

// - Global variables -
const DISABLE_DEACTIVATION = 4;

let gravityOn = false;
const gravity = 9.82;

const resetTorque = 200000;
const resetDamping = 1;
const tippingAcceleration = gravity * 25;
const framesToFullSteer = 40;

let speedometer;
const limiter = 85;//mph

// Physics variables
let collisionConfiguration;
let dispatcher;
let broadphase;
let solver;
let physicsWorld;

const syncList = [];

// Keybord actions
const actions = {};
const keysActions = {
  "KeyW":'acceleration',
  "KeyS":'braking',
  "KeyA":'left',
  "KeyD":'right',
  "KeyR":'reset'
};

let parkingBrake = false;
let hardReset = false;

export function init(newTruck, newViewer) {
  truckEntities = newTruck;
  viewer = newViewer;

  speedometer = document.getElementById( 'speedometer' );

  // Physics configuration
  collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  dispatcher = new Ammo.btCollisionDispatcher( collisionConfiguration );
  broadphase = new Ammo.btDbvtBroadphase();
  solver = new Ammo.btSequentialImpulseConstraintSolver();
  physicsWorld = new Ammo.btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );

  createObjects();

  window.addEventListener( 'keydown', keydown);
  window.addEventListener( 'keyup', keyup);
  
}

let frameCount = 0;
let previousTruckSelected = false;
export function update(delta) {
  const truckSelected = viewer.trackedEntity == truckEntities[0];
  if (truckSelected == false) {
    resetOriginOffset();
    
    const position = truckEntities[0].position._value;
    const offset = new Cesium.Cartesian3();
    Cesium.Cartesian3.subtract(position, originOffset, offset);
    const btPosition = new Ammo.btVector3(offset.x, offset.y, offset.z);
    const tm = vehicle.getChassisWorldTransform();
    tm.setOrigin(btPosition);

    const orient = truckEntities[0].orientation._value.clone();
    const quatB = new Cesium.Quaternion(0, 0, 0, 1);
    Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_X, Math.PI / 2, quatB);
    Cesium.Quaternion.multiply(orient, quatB, orient);
    Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_Y, Math.PI, quatB);
    Cesium.Quaternion.multiply(orient, quatB, orient);
    tm.setRotation(new Ammo.btQuaternion(orient.x, orient.y, orient.z, orient.w));

    const body = vehicle.getRigidBody();
    body.setLinearVelocity(new Ammo.btVector3(0, 0, 0));
    body.setAngularVelocity(new Ammo.btVector3(0, 0, 0));
    
    resetWhitelist();
    cleanUpTerrain();
  }
  
//   if (truckSelected) {
  if (true) {
    if (gravityOn) {
      const position = truckEntities[0].position.getValue(truckEntities.now());
      const normal = new Ammo.btVector3(position.x, position.y, position.z);
      normal.normalize();
      normal.op_mul(-gravity);
      physicsWorld.setGravity( normal );
    } else {
      physicsWorld.setGravity( new Ammo.btVector3(0, 0, 0) );
    }

    for (let i = 0; i < syncList.length; i++) { syncList[i](delta); }
    physicsWorld.stepSimulation(delta, 10);
  }
  previousTruckSelected = truckSelected;
  
  const deadSeaElevation = -430.5;
  const position = truckEntities[0].position.getValue(truckEntities.now());
  const terrainProvider = viewer.scene.globe.terrainProvider;
  const ellipsoid = terrainProvider.tilingScheme.projection.ellipsoid;
  const cartographic = Cesium.Cartographic.fromCartesian(position, ellipsoid);
  if (cartographic.height < deadSeaElevation && truckSelected == true) {
    hardReset = true;
//     console.log('hardReset');
    viewer.trackedEntity = null;
  } else if(hardReset) {
    hardReset = false;
    viewer.trackedEntity = truckEntities[0];
  }
  
  frameCount++;
  if (frameCount % framesBetweenExtrapolationFit == 0) {
    const points = [];
    for (let i = 0; i < terrainBodies.length; i++) {
      if (terrainBodies[i].isResolved) {
        for (let j = 0; j < 4; j++) {
          const data = terrainBodies[i].retainedData;
          points.push([data[j].longitude, data[j].latitude, data[j].height]);
        }
      }
    }
    extrapolation.fitHeightPlane(points);
  }
}

function keyup(e) {
  if(keysActions[e.code]) {
    actions[keysActions[e.code]] = false;
    e.preventDefault();
    e.stopPropagation();
    return false;
  }
}

function keydown(e) {
  if(keysActions[e.code]) {
    actions[keysActions[e.code]] = true;
    e.preventDefault();
    e.stopPropagation();
    return false;
  }
}

function createObjects() {
  const position = truckEntities[0].position.getValue(truckEntities.now());
  const quaternion = truckEntities[0].orientation.getValue(truckEntities.now());
  createVehicle(position, quaternion);
}

function addPoint(cartesian3) {
  return viewer.entities.add({
    position: cartesian3,
    point: {
      pixelSize: 1,
      color: Cesium.Color.WHITE,
    },
  });
}

function resetOriginOffset() {
  const position = truckEntities[0].position.getValue(truckEntities.now());
  originOffset = new Cesium.Cartesian3(position.x, position.y, position.z);
}

function createVehicle(pos, quat) {

  // Vehicle contants

  const massVehicle = 3787.5; // 800;

  const wheelAxisPositionBack = -2.07;
  const wheelRadiusBack = 0.35;
  const wheelWidthBack = 0.245;
  const wheelHalfTrackBack = 0.8;
  const wheelAxisHeightBack = 0.3;

  const wheelAxisFrontPosition = 1.46;
  const wheelHalfTrackFront = 0.8;
  const wheelAxisHeightFront = 0.3;
  const wheelRadiusFront = 0.35;
  const wheelWidthFront = 0.245;

  const friction = 1000;
  const suspensionStiffness = 20; // 94.7; // 20.0;
  const suspensionDamping = 2.3; // 10.9; // 2.3;
  const suspensionCompression = 4.4; // 20.8; // 4.4;
  const suspensionRestLength = 0.8;
  const rollInfluence = 0.2;

  const steeringIncrement = .1;
  let steeringClamp = Math.PI/6;
  const maxEngineForce = 9468; // 2000;
  const maxBreakingForce = 236; // 50;
  
  // Chassis
  const geometry = new Ammo.btConvexHullShape();
  
  const hoodLength = 2;
  const tailLength = 3.25;
  const hoodBedHeight = .45;
  const bottomHeight = .25;
  const cabHeight = .925;
  const cabLengthFront = .4;
  const cabLengthBack = .85;
  const halfTruckWidth = .85;
  const halfCabWidth = .65;
  
  geometry.addPoint(new Ammo.btVector3( halfTruckWidth,  hoodBedHeight,  hoodLength));
  geometry.addPoint(new Ammo.btVector3(-halfTruckWidth,  hoodBedHeight,  hoodLength));
  geometry.addPoint(new Ammo.btVector3( halfTruckWidth, -bottomHeight,   hoodLength));
  geometry.addPoint(new Ammo.btVector3(-halfTruckWidth, -bottomHeight,   hoodLength));
  geometry.addPoint(new Ammo.btVector3( halfTruckWidth,  hoodBedHeight, -tailLength));
  geometry.addPoint(new Ammo.btVector3(-halfTruckWidth,  hoodBedHeight, -tailLength));
  geometry.addPoint(new Ammo.btVector3( halfTruckWidth, -bottomHeight,  -tailLength));
  geometry.addPoint(new Ammo.btVector3(-halfTruckWidth, -bottomHeight,  -tailLength));
  
  geometry.addPoint(new Ammo.btVector3( halfCabWidth, cabHeight,  cabLengthFront));
  geometry.addPoint(new Ammo.btVector3(-halfCabWidth, cabHeight,  cabLengthFront));
  geometry.addPoint(new Ammo.btVector3( halfCabWidth, cabHeight, -cabLengthBack));
  geometry.addPoint(new Ammo.btVector3(-halfCabWidth, cabHeight, -cabLengthBack));
  
  const localInertia = new Ammo.btVector3(0, 0, 0);
  geometry.calculateLocalInertia(massVehicle, localInertia);
  
  const transform = new Ammo.btTransform();
  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3(0, 0, 0));
  resetOriginOffset();

  const quatB = new Cesium.Quaternion(0, 0, 0, 1);
  Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_X, Math.PI / 2, quatB);
  Cesium.Quaternion.multiply(quat, quatB, quat);
  Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_Y, Math.PI, quatB);
  Cesium.Quaternion.multiply(quat, quatB, quat);
  transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
  const motionState = new Ammo.btDefaultMotionState(transform);

  const body = new Ammo.btRigidBody(new Ammo.btRigidBodyConstructionInfo(massVehicle, motionState, geometry, localInertia));
  body.setActivationState(DISABLE_DEACTIVATION);
  physicsWorld.addRigidBody(body);
  
  // Raycast Vehicle
  let engineForce = 0;
  let vehicleSteering = 0;
  let breakingForce = 0;
  const tuning = new Ammo.btVehicleTuning();
  const rayCaster = new Ammo.btDefaultVehicleRaycaster(physicsWorld);
  vehicle = new Ammo.btRaycastVehicle(tuning, body, rayCaster);
  vehicle.setCoordinateSystem(0, 1, 2);
  physicsWorld.addAction(vehicle);
  
  // Wheels
  const FRONT_LEFT = 0;
  const FRONT_RIGHT = 1;
  const BACK_LEFT = 2;
  const BACK_RIGHT = 3;
  const wheelMeshes = [];
  const wheelDirectionCS0 = new Ammo.btVector3(0, -1, 0);
  const wheelAxleCS = new Ammo.btVector3(-1, 0, 0);
  
  function addWheel(isFront, pos, radius, width, index) {
    const wheelInfo = vehicle.addWheel(
      pos,
      wheelDirectionCS0,
      wheelAxleCS,
      suspensionRestLength,
      radius,
      tuning,
      isFront);
    wheelInfo.set_m_suspensionStiffness(suspensionStiffness);
    wheelInfo.set_m_wheelsDampingRelaxation(suspensionDamping);
    wheelInfo.set_m_wheelsDampingCompression(suspensionCompression);
    wheelInfo.set_m_frictionSlip(friction);
    wheelInfo.set_m_rollInfluence(rollInfluence);
    wheelInfo.set_m_maxSuspensionForce(1000000);
  }
  
  addWheel(true, new Ammo.btVector3(wheelHalfTrackFront, wheelAxisHeightFront, wheelAxisFrontPosition), wheelRadiusFront, wheelWidthFront, FRONT_LEFT);
  addWheel(true, new Ammo.btVector3(-wheelHalfTrackFront, wheelAxisHeightFront, wheelAxisFrontPosition), wheelRadiusFront, wheelWidthFront, FRONT_RIGHT);
  addWheel(false, new Ammo.btVector3(-wheelHalfTrackBack, wheelAxisHeightBack, wheelAxisPositionBack), wheelRadiusBack, wheelWidthBack, BACK_LEFT);
  addWheel(false, new Ammo.btVector3(wheelHalfTrackBack, wheelAxisHeightBack, wheelAxisPositionBack), wheelRadiusBack, wheelWidthBack, BACK_RIGHT);

  // Sync keybord actions and physics and graphics
  function sync(dt) {
    const speed = vehicle.getCurrentSpeedKmHour() * 0.621371;
    speedometer.innerHTML = Math.abs(speed).toFixed(0) + ' mph';
    
    breakingForce = 0;
    engineForce = 0;
    if (actions.acceleration && speed < limiter) {
      parkingBrake = false;
      if (speed < -1) breakingForce = maxBreakingForce; else engineForce = maxEngineForce;
    } else if (actions.braking && speed > -limiter) {
      parkingBrake = false;
      if (speed > 1) breakingForce = maxBreakingForce; else engineForce = -maxEngineForce;
    } else if (Math.abs(speed) < 1 || parkingBrake) {
      breakingForce = maxBreakingForce;
      parkingBrake = true;
    }
    
    steeringClamp = Math.asin( 3.5 * tippingAcceleration / Math.abs(vehicle.getCurrentSpeedKmHour()) ** 2 );
    if (steeringClamp > Math.PI/6 || isNaN(steeringClamp)) { steeringClamp = Math.PI/6; }
    const steeringSpeed = steeringClamp / framesToFullSteer;
    let notSteering = true;
    if (actions.left && vehicleSteering < steeringClamp) {
      vehicleSteering += steeringSpeed;
      notSteering = false;
    }
    if (actions.right && vehicleSteering > -steeringClamp) {
      vehicleSteering -= steeringSpeed;
      notSteering = false;
    }
    if (vehicleSteering < -steeringClamp) { vehicleSteering = -steeringClamp; }
    if (vehicleSteering > steeringClamp) { vehicleSteering = steeringClamp; }
    if (notSteering) {
      if (vehicleSteering > steeringSpeed) { vehicleSteering -= steeringSpeed; }
      if (vehicleSteering < -steeringSpeed) { vehicleSteering += steeringSpeed; }
    }
    
    vehicle.applyEngineForce(engineForce, BACK_LEFT);
    vehicle.applyEngineForce(engineForce, BACK_RIGHT);

    vehicle.setBrake(breakingForce / 2, FRONT_LEFT);
    vehicle.setBrake(breakingForce / 2, FRONT_RIGHT);
    vehicle.setBrake(breakingForce, BACK_LEFT);
    vehicle.setBrake(breakingForce, BACK_RIGHT);

    vehicle.setSteeringValue(vehicleSteering, FRONT_LEFT);
    vehicle.setSteeringValue(vehicleSteering, FRONT_RIGHT);
    
    let tm, p, q, v, i;
    const n = vehicle.getNumWheels();
    for (i = 0; i < n; i++) {
      vehicle.updateWheelTransform(i, true);
      tm = vehicle.getWheelTransformWS(i);
      p = tm.getOrigin();
      q = tm.getRotation();
      
      const position = new Cesium.Cartesian3(p.x(), p.y(), p.z());
      Cesium.Cartesian3.add(position, originOffset, position);
      truckEntities[i + 1].position = position;

      const quaternion = new Cesium.Quaternion(q.x(), q.y(), q.z(), q.w());
      if (i == 0 || i == 3) {
        const quaternionB = new Cesium.Quaternion(0, 0, 0, 1);
        Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_Y, Math.PI, quaternionB);
        Cesium.Quaternion.multiply(quaternion, quaternionB, quaternion);
      }
      truckEntities[i + 1].orientation = quaternion;
    }
    
    tm = vehicle.getChassisWorldTransform();
    p = tm.getOrigin();
    q = tm.getRotation();
    v = body.getLinearVelocity();
    
    let position = new Cesium.Cartesian3(p.x(), p.y(), p.z());
    const velocity = new Cesium.Cartesian3(v.x(), v.y(), v.z());
    Cesium.Cartesian3.multiplyByScalar(velocity, 0.013, velocity);
    Cesium.Cartesian3.subtract(position, velocity, position);
    Cesium.Cartesian3.add(position, originOffset, position);
    truckEntities[0].position = position;
    
    let quaternion = new Cesium.Quaternion(q.x(), q.y(), q.z(), q.w());
    const quaternionB = new Cesium.Quaternion(0, 0, 0, 1);
    Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_X, -Math.PI / 2, quaternionB);
    Cesium.Quaternion.multiply(quaternion, quaternionB, quaternion);
    Cesium.Quaternion.fromAxisAngle(Cesium.Cartesian3.UNIT_Z, Math.PI, quaternionB);
    Cesium.Quaternion.multiply(quaternion, quaternionB, quaternion);
    truckEntities[0].orientation = quaternion;
    
    if (actions.reset && gravityOn) {
      
      let aboveVehicle = new Cesium.Cartesian3(0, 1, 0);
      quaternion = new Cesium.Quaternion(q.x(), q.y(), q.z(), q.w());
      const matrix3 = new Cesium.Matrix3();
      Cesium.Matrix3.fromQuaternion(quaternion, matrix3);
      Cesium.Matrix3.multiplyByVector(matrix3, aboveVehicle, aboveVehicle);
      const truckPosition = truckEntities[0].position._value.clone();
      Cesium.Cartesian3.normalize(position, position);
      const crossProduct = new Cesium.Cartesian3();
      Cesium.Cartesian3.cross(aboveVehicle, position, crossProduct);
      let resetTorqueValue = Cesium.Cartesian3.angleBetween(aboveVehicle, position) * resetTorque / (Math.PI / 4);
      if (resetTorqueValue > resetTorque) { resetTorqueValue = resetTorqueValue; }
      Cesium.Cartesian3.normalize(crossProduct, crossProduct);
      Cesium.Cartesian3.multiplyByScalar(crossProduct, resetTorqueValue, crossProduct);
      
      const restoreTorque = new Ammo.btVector3(crossProduct.x, crossProduct.y, crossProduct.z);
      body.applyTorque(restoreTorque);
      Ammo.destroy(restoreTorque);
      
      body.setDamping(0, resetDamping);
    } else {
      body.setDamping(0, 0);
    }
    
    if (viewer.trackedEntity == truckEntities[0]) {
      
      resetWhitelist();
      
      position = new Cesium.Cartesian3(p.x(), p.y(), p.z());
      Cesium.Cartesian3.add(position, originOffset, position);
      
      const terrainProvider = viewer.scene.globe.terrainProvider;
      const ellipsoid = terrainProvider.tilingScheme.projection.ellipsoid;
      const cartographic = Cesium.Cartographic.fromCartesian(position, ellipsoid);
      
      const longitudeIndex = ( cartographic.longitude - ( -Math.PI ) ) * quadtreePower;
      const latitudeIndex = ( cartographic.latitude - ( -Math.PI / 2 ) ) * quadtreePower;
      
      for (let m = -quadtreeGridWidth / 2; m <= quadtreeGridWidth / 2; m++) {
        for (let n = -quadtreeGridHeight / 2; n <= quadtreeGridHeight / 2; n++) {
          const indexM = Math.floor(longitudeIndex + m);
          const indexN = Math.floor(latitudeIndex + n);
          
          tryToCreateTerrain(indexM, indexN);
        }
      }
      
      cleanUpTerrain();
    }
    
  }
  
  syncList.push(sync);

}

let destroyableTerrainCounter = 0;
class DestroyableTerrain {
  constructor(lon, lat) {
    this.quadtreeGridPoints = [];
    this.longitudeIndex = lon;
    this.latitudeIndex = lat;
    this.whitelist = true;
    
    const positions = [];
    for (let m = 0; m <= 1; m++) {
      for (let n = 0; n <= 1; n++) {
        const indexM = Math.floor(this.longitudeIndex + m);
        const indexN = Math.floor(this.latitudeIndex + n);
        const longitudeM = indexM / quadtreePower + ( -Math.PI );
        const latitudeN = indexN / quadtreePower + ( -Math.PI / 2 );
        const predictedHeight = extrapolation.extrapolate(longitudeM, latitudeN);
        const cartographicMN = new Cesium.Cartographic(longitudeM, latitudeN, predictedHeight);
        positions.push(cartographicMN);
      }
    }
    
    this.makeTerrain(positions);
    
    const terrainProvider = viewer.scene.globe.terrainProvider;
    const promise = Cesium.sampleTerrainMostDetailed(terrainProvider, positions);
    this.isResolved = false;
    this.loadStarted = performance.now();
    const thisTerrain = this;
    Promise.resolve(promise).then(function(updatedPositions) {
      thisTerrain.loadEnded = performance.now()
      console.log(`Call to doSomething took ${thisTerrain.loadEnded - thisTerrain.loadStarted} milliseconds`)
      thisTerrain.retainedData = updatedPositions;
      thisTerrain.isResolved = true;
      
      thisTerrain.destroy();
      thisTerrain.makeTerrain(updatedPositions);
      
      gravityOn = true;
    }).catch(error => { throw error })
    
  }
  
  makeTerrain(positions) {
    const terrainProvider = viewer.scene.globe.terrainProvider;
    const ellipsoid = terrainProvider.tilingScheme.projection.ellipsoid;
    this.shape = new Ammo.btConvexHullShape();
    this.vertices = new Array(8);
    for (let i = 0; i < positions.length; i++) {
      const cartesian3 = Cesium.Cartographic.toCartesian(positions[i], ellipsoid);
      const skirtHeight = 1;
      const cartographicSkirt = new Cesium.Cartographic(positions[i].longitude, positions[i].latitude, positions[i].height - skirtHeight);
      const skirtCartesian3 = Cesium.Cartographic.toCartesian(cartographicSkirt, ellipsoid);
      if (showQuadtreeGrid) {
        this.quadtreeGridPoints.push(addPoint(cartesian3));
        this.quadtreeGridPoints.push(addPoint(skirtCartesian3));
      }
      this.vertices[i * 2] = cartesian3;
      this.vertices[i * 2 + 1] = skirtCartesian3;
    }
    for (let i = 0; i < this.vertices.length; i++) {
      Cesium.Cartesian3.subtract(this.vertices[i], originOffset, this.vertices[i]);
      this.vertices[i] = new Ammo.btVector3(this.vertices[i].x, this.vertices[i].y, this.vertices[i].z);
      this.shape.addPoint(this.vertices[i]);
    }
    const transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(new Ammo.btVector3(0, 0, 0));
    transform.setRotation(new Ammo.btQuaternion(0, 0, 0, 1));
    this.motionState = new Ammo.btDefaultMotionState(transform);
    Ammo.destroy(transform);
    this.localInertia = new Ammo.btVector3(0, 0, 0);

    const rbInfo = new Ammo.btRigidBodyConstructionInfo(0, this.motionState, this.shape, this.localInertia);
    this.terrainBody = new Ammo.btRigidBody(rbInfo);
    Ammo.destroy(rbInfo);

    physicsWorld.addRigidBody(this.terrainBody);
  }

  destroy() {
    for (let i = 0; i < this.vertices.length; i++) {
      Ammo.destroy(this.vertices[i]);
    }
    delete this.vertices;
    Ammo.destroy(this.shape);
    delete this.shape;

    physicsWorld.removeRigidBody(this.terrainBody);

    Ammo.destroy(this.motionState);
    delete this.motionState;
    Ammo.destroy(this.localInertia);
    delete this.localInertia;
    Ammo.destroy(this.terrainBody);
    delete this.terrainBody;

    if (showQuadtreeGrid) {
      for (let i = 0; i < this.quadtreeGridPoints.length; i++) {
        viewer.entities.remove(this.quadtreeGridPoints[i]);
      }
      this.quadtreeGridPoints = [];
    }
  }

}

function createTerrain(lon, lat) {
  terrainBodies.push(new DestroyableTerrain(lon, lat));
}

function tryToCreateTerrain(lon, lat) {
  let alreadyCreated = false;
  for (let i = 0; i < terrainBodies.length; i++) {
    if (terrainBodies[i].longitudeIndex == lon && terrainBodies[i].latitudeIndex == lat) {
      alreadyCreated = true;
      terrainBodies[i].whitelist = true;
      break;
    }
  }
  if (alreadyCreated == false) {
    createTerrain(lon, lat);
  }
}

function resetWhitelist() {
  for (let i = 0; i < terrainBodies.length; i++) {
    terrainBodies[i].whitelist = false;
  }
}

function cleanUpTerrain() {
  for (let i = terrainBodies.length - 1; i >= 0; i--) {
    if (terrainBodies[i].whitelist == false && terrainBodies[i].isResolved == true) {
      terrainBodies[i].destroy();
      delete terrainBodies[i];
      terrainBodies.splice(i, 1);
    }
  }
}
