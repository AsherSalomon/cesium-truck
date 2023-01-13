// https://github.com/kripken/ammo.js/blob/main/examples/webgl_demo_vehicle/index.html

let once = true;

const quadtreeLevel = 22;
const quadtreePower = Math.pow(2, quadtreeLevel);
const quadtreeGridWidth = 8;
const showQuadtreeGrid = true;

let viewer;

let truckEntities;
const terrainBodies = [];
let originOffset;

let vehicle;

// - Global variables -
const DISABLE_DEACTIVATION = 4;

let gravityOn = false;
const gravity = 9.82;

let speedometer;

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
  if (truckSelected != previousTruckSelected) {
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
    
    once = true;
  }
  
  if (truckSelected) {
    if (gravityOn) {
      const position = truckEntities[0].position.getValue(truckEntities.now());
      const normal = new Ammo.btVector3(position.x, position.y, position.z);
      normal.normalize();
      normal.op_mul(-gravity);
      physicsWorld.setGravity( normal );
    } else {
      physicsWorld.setGravity( new Ammo.btVector3(0, 0, 0) );
    }

    frameCount++;
    if (frameCount % 1 == 0) {
      for (let i = 0; i < syncList.length; i++) { syncList[i](delta); }
      physicsWorld.stepSimulation(delta, 10);
    }
  }
  previousTruckSelected = truckSelected;
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

  const chassisWidth = 2.032;
  const chassisHeight = .8;
  const chassisLength = 6.761 * 0.8;
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

  const steeringIncrement = .2;
  const steeringClamp = .5;
  const maxEngineForce = 9468; // 2000;
  const maxBreakingForce = 236; // 50;
  
  // Chassis
  const geometry = new Ammo.btBoxShape(new Ammo.btVector3(chassisWidth * .5, chassisHeight * .5, chassisLength * .5));
  
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
    const speed = vehicle.getCurrentSpeedKmHour();
    speedometer.innerHTML = Math.abs(speed * 0.621371).toFixed(0) + ' mph';
    
    breakingForce = 0;
    engineForce = 0;
    if (actions.acceleration) {
      parkingBrake = false;
      if (speed < -1) breakingForce = maxBreakingForce; else engineForce = maxEngineForce;
    } else if (actions.braking) {
      parkingBrake = false;
      if (speed > 1) breakingForce = maxBreakingForce; else engineForce = -maxEngineForce;
    } else if (Math.abs(speed) < 1 || parkingBrake) {
      breakingForce = maxBreakingForce;
      parkingBrake = true;
    }
    
    const steeringSpeed = steeringIncrement * dt / 0.0167 / Math.max(Math.abs(speed), 10);
    if (actions.left) {
      if (vehicleSteering < steeringClamp)
        vehicleSteering += steeringSpeed;
    } else {
      if (actions.right) {
        if (vehicleSteering > -steeringClamp)
          vehicleSteering -= steeringSpeed;
      } else {
        if (vehicleSteering < -steeringSpeed)
          vehicleSteering += steeringSpeed;
        else {
          if (vehicleSteering > steeringSpeed)
            vehicleSteering -= steeringSpeed;
          else {
            vehicleSteering = 0;
          }
        }
      }
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
      position = new Cesium.Cartesian3(p.x(), p.y(), p.z());
      quaternion = new Cesium.Quaternion(q.x(), q.y(), q.z(), q.w());
      const matrix3 = new Cesium.Matrix3();
      Cesium.Matrix3.fromQuaternion(quaternion, matrix3);
      Cesium.Matrix3.multiplyByVector(matrix3, aboveVehicle, aboveVehicle);
      aboveVehicle = new Ammo.btVector3(aboveVehicle.x, aboveVehicle.y, aboveVehicle.z);
      Cesium.Cartesian3.add(position, originOffset, position);
      Cesium.Cartesian3.normalize(position, position);
      const resetForce = massVehicle * gravity;
      Cesium.Cartesian3.multiplyByScalar(position, resetForce, position);
      position = new Ammo.btVector3(position.x, position.y, position.z);
      body.applyForce(position, aboveVehicle);
      Ammo.destroy(aboveVehicle);
      Ammo.destroy(position);
    }
    
    if ( once ) {
      once = false;
      
      resetWhitelist();
      
      position = new Cesium.Cartesian3(p.x(), p.y(), p.z());
      Cesium.Cartesian3.add(position, originOffset, position);
      
      const terrainProvider = viewer.scene.globe.terrainProvider;
      const ellipsoid = terrainProvider.tilingScheme.projection.ellipsoid;
      const cartographic = Cesium.Cartographic.fromCartesian(position, ellipsoid);
      
      const longitudeIndex = ( cartographic.longitude - ( -Math.PI ) ) * quadtreePower;
      const latitudeIndex = ( cartographic.latitude - ( -Math.PI / 2 ) ) * quadtreePower;
      
//       const positions = [];
//       const quadtreeNames = [];
      for (let m = -quadtreeGridWidth / 2; m <= quadtreeGridWidth / 2; m++) {
        for (let n = -quadtreeGridWidth / 2; n <= quadtreeGridWidth / 2; n++) {
          const indexM = Math.floor(longitudeIndex + m);
          const indexN = Math.floor(latitudeIndex + n);
          
          tryToCreateTerrain(indexM, indexN);
//           const longitudeM = indexM / quadtreePower + ( -Math.PI );
//           const latitudeN = indexN / quadtreePower + ( -Math.PI / 2 );
//           const cartographicMN = new Cesium.Cartographic(longitudeM, latitudeN, 0);
//           positions.push(cartographicMN);
//           quadtreeNames.push(indexM + '_' + indexN);
        }
      }
      
//       const promise = Cesium.sampleTerrainMostDetailed(terrainProvider, positions);
      
//       Promise.resolve(promise).then(function(updatedPositions) {
        
//         for (let i = 0; i < positions.length; i++) {
//           const skirtHeight = 1;
//           const cartographicSkirt = new Cesium.Cartographic(positions[i].longitude, positions[i].latitude, positions[i].height - skirtHeight);
//           const cartesian3 = Cesium.Cartographic.toCartesian(positions[i], ellipsoid);
//           const skirtCartesian3 = Cesium.Cartographic.toCartesian(cartographicSkirt, ellipsoid);
      
//           if (showQuadtreeGrid) {
//             addPoint(cartesian3);
//             addPoint(skirtCartesian3);
//           }
//         }
//       }).catch(error => { throw error })
      
      cleanUpTerrain();
    }
    
  }
  
  syncList.push(sync);

}

class DestroyableTerrain {
  constructor(lon, lat) {
    this.longitudeIndex = lon;
    this.latitudeIndex = lat;
    this.whitelist = false;
    
    const positions = [];
    for (let m = 0; m <= 1; m++) {
      for (let n = 0; n <= 1; n++) {
        const indexM = Math.floor(this.longitudeIndex + m);
        const indexN = Math.floor(this.latitudeIndex + n);
        const longitudeM = indexM / quadtreePower + ( -Math.PI );
        const latitudeN = indexN / quadtreePower + ( -Math.PI / 2 );
        const cartographicMN = new Cesium.Cartographic(longitudeM, latitudeN, 0);
        positions.push(cartographicMN);
      }
    }
    
    const terrainProvider = viewer.scene.globe.terrainProvider;
    const ellipsoid = terrainProvider.tilingScheme.projection.ellipsoid;
    const promise = Cesium.sampleTerrainMostDetailed(terrainProvider, positions);
    Promise.resolve(promise).then(function(updatedPositions) {
      for (let i = 0; i < positions.length; i++) {
        const cartesian3 = Cesium.Cartographic.toCartesian(positions[i], ellipsoid);
        const skirtHeight = 1;
        const cartographicSkirt = new Cesium.Cartographic(positions[i].longitude, positions[i].latitude, positions[i].height - skirtHeight);
        const skirtCartesian3 = Cesium.Cartographic.toCartesian(cartographicSkirt, ellipsoid);
        if (showQuadtreeGrid) {
          addPoint(cartesian3);
          addPoint(skirtCartesian3);
        }
      }
    }).catch(error => { throw error })
    
//     this.shapes = new Array(indices.length / 3);
//     this.vertices = new Array(positions.length);
//     this.skirtices = new Array(positions.length);
//     for (let i = 0; i < positions.length; i++) {
//       Cesium.Cartesian3.subtract(positions[i], originOffset, positions[i]);
//       this.vertices[i] = new Ammo.btVector3(positions[i].x, positions[i].y, positions[i].z);
//     }
//     const normal = originOffset.clone();
//     Cesium.Cartesian3.normalize(normal, normal);
//     Cesium.Cartesian3.multiplyByScalar(normal, skirtHeight, normal);
//     for (let i = 0; i < positions.length; i++) {
//       Cesium.Cartesian3.subtract(positions[i], normal, positions[i]);
//       this.skirtices[i] = new Ammo.btVector3(positions[i].x, positions[i].y, positions[i].z);
//     }
//     for (let i = 0; i < indices.length; i += 3) {
//       this.shapes[i / 3] = new Ammo.btConvexHullShape();
//       for (let j = 0; j < 3; j++) {
//         this.shapes[i / 3].addPoint(this.vertices[indices[i + j]]);
//         this.shapes[i / 3].addPoint(this.skirtices[indices[i + j]]);
//       }
//     }
//     const transform = new Ammo.btTransform();
//     transform.setIdentity();
//     transform.setOrigin(new Ammo.btVector3(0, 0, 0));
//     transform.setRotation(new Ammo.btQuaternion(0, 0, 0, 1));
//     this.motionState = new Ammo.btDefaultMotionState(transform);
//     Ammo.destroy(transform);
//     this.localInertia = new Ammo.btVector3(0, 0, 0);

//     this.terrainBodies = new Array(this.shapes.length);
//     for (let i = 0; i < this.shapes.length; i++) {
//       const rbInfo = new Ammo.btRigidBodyConstructionInfo(0, this.motionState, this.shapes[i], this.localInertia);
//       this.terrainBodies[i] = new Ammo.btRigidBody(rbInfo);
//       Ammo.destroy(rbInfo);

//       physicsWorld.addRigidBody(this.terrainBodies[i]);
//     }

  }

//   destroy() {
//     for (let i = 0; i < this.terrainBodies.length; i++) {
//       physicsWorld.removeRigidBody(this.terrainBodies[i]);
//     }

//     for (let i = 0; i < this.vertices.length; i++) {
//       Ammo.destroy(this.vertices[i]);
//       Ammo.destroy(this.skirtices[i]);
//     }
//     delete this.vertices;
//     delete this.skirtices;
//     for (let i = 0; i < this.shapes.length; i++) {
//       Ammo.destroy(this.shapes[i]);
//     }
//     delete this.shapes;
//     Ammo.destroy(this.motionState);
//     Ammo.destroy(this.localInertia);
//     delete this.motionState;
//     delete this.localInertia;
//     for (let i = 0; i < this.terrainBodies.length; i++) {
//       Ammo.destroy(this.terrainBodies[i]);
//     }
//     delete this.terrainBodies;
//   }

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

// function removeTerrain(lon, lat) {
//   for (let i = 0; i < terrainBodies.length; i++) {
//     if (terrainBodies[i].longitudeIndex == lon && terrainBodies[i].latitudeIndex == lat) {
//       terrainBodies[i].destroy();
//       delete terrainBodies[i];
//       terrainBodies.splice(i, 1);
//       break;
//     }
//   }
// }

function resetWhitelist() {
  for (let i = 0; i < terrainBodies.length; i++) {
    terrainBodies[i].whitelist = false;
  }
}

function cleanUpTerrain() {
  for (let i = terrainBodies.length - 1; i >= 0; i--) {
    if(terrainBodies[i].whitelist == false) {
      terrainBodies[i].destroy();
      delete terrainBodies[i];
      terrainBodies.splice(i, 1);
    }
  }
}
