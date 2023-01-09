// https://github.com/kripken/ammo.js/blob/main/examples/webgl_demo_vehicle/index.html

let viewer;

let truckEntities;
const terrainBodies = {};
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
    const position = truckEntities[0].position._value;
    const offset = new Cesium.Cartesian3();
    Cesium.Cartesian3.subtract(position, originOffset, offset);
    const btPosition = new Ammo.btVector3(offset.x, offset.y, offset.z);
    cons = vehicle.getChassisWorldTransform();
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

ion createObjects() {
  const position = truckEntities[0].position.getValue(truckEntities.now());
  const quaternion = truckEntities[0].orientation.getValue(truckEntities.now());
  createVehicle(position, quaternion);
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
  originOffset = new Cesium.Cartesian3(pos.x, pos.y, pos.z); // to do, reset originOffest each time the truck is placed somewhere on earth.

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
    
  }

}
