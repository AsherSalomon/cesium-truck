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
