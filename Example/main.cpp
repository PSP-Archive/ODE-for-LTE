/* 
   This tutorials describes how to use ODE in LTE Game Engine.
   This example is based on the ODE Irrlicht example found at
   http://irrlicht.sourceforge.net/tut_ode.html
   
   
   Commands:
   - Press X to add a box
   - Press O to toggle simulation
*/

#include <engine.h>

#include <ode/ode.h>
#include "../common.h"

#define MAX_CONTACTS 10

using namespace engine;

dWorldID __dworld_id; // The ID of the World
dSpaceID __dspace_id; // The ID of the Space, used for collision detection
dJointGroupID __djoint_id; // The ID of the Joint Group

dxGeom* room; // The ODE geom id of the room

core::array<dxGeom*> boxList; // An array containing the list of the boxes
	

engineDevice *device = 0;

int SimulationRunning = 1;


//! Update the rotation and the translation of an ODE object
void updateGeomInWorld(dxGeom* geom)
{
	
	core::vector3df pos;
	core::vector3df rot;
				
	// Get the new position of the geometry
	dReal* ode_pos=(dReal*)dGeomGetPosition(geom);
	
	// Set the new 3d vector of the position
	pos.set((f32)ode_pos[0],(f32)ode_pos[1],(f32)ode_pos[2]);
	
	// Get the quaternion rotation
	dQuaternion result;
	dGeomGetQuaternion(geom, result);
	
	// Convert it to eulerangles
  core::quaternion(result[3], result[2], result[1], result[0]).toEuler(rot);
	rot*=core::GRAD_PI;
		
	scene::ISceneNode* node= (scene::ISceneNode*)dGeomGetData(geom);
				
	// Set the new rotation and translation of the scene node
	node->setRotation(rot);		
	node->setPosition(pos);	
	
}

// Create an ODE geometry by a Bounding Box
dxGeom* createGeomByBoundingBox(scene::ISceneNode* node)
{
	 dxGeom* geom;
	 dBodyID body;
	 dMass mass;
	 
	 if (!node)
	 	 return 0;
	 
// get the boundingbox
  core::aabbox3d<f32> box=node->getBoundingBox();
  	
  core::vector3df extend=box.getExtent();
  // get the position of the scenenode
  core::vector3df pos=node->getPosition();
  // build a box shaped geometry for ODE
  geom=dCreateBox(__dspace_id,(dReal)extend.X,(dReal)extend.Y,(dReal)extend.Z);
  // set the position of the ODE geom
  dGeomSetPosition(geom,pos.X,pos.Y,pos.Z);
  // set a pointer to our Bounceable, 
  // this will come in handy when we do more complicated collisions
  dGeomSetData(geom,(void*)node);
  // create a body for this object
  body=dBodyCreate(__dworld_id);
  // setup the mass
  dMassSetBox(&mass,5.0,(dReal)extend.X,(dReal)extend.Y,(dReal)extend.Z);
  // combine body and mass
  dBodySetMass(body,&mass);
  // add the body to the geom
  dGeomSetBody(geom,body);
  // set the bodys position (same as geom position)
  dBodySetPosition(body,pos.X,pos.Y,pos.Z);
  dBodySetData(body,(void*)node);
 
  return geom;
	
}

// Create an ODE Geometry by a Mesh
dxGeom* createGeomByMesh(engine::scene::IMesh* mesh, engine::scene::ISceneNode* node){

	dxGeom* geom;
  // do nothing if the mesh or node is NULL
  if(mesh==NULL || node==NULL) return 0; 
  	
   int VertexCount = 0,  IndexCount = 0;
   for (int x = 0; x < mesh->getMeshBufferCount(); x++)
   {
   	VertexCount += mesh->getMeshBuffer(x)->getVertexCount();
   	IndexCount  += mesh->getMeshBuffer(x)->getIndexCount();
   }
     
    // build structure for ode trimesh geom
   dVector3* vertices=new dVector3[VertexCount];
   int* indices=new int[IndexCount];

   int i = 0, j = 0, s = 0;
    
   for (int x = 0; x < mesh->getMeshBufferCount(); x++)
   {
			scene::IMeshBuffer *mb = mesh->getMeshBuffer(x);
			video::S3DVertex *vtx = (video::S3DVertex*)mb->getVertices();
      u16* idx = mb->getIndices();
			for (int y = 0 ; y < mb->getVertexCount(); y++)
			{
				vertices[i][0] = vtx[y].Pos.X;
				vertices[i][1] = vtx[y].Pos.Y;
				vertices[i][2] = vtx[y].Pos.Z;
				i++;
			}
		  for (int y = 0; y < mb->getIndexCount(); y++)
		  	indices[j++] = idx[y] + s;
	
		  s += mb->getVertexCount();
   }    
    

  core::vector3df pos=node->getPosition();
  // build the trimesh data
  dTriMeshDataID data=dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSimple(data,(dReal*)vertices, 
                      VertexCount, indices, IndexCount);
  // build the trimesh geom 
  geom=dCreateTriMesh(__dspace_id,data,0,0,0);
  // set the geom position 
  dGeomSetPosition(geom,pos.X,pos.Y,pos.Z);
  // lets have a pointer to our bounceable
  // we could need this in the collision callback
  dGeomSetData(geom,(void*)node); 
  // in our application we don't want geoms 
  // converted from meshes to have a body
  dGeomSetBody(geom,0); 
  
  return geom;
}


// This function is called when an object is near to make a collision
// with another object
void nearCollisionCallback(void* data, dGeomID o1, dGeomID o2){
	

  int i=0;
  dBodyID b1=dGeomGetBody(o1);
  dBodyID b2=dGeomGetBody(o2);
  if(b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))return;
  dContact contact[MAX_CONTACTS];
  for(i=0;i<MAX_CONTACTS;i++){
    contact[i].surface.mode=dContactBounce | dContactSoftCFM;
    contact[i].surface.mu=dInfinity;
    contact[i].surface.mu2=0;
    contact[i].surface.bounce=1e-5f;
    contact[i].surface.bounce_vel=1e-9f;
    contact[i].surface.soft_cfm=1e-6f;
  }
  int numc=dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact));
  if(numc>0){			
    for(i=0;i<numc;i++){		
      dJointID c=dJointCreateContact(__dworld_id,__djoint_id,&contact[i]);
      dJointAttach(c,b1,b2);
    }
  }

}

void doSimulation()
{
	
	// Simulate only if is enabled
	if (SimulationRunning) {

   dSpaceCollide(__dspace_id,0,&nearCollisionCallback);
   dWorldStepFast1(__dworld_id,0.1,1);
   dJointGroupEmpty(__djoint_id);

   // update the position of the boxes
   for (int x = 0; x < boxList.size(); x++) {
     updateGeomInWorld ( boxList[x] );
   }
  
  }
   
}

// Add a box
void addBox()
{
	dxGeom* flybox;
  scene::ISceneNode *node;
	scene::ISceneManager *smgr = device->getSceneManager();
  node = smgr->addTestSceneNode(50, 0, -1,core::vector3df(-50, 200, 20));
  
  node->setMaterialFlag(video::EMF_STATIC_MESH, true);
  	
  flybox = createGeomByBoundingBox(node);	
  boxList.push_back(flybox);	
	
}


class MyEventReceiver : public IEventReceiver
{
public:
	virtual bool OnEvent(SEvent event)
	{
	
	  /* A Box is created when CROSS is pressed */

		if (event.EventType == engine::EET_KEY_INPUT_EVENT&&
			event.KeyInput.PressedOnce)
		{
			switch(event.KeyInput.Key)
			{
			case KEY_CROSS:
				{
					addBox();
				}
			break;
			
			case KEY_CIRCLE:
				{
					
					SimulationRunning ^= 1;
					
			  }
				return true;
			}
		}

		return false;
	}
};

int main()
{

  setupPSP();


	MyEventReceiver receiver;
	
	device =	createDevice(&receiver, false);

	if (device == 0)
		return 1; // could not create selected driver.

	video::IVideoDriver* driver = device->getVideoDriver();
	scene::ISceneManager* smgr = device->getSceneManager();
		
	/* Create the World, Space and Joint ID */
	__dworld_id = dWorldCreate();
	__dspace_id = dSimpleSpaceCreate(0);
	__djoint_id = dJointGroupCreate(0);
	
	// Set the Gravity of the world
	 
	                             /*   +-- The Y gravity of our world 
	                                  |                                */
	dWorldSetGravity(__dworld_id,0,-75.0f,0);


	scene::IAnimatedMesh* mesh = smgr->getMesh(
		"ms0:/media/room.3ds");

	smgr->getMeshManipulator()->makePlanarTextureMapping(
		mesh->getMesh(0), 0.004f);

	scene::ISceneNode* node = 0;

	node = smgr->addAnimatedMeshSceneNode(mesh);
	node->setMaterialTexture(0,	driver->getTexture("ms0:/media/wall.jpg"));
	node->getMaterial(0).EmissiveColor.set(0,0,0,0);
	node->setMaterialFlag(video::EMF_LIGHTING, false);
	node->setMaterialFlag(video::EMF_CLIPPING, true);


  room = createGeomByMesh(mesh->getMesh(0), node);

  	

 /*
	Finally we simply have to draw everything, that's all.
	*/

	scene::ICameraSceneNode* camera = smgr->addCameraSceneNodeFPS();
	camera->setPosition(core::vector3df(-50,50,-150));

	// disable mouse cursor
	device->getCursorControl()->setVisible(false);

	while(device->run())
	{
		
		
		doSimulation();
		
		driver->beginScene(true, true, 0);

		smgr->drawAll();

		driver->endScene();

	}

	device->drop();

	return 0;
}

