
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.linearmath.QuaternionUtil; 
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.dynamics.RigidBody; 

import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.GhostPairCallback;

import peasy.*;
import bRigid.*;
import java.util.*;

PeasyCam cam;

BPhysics physics;

BBox box1;
BBox box2;
BJointHinge hinge;
BJointHinge hinge2;
BObject b1;

BObject test;

ArrayList<BObject> obj;

int[][] map = {
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1}
};

public void setup() {
  
  obj = new ArrayList<BObject>();
  size(640,480,P3D);
  frameRate(60);
  
  cam = new PeasyCam(this, 0,-200,0, 1000);
  
  Vector3f min = new Vector3f(-1200, -2500, -1200);
  Vector3f max = new Vector3f(1200, 2500, 1200);
  
  physics = new BPhysics(min, max);
  
  physics.world.setGravity(new Vector3f(0,500, 0));
  
  Vector3f pos1 = new Vector3f(0,0,0);
  Vector3f pos2 = new Vector3f(0,-60,0);
  Vector3f pos4 = new Vector3f(0, 0,0);
  
  box1 = new BBox(this,100,1000, 10,1000);
  box2 = new BBox(this,  1, 50, 50, 50);
  
  b1 = new BObject(this,100,box1,pos1,true);
  
  
  //Kinematic Object Settings
  Transform transform = new Transform();
  b1.rigidBody.getMotionState().getWorldTransform(transform);
  b1.rigidBody.setCollisionFlags(b1.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
  b1.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
  
  Vector3f pos3 = new Vector3f(0,0,0);
  BSphere sphere = new BSphere(this,1,20,pos3,true);
  sphere.setPosition(new Vector3f(50,-500,50));
  
  physics.addBody(b1);
  physics.addBody(sphere);

  for (int j=0;j<20;j++) {
    for (int i=0;i<20;i++) {
      if (map[j][i] == 0)continue;
      
      
      Vector3f pos5 = new Vector3f(50*j-475,-30f,50*i-475);
      BObject obstracle = new BObject(this,1,box2,pos5,true);
      physics.addBody(obstracle);
      
      Transform frameInA = new Transform();
      Transform frameInB = new Transform();
      frameInA.setIdentity();
      frameInB.setIdentity();
      
      frameInA.origin.x = pos5.x;
      frameInA.origin.z = pos5.z;
      frameInA.origin.y = -30f;
      
      Generic6DofConstraint Fixed;
      
      Fixed = new Generic6DofConstraint(b1.rigidBody,obstracle.rigidBody,frameInA,frameInB,true);
        
        
      Vector3f limit = new Vector3f(0f,0f,0f);
      Fixed.setAngularLowerLimit(limit);
      Fixed.setAngularUpperLimit(limit);
      Fixed.setLinearLowerLimit(new Vector3f(0f,0f,0f));
      Fixed.setLinearUpperLimit(new Vector3f(0f,0f,0f));
      physics.addJoint(Fixed);
      
    }
    
    
    
  }
  
  Vector3f position_test = new Vector3f(30,-10,0);
    test = new BObject(this,1,box2,position_test,true);
    
    Transform transform2 = new Transform();
    test.rigidBody.getMotionState().getWorldTransform(transform2);
    test.rigidBody.setCollisionFlags(test.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
    test.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    physics.addBody(test);
}  

float angle_x = .0f;
float angle_y = .0f;

public void draw() {
  
  background(0);
  lights();
  
  getkey();
  
  updateGhost();

  physics.update();
  physics.display();
}

public void getkey() {
  if(keyPressed) {
  if (key == CODED) {
    if (keyCode == UP) {
      
      
      angle_y -= .005f;
      //setRotation(b1.rigidBody,new Vector3f(0,0,1), angle_y);
    }
    else if (keyCode == DOWN) {
      angle_y += .005f;
      //setRotation(b1.rigidBody,new Vector3f(0,0,1), angle_y);
    }
    else if (keyCode == RIGHT) {
      angle_x -= .005f;
      //setRotation(b1.rigidBody,new Vector3f(1,0,0), angle_x);
    }
    else if (keyCode == LEFT) {
      angle_x += .005f;
      //setRotation(b1.rigidBody,new Vector3f(1,0,0), angle_x);
      
      
    }
  }
  
  if(angle_x > .5f) angle_x = .45f;
  if(angle_x < -.5f) angle_x = -.45f;
  if(angle_y > .5f) angle_y = .45f;
  if(angle_y < -.5f) angle_y = -.45f;
  
  
  //Rotation Matrix
  /*Matrix4f bius = new Matrix4f(
    1,0,0,0,
    0,1,0,-200,
    0,0,1,0,
    0,0,0,1
  );*/
  
  //Rotation Matrix  
  Matrix3f mat1 = new Matrix3f(
    1,0,0,
    0,cos(angle_y),sin(angle_y),
    0,-sin(angle_y),cos(angle_y)
  );
  
  Matrix3f mat2 = new Matrix3f(
    cos(angle_x),sin(angle_x),0,
    -sin(angle_x) ,cos(angle_x),0,
    0,0,1
    );
    
  mat1.mul(mat2);
  
  //Quat4f qout = new Quat4f();
  
  Transform t= new Transform(mat1);
  //Transform t = new Transform(bius);
  //t.setRotation(t2.getRotation(qout));
  b1.rigidBody.getMotionState().setWorldTransform(t);
  
  }
  
}

public void updateGhost() {
  
  float distance=100.f;
  float bius = 30.f;
  
  Vector3f pos_1 = new Vector3f(distance*cos(angle_x),distance*sin(angle_y),0);
  Vector3f pos_2 = new Vector3f(0,-distance*sin(angle_x),distance*cos(angle_y));
  
  
  Vector3f up1 = new Vector3f(pos_1.y,-pos_1.x,0);
  Vector3f up2 = new Vector3f(0,-pos_2.z,+pos_2.y);
  
  
  //System.out.println(up1.x+","+up1.y+","+up1.z);
  //System.out.println(up2.x+","+up2.y+","+up2.z);

  if(up1.lengthSquared() > 0)
    up1.normalize();
  if(up2.lengthSquared() > 0)
    up2.normalize();
  
 System.out.println(up1.x+","+up1.y+","+up1.z);
  System.out.println(up2.x+","+up2.y+","+up2.z);

  //Vector3f up = new Vector3f(0,0,0);
  
  Matrix4f mat1 = new Matrix4f(
    1,0,0,                      pos_1.x+pos_2.x+bius*(up1.x+up2.x),
    0,cos(angle_y),sin(angle_y),pos_1.y+pos_2.y+bius*(up1.y+up2.y),
    0,-sin(angle_y),cos(angle_y),pos_1.z+pos_2.z+bius*(up1.z+up2.z),
    0,0,0,1
  );
  
  Matrix4f mat2 = new Matrix4f(
    cos(angle_x),sin(angle_x),0,0,
    -sin(angle_x),cos(angle_x),0,0,
    0,0,1,0,
    0,0,0,1
  );
  
  mat1.mul(mat2);

  Transform t= new Transform(mat1);
  test.rigidBody.getMotionState().setWorldTransform(t);
}
