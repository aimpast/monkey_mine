import bRigid.*;
import com.bulletphysics.collision.dispatch.GhostObject;

class Mine {
  public float x;
  public float y;
  public BObject     obj;
  public GhostObject ghost;
  
  Mine(float x,float y,BObject obj,GhostObject ghost) {
    this.x = x;
    this.y = y;
    this.obj = obj;
    this.ghost = ghost;
  }
}
