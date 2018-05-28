////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "rigtform.h"
#include "arcball.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include "sgutils.h"
#include "geometry.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------


static bool pickingMode = false;

///////////////////////////////////////
const bool g_Gl2Compatible = true;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

enum Mode {Robot1, Robot2, Skycam};

static Mode viewMode = Skycam; // 2 for skyCam, 0 for cube1, 1 for cube2
static bool skySkyMode = false;

static const int sphereSlices = 16;
static const int sphereStacks = 16;
static double g_arcballScale;
static int g_arcballScreenRadius;
static bool g_arcballScaleUpdate = true;
static float radius;
static RigTForm invEyeRbt;
static bool g_arcballMode = true;

static const double g_translationFactor = 0.01;


// --------- Materials
// This should replace all the contents in the Shaders section, e.g., g_numShaders, g_shaderFiles, and so on

static shared_ptr<Material> g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat;

shared_ptr<Material> g_overridingMaterial;


// --------- Geometry

typedef SgGeometryShapeNode MyShapeNode;





// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;
static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_light1Node, g_light2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking
static shared_ptr<SgRbtNode> *viewNodes[3] = {&g_robot1Node, &g_robot2Node, &g_skyNode};

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static RigTForm g_skyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
static RigTForm g_objectRbt[3] = {RigTForm(Cvec3(-1,0,0)), // currently only 1 obj is defined
                                 RigTForm(Cvec3(1,0,0)),
                               g_skyRbt};  // new one
static Cvec3f g_objectColors[2] = {Cvec3f(1, 0, 0), Cvec3f(0, 1, 0)};

// --------- For HW 5
static list<vector<RigTForm> > frames;
static list<vector<RigTForm> >::iterator currentFrame = frames.begin();
static int currentFrameIdx = -1;
static int rbtNodesSize = -1;
static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback
static bool animating = false;
static unsigned int g_clock = 0;

///////////////// END OF G L O B A L S //////////////////////////////////////////////////


static void initGround() {
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}



//////////////////////////


// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
}


// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

static void drawStuff(bool picking)  {


  // Declare an empty uniforms
  Uniforms uniforms;

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

  //===================================================================
  //const RigTForm eyeRbt = (g_objectRbt[viewMode]);
  RigTForm eyeRbt = getPathAccumRbt(g_world, *viewNodes[viewMode]);
  RigTForm invEyeRbt = inv(eyeRbt);
  //===================================================================
  invEyeRbt = inv(eyeRbt);

  Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
  Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(light2, 1)); // g_light2 position in eye coordinates
  // send the eye space coordinates of lights to uniforms
  uniforms.put("uLight", eyeLight1);
  uniforms.put("uLight2", eyeLight2);

  // draw ground
  // ===========
  //
  const RigTForm groundRbt = RigTForm();  // identity
  RigTForm MVM_rgt = invEyeRbt * groundRbt;
  Matrix4 MVM = rigTFormToMatrix(MVM_rgt);
  Matrix4 NMVM = normalMatrix(MVM);

  if (!picking) {
    Drawer drawer(invEyeRbt, uniforms);
    g_world->accept(drawer);
  //===============================================================================
    // draw sphere


    if(!(skySkyMode && viewMode == Skycam && *viewNodes[viewMode] == g_currentPickedRbtNode) &&
       !(*viewNodes[viewMode] == g_currentPickedRbtNode && viewMode != Skycam) &&
       !(viewMode != Skycam && g_currentPickedRbtNode == g_skyNode)){

      RigTForm arcBallCenter;
      if(g_currentPickedRbtNode == g_skyNode){
        arcBallCenter = RigTForm();
      }
      else arcBallCenter = getPathAccumRbt(g_world, g_currentPickedRbtNode);


      MVM_rgt = invEyeRbt * arcBallCenter;
      MVM = rigTFormToMatrix(MVM_rgt);

      if(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))
        g_arcballScaleUpdate = false;


      g_arcballScale = g_arcballScaleUpdate ? getScreenToEyeScale(MVM_rgt.getTranslation()[2], g_frustFovY, g_windowHeight) : g_arcballScale;

      radius = g_arcballScale * g_arcballScreenRadius;
      g_arcballScale = g_arcballScaleUpdate ? getScreenToEyeScale(MVM_rgt.getTranslation()[2], g_frustFovY, g_windowHeight) : g_arcballScale;
      MVM *= Matrix4::makeScale(Cvec3(radius, radius, radius));

      sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

      g_arcballMat->draw(*g_sphere, uniforms);


    }
    //cout << radius << endl;
    g_arcballScaleUpdate = true;

  }else{
    Picker picker(invEyeRbt, uniforms);

    // set overiding material to our picking material
    g_overridingMaterial = g_pickingMat;

    g_world->accept(picker);

    // unset the overriding material
    g_overridingMaterial.reset();

    glFlush();

    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);

    if (g_currentPickedRbtNode == g_groundNode || g_currentPickedRbtNode == NULL){
      g_currentPickedRbtNode = *viewNodes[viewMode];   // set to viewMode
    }
    cout << g_currentPickedRbtNode << endl;
    if(!(skySkyMode && viewMode == Skycam && *viewNodes[viewMode] == g_currentPickedRbtNode) &&
       !(*viewNodes[viewMode] == g_currentPickedRbtNode && viewMode != Skycam) &&
       !(viewMode != Skycam && g_currentPickedRbtNode == g_skyNode)){
         cout<< "arcball mode!" << endl;
         g_arcballMode = true;
       }else{
         g_arcballMode = false;
       }
  }
}


static void display() {
  // No more glUseProgram

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(false); // no more curSS

  glutSwapBuffers();

  checkGlErrors();
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // No more glUseProgram
  drawStuff(true); // no more curSS

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
  cerr << "Size of window is now " << w << "x" << h << ", radius: " << g_arcballScreenRadius << endl;
  updateFrustFovY();
  glutPostRedisplay();
}

static RigTForm arcballRotation(const int x1, const int y1, const int x2, const int y2){
  Cvec2 origin2;
  if(g_currentPickedRbtNode == *viewNodes[viewMode])
    origin2 = getScreenSpaceCoord(inv(getPathAccumRbt(g_world, g_skyNode)).getTranslation(), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
  else
    origin2 = getScreenSpaceCoord((inv(getPathAccumRbt(g_world, *viewNodes[viewMode])) * getPathAccumRbt(g_world, g_currentPickedRbtNode)).getTranslation(), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);

  Cvec3 origin = Cvec3(origin2, 0);
  //cout << origin[0] << " " << origin[1] << " " << origin[2] << endl;
  Cvec3 s1 = Cvec3((double)x1, (double)y1, 0);
  //cout << s1[0] << " " << s1[1] << " " << s1[2] << endl;
  Cvec3 s2 = Cvec3((double)x2, (double)y2, 0);
  //cout << s2[0] << " " << s2[1] << " " << s2[2] << endl;
  //cout << endl;
  Cvec3 v1 = s1 - origin;
  Cvec3 v2 = s2 - origin;

  if(norm(v1) >= g_arcballScreenRadius){
    v1 = v1.normalize();
    v1 *= g_arcballScreenRadius-CS175_EPS; // in order to avoid floating point approximation error
    //cout << "v1warning!!" << endl;
  }

  if(norm(v2) >= g_arcballScreenRadius){
    v2 = v2.normalize();
    v2 *= g_arcballScreenRadius-CS175_EPS; // in order to avoid floating point approximation error
    //cout << "v2warning!!" << endl;
  }


  v1[2] = sqrt(pow(g_arcballScreenRadius,2) - norm2(v1));
  v2[2] = sqrt(pow(g_arcballScreenRadius,2) - norm2(v2));
  v1 = v1.normalize();
  v2 = v2.normalize();

  Quat rot = Quat(dot(v1,v2), cross(v1,v2));
  return RigTForm(rot);
}

static void motion(const int x, const int y) {
  //cout << "x : " << g_mouseClickX << "->" << x << endl;
  //cout << "y : " << g_mouseClickY << "->" << g_windowHeight - y - 1 << endl << endl;

  RigTForm eyeMat = getPathAccumRbt(g_world, *viewNodes[viewMode]);
  RigTForm viewMat = inv(eyeMat);

  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  RigTForm m, a;
  if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
    if(!(skySkyMode && viewMode == Skycam && *viewNodes[viewMode] == g_currentPickedRbtNode) &&
       !(*viewNodes[viewMode] == g_currentPickedRbtNode && viewMode != Skycam) &&
       !(viewMode != Skycam && g_currentPickedRbtNode == g_skyNode)){
         m = arcballRotation(g_mouseClickX,g_mouseClickY,x, g_windowHeight - y -1);
    } else {
      m = RigTForm(Quat::makeXRotation(-dy)) * RigTForm(Quat::makeYRotation(dx));
    }
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
    if(!(skySkyMode && viewMode == Skycam && *viewNodes[viewMode] == g_currentPickedRbtNode) &&
       !(*viewNodes[viewMode] == g_currentPickedRbtNode && viewMode != Skycam) &&
       !(viewMode != Skycam && g_currentPickedRbtNode == g_skyNode)){
         m = RigTForm(Cvec3(dx, dy, 0) * g_arcballScale);
    } else {
      m = RigTForm(Cvec3(dx, dy, 0) * 0.01);
    }
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
    if(!(skySkyMode && viewMode == Skycam && *viewNodes[viewMode] == g_currentPickedRbtNode) &&
       !(*viewNodes[viewMode] == g_currentPickedRbtNode && viewMode != Skycam) &&
       !(viewMode != Skycam && g_currentPickedRbtNode == g_skyNode)){
         m = RigTForm(Cvec3(0, 0, -dy) * g_arcballScale);
    } else {
      m = RigTForm(Cvec3(0, 0, -dy) * 0.01);
    }
  }

  if(g_currentPickedRbtNode == *viewNodes[viewMode]){
    // ego motion for undifined picked node
    if(skySkyMode || (viewMode != Skycam)){

      a = eyeMat;
      if (g_mouseLClickButton && !g_mouseRClickButton)
        m = inv(m);

      g_currentPickedRbtNode->setRbt(a * m * inv(a) * g_currentPickedRbtNode->getRbt());

    }else if(!skySkyMode && (viewMode == Skycam)){
      // world-sky frame
      a = RigTForm(eyeMat.getRotation());
      m = inv(m);
      g_currentPickedRbtNode->setRbt(a * m * inv(a) * g_currentPickedRbtNode->getRbt());
    }
  }else{
    // arcball motion for non-ego objects
    RigTForm objectRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
    RigTForm parentRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode, 1);
    a = RigTForm((inv(parentRbt) * objectRbt).getTranslation(), (inv(parentRbt.getRotation())) * eyeMat.getRotation());
    g_currentPickedRbtNode->setRbt(a * m * inv(a) * g_currentPickedRbtNode->getRbt());
  }


  if (g_mouseClickDown) {
    //g_currentPickedRbtNode->setRbt(a * m * inv(a) * g_currentPickedRbtNode->getRbt()); // Simply right-multiply is WRONG
    glutPostRedisplay(); // we always redraw if we changed the scene
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}


static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;


  if(g_mouseLClickButton){

    if(pickingMode){
      pick();
      pickingMode = false;
      cout << "Picking Finished!\n\n";


    }
  }

  glutPostRedisplay();
}

static void frameToScene(){

  vector<shared_ptr<SgRbtNode> > rbtNodes;
  dumpSgRbtNodes(g_world, rbtNodes);

  for(int i = 0; i < rbtNodesSize; i++){
    rbtNodes[i] -> setRbt((*currentFrame)[i]);
  }

  cout << "Set frame[" << currentFrameIdx <<"] as current Scene!" << endl;
  return;
}

static void newFrame(){

  vector<shared_ptr<SgRbtNode> > rbtNodes;
  dumpSgRbtNodes(g_world, rbtNodes);
  rbtNodesSize = rbtNodes.size();
  vector<RigTForm> newFrame(rbtNodesSize);
  for(int i = 0; i < rbtNodesSize; i++){
    newFrame[i] = rbtNodes[i]->getRbt();
  }

  if(frames.size() != 0){ //If the current key frame is deﬁned,
  //create a new key frame immediately after current key frame.
    frames.insert(++currentFrame, newFrame);
    --currentFrame;
    cout << "current frame is saved as keyframe[" << ++currentFrameIdx << "] current size:" << frames.size() << endl;
  }else{// Otherwise just create a new key frame
  // Copy scene graph RBT data to the new key frame.
  // Set the current key frame to the newly created key frame.
    frames.insert(currentFrame, newFrame);
    --currentFrame;
    cout << "current frame is saved as keyframe[" << ++currentFrameIdx << "] current size:" << frames.size() << endl;

  }


  }

static void updateFrame(){

  vector<shared_ptr<SgRbtNode> > rbtNodes;

  dumpSgRbtNodes(g_world, rbtNodes);
  rbtNodesSize = rbtNodes.size();
  vector<RigTForm> newFrame(rbtNodesSize);
  for(int i = 0; i < rbtNodesSize; i++){
    newFrame[i] = rbtNodes[i]->getRbt();
  }

  list<vector<RigTForm> >::iterator temp = currentFrame;
  ++temp;
  frames.insert(temp, newFrame);
  frames.erase(currentFrame);
  currentFrame = --temp;
  cout << "current frame is updated for keyframe[" << currentFrameIdx << "]!" << endl;
}

static void deleteFrame(){
  // If the current key frame is deﬁned, delete the current key frame and do the following:
  list<vector<RigTForm> >::iterator temp = currentFrame;
  ++temp;

  cout << "Frame[" << currentFrameIdx <<"] has deleted. ";

  if(frames.size() == 0 || currentFrame == frames.begin()){
    frames.erase(currentFrame);
    currentFrame = temp; // NULL(size == 0) or next frame(begin)
  }else{
    frames.erase(currentFrame);
    currentFrame = --temp;
    currentFrameIdx--;
  }

  if(frames.size() != 0){
    frameToScene();
  }else{
    currentFrameIdx = -1;
  }
}

static void printList(){
  ofstream file("animation.txt");

  file << frames.size() << " " << rbtNodesSize << endl;
  for(list<vector<RigTForm> >::iterator iter = frames.begin(),
      end = frames.end();
      iter != end; ++iter){
    for(int i = 0; i < rbtNodesSize; i++){
      Cvec3 trans = (*iter)[i].getTranslation();
      Quat rot = (*iter)[i].getRotation();
      file << trans[0] <<" "<< trans[1]<<" "<<trans[2]<<" "<<rot[0]<<" "<< rot[1]<<" "<< rot[2]<<" "<< rot[3] << endl;
    }
  }
  file.close();
  cout << "Writing Finished! : animation.txt" << endl;
}

static void readList(){
  ifstream file("animation.txt");

  if(!file.good()){
    cout << "Failed to open file: no matching file exists." << endl;
    return;
  }

  int framesSize;
  file >> framesSize >> rbtNodesSize;

  frames.clear();

  for(int i = 0; i < framesSize; i++){
    vector<RigTForm> newFrame(rbtNodesSize);

    for(int j = 0; j < rbtNodesSize; j++){
      Cvec3 trans = Cvec3();
      Quat rot = Quat();

      file >> trans[0] >> trans[1] >> trans[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
      newFrame[j] = RigTForm(trans, rot);
    }
    frames.push_back(newFrame);
  }
  file.close();

  cout << "Reading Finished! : animation.txt"<< endl;
  currentFrame = frames.begin();
  currentFrameIdx = 0;
  frameToScene();

}

// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if we are at the end of the animation
// sequence, or false otherwise.

bool interpolateAndDisplay(float t) {

  //cout << t << " " << frames.size()<< endl;

  if(t >= frames.size()-3-CS175_EPS){
    //cout << "SADfasdf" << endl;

    return true;
  }
  int startKeyIdx = floor(t) + 1;
  float alpha = t - floor(t);
  list<vector<RigTForm> >::iterator startFrame = frames.begin();

  for(int i = 0; i<startKeyIdx; i++){
    startFrame++;
  }
  // now, startFrame points the start keyframe
  list<vector<RigTForm> >::iterator endFrame = startFrame;
  endFrame++;

  list<vector<RigTForm> >::iterator prevFrame = startFrame;
  prevFrame--;

  list<vector<RigTForm> >::iterator nextFrame = endFrame;
  nextFrame++;

  // now, endFrame points the end keyframe

  vector<shared_ptr<SgRbtNode> > rbtNodes;
  dumpSgRbtNodes(g_world, rbtNodes);

  for(int i = 0; i < rbtNodesSize; i++){
    rbtNodes[i] -> setRbt(catmullRom((*prevFrame)[i], (*startFrame)[i], (*endFrame)[i], (*nextFrame)[i], alpha));
    //rbtNodes[i] -> setRbt(slerp((*startFrame)[i], (*endFrame)[i], alpha));
  }

  glutPostRedisplay();
  return false;
}

// Interpret "ms" as milliseconds into the animation
static void animateTimerCallback(int ms) {
  float t = (float)ms/(float)g_msBetweenKeyFrames;

  bool endReached = interpolateAndDisplay(t);

  //cout << endReached << animating << endl;
  if (!endReached && animating)
    glutTimerFunc(1000/g_animateFramesPerSecond, animateTimerCallback, ms + 1000/g_animateFramesPerSecond);
  else {
    cout << "Finished animating... go to the end of animation." << endl;
    animating = false;
    currentFrame = frames.end();
    --currentFrame;
    --currentFrame;
    currentFrameIdx = frames.size() - 2;
    frameToScene();
    glutPostRedisplay();
    return;
  }

}

// 시간이 bin 파일과 다르게 흐르는데 이에 대한 채점 여하 묻기

static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "o\t\tCycle object to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'v':
    if(*viewNodes[viewMode] == g_currentPickedRbtNode){
      viewMode = static_cast<Mode>((viewMode+1)%3);
      g_currentPickedRbtNode = *viewNodes[viewMode];
    }else{
      viewMode = static_cast<Mode>((viewMode+1)%3);
    }

    cout << "viewMode : ";
    if(viewMode == Robot1) cout << "robot 1"<< endl;
    else if(viewMode == Robot2) cout << "robot 2"<< endl;
    else cout << "sky camera" << endl;
    break;
  case 'm':
      g_arcballScaleUpdate = true;
      skySkyMode = !skySkyMode;
      cout << "modifying skyCam in skyCam view : ";
      if(skySkyMode) cout << "sky-sky frame" << endl;
      else cout << "world-sky frame" << endl;
      if(!(skySkyMode && viewMode == Skycam && *viewNodes[viewMode] == g_currentPickedRbtNode) &&
         !(*viewNodes[viewMode] == g_currentPickedRbtNode && viewMode != Skycam) &&
         !(viewMode != Skycam && g_currentPickedRbtNode == g_skyNode)){
           cout<< "arcball mode!" << endl;
           g_arcballMode = true;
         }else {
           g_arcballMode = false;
         }
      cout<<endl;
    break;
  case 'p':
      pickingMode = !pickingMode;
      if(pickingMode) cout << "Pick Me!!!\n";
      else cout << "Dont' Pick Me!!!\n";
    break;
  case ' ': // Space : Copy current key frame RBT data to the scene graph if the current key frame is deﬁned (i.e., the list of frames is not empty).
    if(frames.size() == 0){
      cout << "Empty Frame list!" << endl;
      break;
    }else{
      frameToScene();
      break;
    }

  case 'u':  // update:
    if(frames.size() != 0){ //if the current key frame is deﬁned (i.e., the list of frames is not empty).
      // Copy the scene graph RBT data to the current key frame
      updateFrame();
      break;
    }else{ // If the list of frames is empty, apply the action corresponding to hotkey “n”
      newFrame();
      break;
    }
  case 'n':  // “n” : If the current key frame is deﬁned, create a new key frame immediately after current key frame.
    // Otherwise just create a new key frame. Copy scene graph RBT data to the new key frame. Set the current key frame to the newly created key frame.
    newFrame();
    break;

  case '>':  // “>” : advance to next key frame (if possible). Then copy current key frame data to the scene graph.
    if(frames.size() == 0){
      cout << "Empty Frame list!" << endl;
      break;
    }else if(++currentFrame == frames.end()){
      cout << "This frame is the last frame. Please set next frame first!" << endl;
      --currentFrame;
      break;
    }else{
      currentFrameIdx++;
      cout << "Step forward. ";
      frameToScene();
      break;
    }
  case '<':  // “<” : retreat to previous key frame (if possible). Then copy current key frame data to scene graph.
    if(frames.size() == 0){
      cout << "Empty Frame list!" << endl;
      break;
    }else if(currentFrame == frames.begin()){
      cout << "This frame is the first frame. Please set previous frame first!" << endl;
      break;
    }else{
      --currentFrame;
      currentFrameIdx--;
      cout << "Step backward. ";
      frameToScene();
      break;
    }
  case 'd':  // “d” : delete
    if(frames.size() == 0){
      cout << "Empty Frame list!" << endl;
      break;

    }else{
      deleteFrame();
      break;
    }
  case 'i':  // “i” : input key frames from input ﬁle. (You are free to choose your own ﬁle format.) Set current key frame to the ﬁrst frame. Copy this frame to the scene graph.
    readList();
    break;
  case 'w':  // “w” : output key frames to output ﬁle. Make sure ﬁle format is consistent with input format.
    if(frames.size() == 0){
      cout << "Empty Frame list!" << endl;
      break;
    }else{
      printList();
      break;
    }

  case 'y':
    if(frames.size() < 4){
      cout << "Need more or equal than 4 key frames in order to animate." << endl;
      break;
    }else{
      if(animating){
        cout << "Force Stop animating... go to the end of animation." << endl;
        currentFrame = frames.end();
        --currentFrame;
        --currentFrame;
        currentFrameIdx = frames.size() - 2;
        frameToScene();
        animating = false;
      }else{
        cout << "Start animating..." << endl;
        animating = true;
        g_clock = 0;
        animateTimerCallback(g_clock);

      }
      break;
    }
  case '+': // hard-coded unit variance for 100ms since sample exe file uses it
    if(g_msBetweenKeyFrames == 100){
      cout << "Cannot decrease g_msBetweenKeyFrames lower than 100" <<endl;
    }else{
      g_msBetweenKeyFrames -= 100;
      cout << "g_msBetweenKeyFrames decreased 100 : " << g_msBetweenKeyFrames << "ms" <<endl;
    }
    break;
  case '-':
    g_msBetweenKeyFrames += 100;
    cout << "g_msBetweenKeyFrames increased 100 : " << g_msBetweenKeyFrames << "ms" <<endl;
    break;
  }

  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 2");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

  // normal mapping material
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
  initGround();
  initCubes();
  initSphere();
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {

  const double ARM_LEN = 0.7,
               ARM_THICK = 0.2,
               LEG_LEN = 1.0,
               LEG_THICK = 0.3,
               HEAD_SIZE = 0.6,
               TORSO_LEN = 1.5,
               TORSO_THICK = 0.25,
               TORSO_WIDTH = 1;
  const int NUM_JOINTS = 10,
            NUM_SHAPES = 10;

  struct JointDesc {
    int parent;
    float x, y, z;
  };

  JointDesc jointDesc[NUM_JOINTS] = {
    {-1}, // torso
    {0,  TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper right arm
    {1,  ARM_LEN/2, 0, 0}, // lower right arm
    {0, -TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper left arm
    {3, -ARM_LEN/2, 0, 0}, // lower left arm
    {0, TORSO_WIDTH/2 - LEG_THICK/2, -TORSO_LEN/2, 0}, // upper right leg
    {5, 0, - LEG_LEN/2, 0}, // lower right leg
    {0, -TORSO_WIDTH/2 + LEG_THICK/2, -TORSO_LEN/2, 0}, // upper lef leg
    {7, 0, - LEG_LEN/2, 0}, // lower left leg
    {0, 0, TORSO_LEN/2, 0} // head

  };

  struct ShapeDesc {
    int parentJointId;
    float x, y, z, sx, sy, sz;
    shared_ptr<Geometry> geometry;
  };

  ShapeDesc shapeDesc[NUM_SHAPES] = {
    {0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
    {1, ARM_LEN/2, 0, 0, ARM_LEN/2, ARM_THICK/1.5, ARM_THICK/1.5, g_sphere}, // upper right arm
    {2, ARM_LEN, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
    {3, -ARM_LEN/2, 0, 0, ARM_LEN/2, ARM_THICK/1.5, ARM_THICK/1.5, g_sphere}, // upper left arm
    {4, -ARM_LEN, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower lef arm
    {5, 0, -LEG_LEN/2, 0, LEG_THICK/1.5, LEG_LEN/2, LEG_THICK/1.5, g_sphere}, // upper right leg
    {6, 0, -LEG_LEN, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
    {7, 0, -LEG_LEN/2, 0, LEG_THICK/1.5, LEG_LEN/2, LEG_THICK/1.5, g_sphere}, // upper left leg
    {8, 0, -LEG_LEN, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower left leg
    {9, 0, HEAD_SIZE/4, 0, HEAD_SIZE, HEAD_SIZE, HEAD_SIZE, g_cube} // head
  };

  shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (jointDesc[i].parent == -1)
      jointNodes[i] = base;
    else {
      jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
      jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
    }
  }
  for (int i = 0; i < NUM_SHAPES; ++i) {
    shared_ptr<SgGeometryShapeNode> shape(
      new MyShapeNode(shapeDesc[i].geometry,
                      material, // USE MATERIAL as opposed to color
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                        new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

  g_light1Node.reset(new SgRbtNode(RigTForm(Cvec3(3,8,-4))));
  g_light2Node.reset(new SgRbtNode(RigTForm(Cvec3(-4,1,1))));
  g_light1Node->addChild(shared_ptr<MyShapeNode>(
    new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0))
  ));
  g_light2Node->addChild(shared_ptr<MyShapeNode>(
    new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0))
  ));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
  constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

  g_currentPickedRbtNode = g_skyNode;

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
  g_world->addChild(g_light1Node);
  g_world->addChild(g_light2Node);
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initMaterials();
    initGeometry();
    initScene();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
